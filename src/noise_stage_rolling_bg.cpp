/*
 *  noise_stage_rolling_bg.cpp
 *  PHD Guiding
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#include "phd.h"
#include "noise_stage_rolling_bg.h"
#include "astrometry_solver.h"
#include "camera.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstring>

// ---------------------------------------------------------------------------
// Construction / config
// ---------------------------------------------------------------------------

RollingBackgroundStage::RollingBackgroundStage()
    : m_alpha(0.04f),
      m_outlierSigma(4.0f),
      m_maskRadius(5),
      m_minSamples(8),
      m_warmupSamples(16),
      m_starQuantile(0.995f),
      m_floorAtZero(true),
      m_skipInitialFrames(2),
      m_frameCount(0),
      m_diagnostics(false),
      m_persistModel(false),
      m_checkpointInterval(256),
      m_diagMaskedCat(0),
      m_diagMaskedQuantile(0),
      m_diagOutliers(0),
      m_diagUsed(0),
      m_diagAlpha(0.0f),
      m_diagUpperThresh(0.0f),
      m_diagHaveCatalog(false)
{
    m_enabled = false; // opt-in
}

// Layered config helpers: read profile-scoped key first as the fallback
// default, then let a per-camera key override it. SaveConfig writes
// both, so the profile copy stays useful as the default for a future
// session where the camera id is not available (or for an unidentified
// camera in the same profile).
namespace
{

wxString PerCameraPrefix()
{
    if (!pCamera)
        return wxString();
    const wxString tag = GuideCamera::HardwareIdToFileTag(pCamera->GetHardwareId());
    if (tag.empty())
        return wxString();
    return wxT("/NoisePipeline/Cameras/") + tag + wxT("/RollingBackground/");
}

} // namespace

void RollingBackgroundStage::LoadConfig()
{
    const wxString pp = wxT("/NoisePipeline/RollingBackground/");
    const wxString cp = PerCameraPrefix();

    auto loadBool = [&](const wxString& key, bool def) -> bool {
        bool v = pConfig->Profile.GetBoolean(pp + key, def);
        if (!cp.empty())
            v = pConfig->Profile.GetBoolean(cp + key, v);
        return v;
    };
    auto loadInt = [&](const wxString& key, int def) -> int {
        int v = pConfig->Profile.GetInt(pp + key, def);
        if (!cp.empty())
            v = pConfig->Profile.GetInt(cp + key, v);
        return v;
    };
    auto loadDbl = [&](const wxString& key, double def) -> double {
        double v = pConfig->Profile.GetDouble(pp + key, def);
        if (!cp.empty())
            v = pConfig->Profile.GetDouble(cp + key, v);
        return v;
    };

    m_enabled            = loadBool(wxT("Enabled"),            false);
    m_alpha              = (float) loadDbl(wxT("Alpha"),       m_alpha);
    m_outlierSigma       = (float) loadDbl(wxT("OutlierSigma"),m_outlierSigma);
    m_maskRadius         = loadInt(wxT("MaskRadius"),          m_maskRadius);
    m_minSamples         = loadInt(wxT("MinSamples"),          m_minSamples);
    m_warmupSamples      = loadInt(wxT("WarmupSamples"),       m_warmupSamples);
    m_starQuantile       = (float) loadDbl(wxT("StarQuantile"),m_starQuantile);
    m_floorAtZero        = loadBool(wxT("FloorAtZero"),        m_floorAtZero);
    m_skipInitialFrames  = loadInt(wxT("SkipInitialFrames"),   m_skipInitialFrames);
    m_persistModel       = loadBool(wxT("PersistModel"),       false);
    m_checkpointInterval = loadInt(wxT("CheckpointInterval"),  m_checkpointInterval);
    m_diagnostics        = loadBool(wxT("Diagnostics"),        false);
}

void RollingBackgroundStage::SaveConfig()
{
    const wxString pp = wxT("/NoisePipeline/RollingBackground/");
    const wxString cp = PerCameraPrefix();

    auto saveBool = [&](const wxString& key, bool v) {
        pConfig->Profile.SetBoolean(pp + key, v);
        if (!cp.empty())
            pConfig->Profile.SetBoolean(cp + key, v);
    };
    auto saveInt = [&](const wxString& key, int v) {
        pConfig->Profile.SetInt(pp + key, v);
        if (!cp.empty())
            pConfig->Profile.SetInt(cp + key, v);
    };
    auto saveDbl = [&](const wxString& key, double v) {
        pConfig->Profile.SetDouble(pp + key, v);
        if (!cp.empty())
            pConfig->Profile.SetDouble(cp + key, v);
    };

    saveBool(wxT("Enabled"),            m_enabled);
    saveDbl (wxT("Alpha"),               m_alpha);
    saveDbl (wxT("OutlierSigma"),        m_outlierSigma);
    saveInt (wxT("MaskRadius"),          m_maskRadius);
    saveInt (wxT("MinSamples"),          m_minSamples);
    saveInt (wxT("WarmupSamples"),       m_warmupSamples);
    saveDbl (wxT("StarQuantile"),        m_starQuantile);
    saveBool(wxT("FloorAtZero"),         m_floorAtZero);
    saveInt (wxT("SkipInitialFrames"),   m_skipInitialFrames);
    saveBool(wxT("PersistModel"),        m_persistModel);
    saveInt (wxT("CheckpointInterval"),  m_checkpointInterval);
    saveBool(wxT("Diagnostics"),         m_diagnostics);
}

void RollingBackgroundStage::Reset()
{
    std::fill(m_mean.begin(), m_mean.end(), 0.0f);
    std::fill(m_mean2.begin(), m_mean2.end(), 0.0f);
    std::fill(m_samples.begin(), m_samples.end(), (uint16_t) 0);
    m_frameCount = 0;

    // If a per-camera model file exists for this camera, drop it so the
    // user-visible "Reset" really starts fresh.
    if (m_persistModel)
    {
        const wxString tag = CurrentCameraTag();
        if (!tag.empty())
            DeleteModelOnDisk(tag);
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

void RollingBackgroundStage::Allocate(const wxSize& sz)
{
    const wxString currentTag = CurrentCameraTag();

    // No-op if both the geometry AND the camera identity match what we
    // already have - the in-memory model is still valid.
    if (sz == m_size && !m_mean.empty() && currentTag == m_modelTag)
        return;

    // Sensor geometry or camera changed. Drop state.
    m_size = sz;
    m_modelTag = currentTag;
    const size_t n = (size_t) sz.GetWidth() * (size_t) sz.GetHeight();
    m_mean.assign(n, 0.0f);
    m_mean2.assign(n, 0.0f);
    m_samples.assign(n, (uint16_t) 0);
    m_frameCount = 0;

    // Try to load a saved model for this camera. Silently skips if
    // persistence is off, no hardware id is available, or the file
    // doesn't exist / fails validation.
    if (m_persistModel && !currentTag.empty())
    {
        if (LoadModelFromDisk(currentTag, sz))
        {
            Debug.Write(wxString::Format(
                "RollingBackground: loaded saved model for camera %s, %d frames\n",
                currentTag, m_frameCount));
        }
    }
}

void RollingBackgroundStage::BuildCatalogMask(const PlateSolveResult *solve,
                                              std::vector<uint8_t>& mask) const
{
    const int W = m_size.GetWidth();
    const int H = m_size.GetHeight();
    mask.assign((size_t) W * (size_t) H, (uint8_t) 0);

    if (!solve || !solve->success || solve->stars.empty())
        return;

    const int r  = std::max(1, m_maskRadius);
    const int r2 = r * r;

    for (const SolvedStar& s : solve->stars)
    {
        const int cx = (int) std::lround(s.pixel_x);
        const int cy = (int) std::lround(s.pixel_y);
        const int x0 = std::max(0, cx - r);
        const int x1 = std::min(W - 1, cx + r);
        const int y0 = std::max(0, cy - r);
        const int y1 = std::min(H - 1, cy + r);

        for (int y = y0; y <= y1; ++y)
        {
            const int dy = y - cy;
            uint8_t *row = mask.data() + (size_t) y * (size_t) W;
            for (int x = x0; x <= x1; ++x)
            {
                const int dx = x - cx;
                if (dx * dx + dy * dy <= r2)
                    row[x] = 1;
            }
        }
    }
}

// Fallback "probably-a-star" gate when no plate solution is available.
// Simple global quantile over a coarse histogram - fast, and good enough:
// the per-pixel EMA will converge on sensor-stationary signal regardless,
// this gate just reduces bias from bright stars on early frames.
float RollingBackgroundStage::HeuristicUpperThreshold(const usImage& img) const
{
    constexpr int BINS = 512;
    unsigned int hist[BINS];
    std::memset(hist, 0, sizeof(hist));

    const unsigned short *p = img.ImageData;
    const size_t n = img.NPixels;
    for (size_t i = 0; i < n; ++i)
    {
        unsigned int b = (unsigned int) p[i] >> 7; // 65536 / 512 = 128
        if (b >= BINS) b = BINS - 1;
        hist[b]++;
    }
    const size_t target = (size_t) ((double) n * (double) m_starQuantile);
    size_t running = 0;
    for (int b = 0; b < BINS; ++b)
    {
        running += hist[b];
        if (running >= target)
            return (float) ((b + 1) << 7);
    }
    return 65535.0f;
}

// ---------------------------------------------------------------------------
// Diagnostic stats helpers (only used when m_diagnostics is set)
// ---------------------------------------------------------------------------

namespace
{

struct FrameStats
{
    float          mean;
    float          sigma;         // stddev about the mean
    float          robust_sigma;  // (p84 - p16) / 2 - robust Gaussian-equivalent
    unsigned int   p10;
    unsigned int   p50;
    unsigned int   p90;
    unsigned short minv;
    unsigned short maxv;
};

static void ComputeFrameStats(const unsigned short *p, size_t n, FrameStats& s)
{
    constexpr int BINS = 4096; // 16 ADU per bin
    uint32_t hist[BINS];
    std::memset(hist, 0, sizeof(hist));

    double sum = 0.0, sumSq = 0.0;
    unsigned short mn = 65535, mx = 0;
    for (size_t i = 0; i < n; ++i)
    {
        unsigned short v = p[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum   += (double) v;
        sumSq += (double) v * (double) v;
        unsigned int b = (unsigned int) v >> 4;
        if (b >= BINS) b = BINS - 1;
        hist[b]++;
    }

    s.minv = mn;
    s.maxv = mx;
    s.mean = (float) (sum / (double) n);
    const double var = sumSq / (double) n - (double) s.mean * (double) s.mean;
    s.sigma = (float) std::sqrt(std::max(0.0, var));

    // percentiles p10, p16, p50, p84, p90 from cumulative histogram
    const size_t t10 = (size_t) (0.10 * (double) n);
    const size_t t16 = (size_t) (0.16 * (double) n);
    const size_t t50 = (size_t) (0.50 * (double) n);
    const size_t t84 = (size_t) (0.84 * (double) n);
    const size_t t90 = (size_t) (0.90 * (double) n);
    size_t running = 0;
    int b10 = -1, b16 = -1, b50 = -1, b84 = -1, b90 = -1;
    for (int b = 0; b < BINS; ++b)
    {
        running += hist[b];
        if (b10 < 0 && running >= t10) b10 = b;
        if (b16 < 0 && running >= t16) b16 = b;
        if (b50 < 0 && running >= t50) b50 = b;
        if (b84 < 0 && running >= t84) b84 = b;
        if (b90 < 0 && running >= t90) { b90 = b; break; }
    }
    auto binAdu = [](int b) { return (b < 0 ? 0u : (unsigned int) (b + 1) << 4); };
    s.p10 = binAdu(b10);
    s.p50 = binAdu(b50);
    s.p90 = binAdu(b90);
    const unsigned int p16 = binAdu(b16);
    const unsigned int p84 = binAdu(b84);
    s.robust_sigma = (float) ((double) (p84 > p16 ? p84 - p16 : 0u) * 0.5);
}

struct ModelStats
{
    float  mean;
    float  sigma;
    float  coverage;   // fraction of pixels with count >= minSamples
    int    min_n;
    int    max_n;
    double mean_n;
};

static void ComputeModelStats(const float *m, const uint16_t *cnt, size_t n,
                              int minSamples, ModelStats& s)
{
    double sum = 0.0, sumSq = 0.0;
    size_t covered = 0;
    uint64_t nSum = 0;
    int nMin = INT_MAX;
    int nMax = 0;
    for (size_t i = 0; i < n; ++i)
    {
        sum   += (double) m[i];
        sumSq += (double) m[i] * (double) m[i];
        const int c = (int) cnt[i];
        nSum += (uint64_t) c;
        if (c < nMin) nMin = c;
        if (c > nMax) nMax = c;
        if (c >= minSamples) ++covered;
    }
    s.mean = (float) (sum / (double) n);
    const double var = sumSq / (double) n - (double) s.mean * (double) s.mean;
    s.sigma    = (float) std::sqrt(std::max(0.0, var));
    s.coverage = (float) ((double) covered / (double) n);
    s.min_n    = (nMin == INT_MAX ? 0 : nMin);
    s.max_n    = nMax;
    s.mean_n   = (double) nSum / (double) n;
}

} // namespace

// ---------------------------------------------------------------------------
// Apply: subtract the current model from `img`.
// ---------------------------------------------------------------------------

bool RollingBackgroundStage::Apply(usImage& img, const PlateSolveResult * /*solve*/)
{
    if (!m_enabled || !img.ImageData)
        return false;

    Allocate(img.Size);

    unsigned short *pix = img.ImageData;
    const float *bg     = m_mean.data();
    const size_t n      = img.NPixels;

    FrameStats in_stats;
    const bool diag = m_diagnostics;
    if (diag)
        ComputeFrameStats(pix, n, in_stats);

    // Don't start subtracting until we've observed enough frames.
    if (m_frameCount < m_minSamples)
    {
        if (diag)
        {
            ModelStats ms;
            ComputeModelStats(m_mean.data(), m_samples.data(), n, m_minSamples, ms);
            size_t clamped = 0;
            Debug.Write(wxString::Format(
                "NoiseDiag,frame=%d,size=%dx%d,state=warmup,"
                "in_mean=%.2f,in_sigma=%.2f,in_robust=%.2f,"
                "in_p10=%u,in_p50=%u,in_p90=%u,in_min=%u,in_max=%u,"
                "out_mean=%.2f,out_sigma=%.2f,out_robust=%.2f,"
                "out_p50=%u,clamped_zero=%zu,"
                "model_mean=%.2f,model_sigma=%.2f,model_cov=%.3f,"
                "model_n_min=%d,model_n_max=%d,model_n_mean=%.2f,"
                "masked_cat=%zu,masked_quant=%zu,outliers=%zu,used=%zu,"
                "alpha=%.4f,upper_thresh=%.0f,have_catalog=%d\n",
                m_frameCount, m_size.GetWidth(), m_size.GetHeight(),
                in_stats.mean, in_stats.sigma, in_stats.robust_sigma,
                in_stats.p10, in_stats.p50, in_stats.p90, in_stats.minv, in_stats.maxv,
                in_stats.mean, in_stats.sigma, in_stats.robust_sigma,
                in_stats.p50, clamped,
                ms.mean, ms.sigma, ms.coverage, ms.min_n, ms.max_n, ms.mean_n,
                m_diagMaskedCat, m_diagMaskedQuantile, m_diagOutliers, m_diagUsed,
                m_diagAlpha, m_diagUpperThresh, m_diagHaveCatalog ? 1 : 0));
        }
        return false;
    }

    size_t clamped = 0;
    if (m_floorAtZero)
    {
        for (size_t i = 0; i < n; ++i)
        {
            const float v = (float) pix[i] - bg[i];
            if (v <= 0.0f) { pix[i] = 0; ++clamped; }
            else if (v >= 65535.0f) pix[i] = 65535;
            else pix[i] = (unsigned short) v;
        }
    }
    else
    {
        // Flat-field semantics: subtract the spatial model but add back
        // its scalar mean so global brightness is preserved. This flattens
        // vignetting, amp glow and FPN without darkening the frame.
        double sum = 0.0;
        for (size_t i = 0; i < n; ++i)
            sum += (double) bg[i];
        const float pedestal = (float) (sum / (double) n);

        for (size_t i = 0; i < n; ++i)
        {
            const float v = (float) pix[i] - bg[i] + pedestal;
            if (v <= 0.0f) { pix[i] = 0; ++clamped; }
            else if (v >= 65535.0f) pix[i] = 65535;
            else pix[i] = (unsigned short) v;
        }
    }

    if (diag)
    {
        FrameStats out_stats;
        ComputeFrameStats(pix, n, out_stats);
        ModelStats ms;
        ComputeModelStats(m_mean.data(), m_samples.data(), n, m_minSamples, ms);
        Debug.Write(wxString::Format(
            "NoiseDiag,frame=%d,size=%dx%d,state=subtracting,"
            "in_mean=%.2f,in_sigma=%.2f,in_robust=%.2f,"
            "in_p10=%u,in_p50=%u,in_p90=%u,in_min=%u,in_max=%u,"
            "out_mean=%.2f,out_sigma=%.2f,out_robust=%.2f,"
            "out_p50=%u,clamped_zero=%zu,"
            "model_mean=%.2f,model_sigma=%.2f,model_cov=%.3f,"
            "model_n_min=%d,model_n_max=%d,model_n_mean=%.2f,"
            "masked_cat=%zu,masked_quant=%zu,outliers=%zu,used=%zu,"
            "alpha=%.4f,upper_thresh=%.0f,have_catalog=%d\n",
            m_frameCount, m_size.GetWidth(), m_size.GetHeight(),
            in_stats.mean, in_stats.sigma, in_stats.robust_sigma,
            in_stats.p10, in_stats.p50, in_stats.p90, in_stats.minv, in_stats.maxv,
            out_stats.mean, out_stats.sigma, out_stats.robust_sigma,
            out_stats.p50, clamped,
            ms.mean, ms.sigma, ms.coverage, ms.min_n, ms.max_n, ms.mean_n,
            m_diagMaskedCat, m_diagMaskedQuantile, m_diagOutliers, m_diagUsed,
            m_diagAlpha, m_diagUpperThresh, m_diagHaveCatalog ? 1 : 0));
    }
    return true;
}

// ---------------------------------------------------------------------------
// Observe: update the per-pixel EMA from `img`.
// ---------------------------------------------------------------------------

void RollingBackgroundStage::Observe(const usImage& img, const PlateSolveResult *solve)
{
    if (!m_enabled || !img.ImageData)
        return;

    Allocate(img.Size);

    // Discard the leading frames so the camera AGC has time to settle.
    // Without this, frame 0 (often far from the steady-state mean) is
    // accepted at warmup alpha = 1.0 and poisons the entire model.
    if (m_frameCount < m_skipInitialFrames)
    {
        m_diagMaskedCat = m_diagMaskedQuantile = m_diagOutliers = m_diagUsed = 0;
        m_diagAlpha = 0.0f;
        m_diagUpperThresh = 0.0f;
        m_diagHaveCatalog = false;
        ++m_frameCount;
        return;
    }

    // During warmup: larger effective alpha so the model converges quickly
    // (1 / (n+1) gives perfect arithmetic mean for the first few frames).
    // Index relative to the first *accepted* frame so the leading skipped
    // frames do not eat into the warmup budget.
    const int observedIdx = m_frameCount - m_skipInitialFrames;
    const float a_steady = m_alpha;
    const float a_warm   = 1.0f / (float) std::max(1, std::min(observedIdx + 1, m_warmupSamples));
    const float alpha    = std::max(a_steady, a_warm);

    std::vector<uint8_t> starMask;
    BuildCatalogMask(solve, starMask);
    const bool haveCatalog = solve && solve->success && !solve->stars.empty();
    const uint8_t *mask = starMask.empty() ? nullptr : starMask.data();

    const float upperThresh = haveCatalog ? 65535.0f : HeuristicUpperThreshold(img);

    const size_t n           = img.NPixels;
    const unsigned short *p  = img.ImageData;
    float    *mean           = m_mean.data();
    float    *mean2          = m_mean2.data();
    uint16_t *cnt            = m_samples.data();

    // Per-frame counters consumed by the next Apply() diagnostic line.
    size_t cMaskedCat = 0, cMaskedQuant = 0, cOutliers = 0, cUsed = 0;

    for (size_t i = 0; i < n; ++i)
    {
        if (mask && mask[i])
        {
            ++cMaskedCat;
            continue;
        }

        const float v = (float) p[i];

        // Heuristic gate only when we don't have catalog masking.
        if (!haveCatalog && v > upperThresh)
        {
            ++cMaskedQuant;
            continue;
        }

        // Outlier rejection once we have a tentative estimate.
        if (cnt[i] > 4)
        {
            const float m   = mean[i];
            const float m2  = mean2[i];
            const float var = std::max(1.0f, m2 - m * m);
            const float sd  = std::sqrt(var);
            if (std::fabs(v - m) > m_outlierSigma * sd)
            {
                ++cOutliers;
                continue;
            }
        }

        mean[i]  = (1.0f - alpha) * mean[i]  + alpha * v;
        mean2[i] = (1.0f - alpha) * mean2[i] + alpha * v * v;
        if (cnt[i] < UINT16_MAX)
            cnt[i]++;
        ++cUsed;
    }

    // Snapshot for the next Apply() to emit.
    m_diagMaskedCat      = cMaskedCat;
    m_diagMaskedQuantile = cMaskedQuant;
    m_diagOutliers       = cOutliers;
    m_diagUsed           = cUsed;
    m_diagAlpha          = alpha;
    m_diagUpperThresh    = upperThresh;
    m_diagHaveCatalog    = haveCatalog;

    ++m_frameCount;

    // Periodic checkpoint to disk so the converged model survives a
    // restart. Capped to once every CheckpointInterval frames; a value
    // of zero disables periodic saves (Reset and explicit calls still
    // work).
    if (m_persistModel && m_checkpointInterval > 0 &&
        m_frameCount > m_minSamples &&
        (m_frameCount % m_checkpointInterval) == 0)
    {
        const wxString tag = m_modelTag;
        if (!tag.empty())
            SaveModelToDisk(tag);
    }
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

int RollingBackgroundStage::MinSamples() const
{
    if (m_samples.empty())
        return 0;
    uint16_t m = UINT16_MAX;
    for (uint16_t v : m_samples)
        if (v < m)
            m = v;
    return (int) m;
}

// ---------------------------------------------------------------------------
// Per-camera model persistence
//
// File layout: a multi-extension FITS at
//   GetDarksDir() / "PHD2_noise_rbg_<camTag>.fit"
// containing four HDUs:
//   1. Primary (0x0)         - metadata only: HWID, FRAMECNT, ALPHA, etc.
//   2. MEAN    image float32 - per-pixel running mean
//   3. MEAN2   image float32 - per-pixel running mean of value^2
//   4. SAMPLES image uint16  - per-pixel observation count
// ---------------------------------------------------------------------------

wxString RollingBackgroundStage::CurrentCameraTag()
{
    if (!pCamera)
        return wxString();
    return GuideCamera::HardwareIdToFileTag(pCamera->GetHardwareId());
}

wxString RollingBackgroundStage::ModelFilePath(const wxString& camTag)
{
    if (camTag.empty())
        return wxString();
    return MyFrame::GetDarksDir() + PATHSEPSTR +
           wxT("PHD2_noise_rbg_") + camTag + wxT(".fit");
}

bool RollingBackgroundStage::SaveModelToDisk(const wxString& camTag) const
{
    if (camTag.empty() || m_size.GetWidth() == 0 || m_mean.empty())
        return false;

    const wxString path = ModelFilePath(camTag);
    if (path.empty())
        return false;

    fitsfile *fptr = nullptr;
    int status = 0;
    PHD_fits_create_file(&fptr, path, true, &status);
    if (status)
    {
        Debug.Write(wxString::Format(
            "RollingBackground: SaveModel - cannot create %s, status=%d\n",
            path, status));
        return false;
    }

    // Primary HDU: empty placeholder carrying metadata.
    long primAxes[1] = { 0 };
    fits_create_img(fptr, BYTE_IMG, 0, primAxes, &status);

    const wxString hwId = pCamera ? pCamera->GetHardwareId() : wxString();
    {
        // fits_write_key(TSTRING) wants a non-const char*; copy first.
        const wxScopedCharBuffer hwBuf = hwId.utf8_str();
        char hwTmp[FLEN_VALUE] = { 0 };
        std::strncpy(hwTmp, hwBuf.data(), sizeof(hwTmp) - 1);
        fits_write_key(fptr, TSTRING, "HWID", hwTmp,
                       "Camera hardware id", &status);
    }
    int frameCount = m_frameCount;
    fits_write_key(fptr, TINT,   "FRAMECNT", &frameCount,
                   "Frames observed",      &status);
    float alpha = m_alpha;
    fits_write_key(fptr, TFLOAT, "ALPHA",    &alpha,
                   "Steady-state EMA alpha at save time", &status);
    int width  = m_size.GetWidth();
    int height = m_size.GetHeight();
    fits_write_key(fptr, TINT, "MODELW", &width,  "Model width (px)",  &status);
    fits_write_key(fptr, TINT, "MODELH", &height, "Model height (px)", &status);

    long axes[2] = { (long) width, (long) height };
    long fpixel[2] = { 1, 1 };
    long npix = (long) width * (long) height;

    auto writeImage = [&](int bitpix, int dtype, void *data, const char *name)
    {
        fits_create_img(fptr, bitpix, 2, axes, &status);
        char extName[FLEN_VALUE] = { 0 };
        std::strncpy(extName, name, sizeof(extName) - 1);
        fits_write_key(fptr, TSTRING, "EXTNAME", extName, 0, &status);
        fits_write_pix(fptr, dtype, fpixel, npix, data, &status);
    };

    writeImage(FLOAT_IMG,  TFLOAT,  (void *) m_mean.data(),    "MEAN");
    writeImage(FLOAT_IMG,  TFLOAT,  (void *) m_mean2.data(),   "MEAN2");
    writeImage(USHORT_IMG, TUSHORT, (void *) m_samples.data(), "SAMPLES");

    PHD_fits_close_file(fptr);

    if (status)
    {
        Debug.Write(wxString::Format(
            "RollingBackground: SaveModel - fitsio status=%d for %s\n",
            status, path));
        return false;
    }
    Debug.Write(wxString::Format(
        "RollingBackground: saved model %dx%d (%d frames) to %s\n",
        width, height, m_frameCount, path));
    return true;
}

bool RollingBackgroundStage::LoadModelFromDisk(const wxString& camTag,
                                               const wxSize& sz)
{
    if (camTag.empty())
        return false;
    const wxString path = ModelFilePath(camTag);
    if (path.empty() || !wxFileExists(path))
        return false;

    fitsfile *fptr = nullptr;
    int status = 0;
    if (PHD_fits_open_diskfile(&fptr, path, READONLY, &status) || status)
        return false;

    auto bail = [&](const wxString& reason) -> bool {
        Debug.Write(wxString::Format(
            "RollingBackground: LoadModel - rejecting %s (%s)\n",
            path, reason));
        int s = 0;
        if (fptr) PHD_fits_close_file(fptr);
        return false;
    };

    // Validate metadata in primary HDU.
    char hwId[FLEN_VALUE] = { 0 };
    char dummy[FLEN_COMMENT] = { 0 };
    int hwStatus = 0;
    fits_read_key(fptr, TSTRING, "HWID", hwId, dummy, &hwStatus);
    const wxString fileHwId = (hwStatus == 0) ? wxString::FromUTF8(hwId)
                                              : wxString();
    const wxString camHwId  = pCamera ? pCamera->GetHardwareId() : wxString();
    if (fileHwId.empty() || camHwId.empty() || fileHwId != camHwId)
        return bail("HWID missing or mismatch");

    int width = 0, height = 0;
    int s2 = 0;
    fits_read_key(fptr, TINT, "MODELW", &width,  dummy, &s2);
    int s3 = 0;
    fits_read_key(fptr, TINT, "MODELH", &height, dummy, &s3);
    if (s2 || s3 || width != sz.GetWidth() || height != sz.GetHeight())
        return bail("dimension mismatch");

    int frameCount = 0;
    int sFc = 0;
    fits_read_key(fptr, TINT, "FRAMECNT", &frameCount, dummy, &sFc);
    if (sFc) frameCount = 0;

    const long npix = (long) width * (long) height;
    long fpixel[2] = { 1, 1 };

    // The three image extensions are written in order MEAN, MEAN2, SAMPLES.
    // movabs_hdu uses 1-based indexing (1=primary).
    auto readExt = [&](int hduIdx, int dtype, void *dst) -> bool {
        int s = 0;
        fits_movabs_hdu(fptr, hduIdx, nullptr, &s);
        if (s) return false;
        fits_read_pix(fptr, dtype, fpixel, npix, nullptr, dst, nullptr, &s);
        return s == 0;
    };

    std::vector<float>    newMean (npix, 0.0f);
    std::vector<float>    newMean2(npix, 0.0f);
    std::vector<uint16_t> newSmpl (npix, 0);

    if (!readExt(2, TFLOAT,  newMean.data()) ||
        !readExt(3, TFLOAT,  newMean2.data()) ||
        !readExt(4, TUSHORT, newSmpl.data()))
        return bail("read failure");

    PHD_fits_close_file(fptr);

    m_mean    = std::move(newMean);
    m_mean2   = std::move(newMean2);
    m_samples = std::move(newSmpl);
    m_frameCount = frameCount;
    return true;
}

bool RollingBackgroundStage::DeleteModelOnDisk(const wxString& camTag) const
{
    if (camTag.empty())
        return false;
    const wxString path = ModelFilePath(camTag);
    if (path.empty() || !wxFileExists(path))
        return false;
    bool ok = wxRemoveFile(path);
    Debug.Write(wxString::Format(
        "RollingBackground: %s saved model at %s\n",
        ok ? "deleted" : "failed to delete", path));
    return ok;
}
