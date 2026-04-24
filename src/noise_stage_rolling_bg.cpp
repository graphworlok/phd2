/*
 *  noise_stage_rolling_bg.cpp
 *  PHD Guiding
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#include "phd.h"
#include "noise_stage_rolling_bg.h"
#include "astrometry_solver.h"

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
      m_frameCount(0),
      m_diagnostics(false),
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

void RollingBackgroundStage::LoadConfig()
{
    const wxString p = wxT("/NoisePipeline/RollingBackground/");
    m_enabled       = pConfig->Profile.GetBoolean(p + wxT("Enabled"), false);
    m_alpha         = (float) pConfig->Profile.GetDouble(p + wxT("Alpha"), m_alpha);
    m_outlierSigma  = (float) pConfig->Profile.GetDouble(p + wxT("OutlierSigma"), m_outlierSigma);
    m_maskRadius    = pConfig->Profile.GetInt(p + wxT("MaskRadius"), m_maskRadius);
    m_minSamples    = pConfig->Profile.GetInt(p + wxT("MinSamples"), m_minSamples);
    m_warmupSamples = pConfig->Profile.GetInt(p + wxT("WarmupSamples"), m_warmupSamples);
    m_starQuantile  = (float) pConfig->Profile.GetDouble(p + wxT("StarQuantile"), m_starQuantile);
    m_floorAtZero   = pConfig->Profile.GetBoolean(p + wxT("FloorAtZero"), m_floorAtZero);
    m_diagnostics   = pConfig->Profile.GetBoolean(p + wxT("Diagnostics"), false);
}

void RollingBackgroundStage::SaveConfig()
{
    const wxString p = wxT("/NoisePipeline/RollingBackground/");
    pConfig->Profile.SetBoolean(p + wxT("Enabled"), m_enabled);
    pConfig->Profile.SetDouble(p + wxT("Alpha"), m_alpha);
    pConfig->Profile.SetDouble(p + wxT("OutlierSigma"), m_outlierSigma);
    pConfig->Profile.SetInt(p + wxT("MaskRadius"), m_maskRadius);
    pConfig->Profile.SetInt(p + wxT("MinSamples"), m_minSamples);
    pConfig->Profile.SetInt(p + wxT("WarmupSamples"), m_warmupSamples);
    pConfig->Profile.SetDouble(p + wxT("StarQuantile"), m_starQuantile);
    pConfig->Profile.SetBoolean(p + wxT("FloorAtZero"), m_floorAtZero);
    pConfig->Profile.SetBoolean(p + wxT("Diagnostics"), m_diagnostics);
}

void RollingBackgroundStage::Reset()
{
    std::fill(m_mean.begin(), m_mean.end(), 0.0f);
    std::fill(m_mean2.begin(), m_mean2.end(), 0.0f);
    std::fill(m_samples.begin(), m_samples.end(), (uint16_t) 0);
    m_frameCount = 0;
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

void RollingBackgroundStage::Allocate(const wxSize& sz)
{
    if (sz == m_size && !m_mean.empty())
        return;

    // Sensor geometry changed (e.g. user changed resolution). Drop state.
    m_size = sz;
    const size_t n = (size_t) sz.GetWidth() * (size_t) sz.GetHeight();
    m_mean.assign(n, 0.0f);
    m_mean2.assign(n, 0.0f);
    m_samples.assign(n, (uint16_t) 0);
    m_frameCount = 0;
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
        // Small pedestal keeps the median above zero so downstream
        // statistics and display stretches behave naturally.
        constexpr float PEDESTAL = 32.0f;
        for (size_t i = 0; i < n; ++i)
        {
            const float v = (float) pix[i] - bg[i] + PEDESTAL;
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

    // During warmup: larger effective alpha so the model converges quickly
    // (1 / (n+1) gives perfect arithmetic mean for the first few frames).
    const float a_steady = m_alpha;
    const float a_warm   = 1.0f / (float) std::max(1, std::min(m_frameCount + 1, m_warmupSamples));
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
