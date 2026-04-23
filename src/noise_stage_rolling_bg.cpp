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
      m_frameCount(0)
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
// Apply: subtract the current model from `img`.
// ---------------------------------------------------------------------------

bool RollingBackgroundStage::Apply(usImage& img, const PlateSolveResult * /*solve*/)
{
    if (!m_enabled || !img.ImageData)
        return false;

    Allocate(img.Size);

    // Don't start subtracting until we've observed enough frames.
    if (m_frameCount < m_minSamples)
        return false;

    unsigned short *pix = img.ImageData;
    const float *bg = m_mean.data();
    const size_t n  = img.NPixels;

    if (m_floorAtZero)
    {
        for (size_t i = 0; i < n; ++i)
        {
            const float v = (float) pix[i] - bg[i];
            pix[i] = (unsigned short) std::clamp(v, 0.0f, 65535.0f);
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
            pix[i] = (unsigned short) std::clamp(v, 0.0f, 65535.0f);
        }
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

    for (size_t i = 0; i < n; ++i)
    {
        if (mask && mask[i])
            continue;

        const float v = (float) p[i];

        // Heuristic gate only when we don't have catalog masking.
        if (!haveCatalog && v > upperThresh)
            continue;

        // Outlier rejection once we have a tentative estimate.
        if (cnt[i] > 4)
        {
            const float m   = mean[i];
            const float m2  = mean2[i];
            const float var = std::max(1.0f, m2 - m * m);
            const float sd  = std::sqrt(var);
            if (std::fabs(v - m) > m_outlierSigma * sd)
                continue;
        }

        mean[i]  = (1.0f - alpha) * mean[i]  + alpha * v;
        mean2[i] = (1.0f - alpha) * mean2[i] + alpha * v * v;
        if (cnt[i] < UINT16_MAX)
            cnt[i]++;
    }

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
