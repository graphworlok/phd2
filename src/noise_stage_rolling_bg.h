/*
 *  noise_stage_rolling_bg.h
 *  PHD Guiding
 *
 *  Rolling per-pixel background / dark model.
 *
 *  Maintains an exponential moving average of each pixel's value,
 *  collected over samples that are (a) not covered by a catalog star
 *  when a plate solution is available, and (b) not transient outliers.
 *
 *  The converged per-pixel model captures every sensor-stationary
 *  signal: bias, amp glow, optical vignetting, IR-filter reflection
 *  rings, column offsets, and hot pixels. Subtracting it from each
 *  incoming frame removes all of those at once, with no pre-captured
 *  dark library, and adapts continuously to temperature drift.
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#ifndef NOISE_STAGE_ROLLING_BG_H_INCLUDED
#define NOISE_STAGE_ROLLING_BG_H_INCLUDED

#include "noise_pipeline.h"

#include <vector>
#include <cstdint>
#include <wx/gdicmn.h>

class RollingBackgroundStage : public NoiseReductionStage
{
    wxSize m_size;

    std::vector<float>    m_mean;     // per-pixel EMA of value
    std::vector<float>    m_mean2;    // per-pixel EMA of value^2 (for sigma)
    std::vector<uint16_t> m_samples;  // per-pixel sample count (saturating)

    // Tunables
    float m_alpha;          // steady-state EMA weight for new sample (0..1)
    float m_outlierSigma;   // reject samples > N sigma from running mean
    int   m_maskRadius;     // pixels - catalog star exclusion radius
    int   m_minSamples;     // frames required before subtraction begins
    int   m_warmupSamples;  // frames during which alpha decays 1 -> m_alpha
    float m_starQuantile;   // fallback bright-pixel rejection quantile
    bool  m_floorAtZero;    // clamp output to >= 0 (else apply pedestal)
    int   m_frameCount;     // frames observed since Reset()
    bool  m_diagnostics;    // emit per-frame NoiseDiag lines to debug log

    // Counters captured during Observe, consumed by the next Apply() call
    // so a single diagnostic line can describe both halves of one frame.
    size_t m_diagMaskedCat;
    size_t m_diagMaskedQuantile;
    size_t m_diagOutliers;
    size_t m_diagUsed;
    float  m_diagAlpha;
    float  m_diagUpperThresh;
    bool   m_diagHaveCatalog;

    void Allocate(const wxSize& sz);
    void BuildCatalogMask(const PlateSolveResult *solve,
                          std::vector<uint8_t>& mask) const;
    float HeuristicUpperThreshold(const usImage& img) const;

public:
    RollingBackgroundStage();

    wxString Name() const override { return wxT("RollingBackground"); }

    bool Apply(usImage& img, const PlateSolveResult *solve) override;
    void Observe(const usImage& img, const PlateSolveResult *solve) override;

    void Reset() override;
    void LoadConfig() override;
    void SaveConfig() override;

    // Introspection
    wxSize        ModelSize()    const { return m_size; }
    const float  *ModelData()    const { return m_mean.data(); }
    int           FrameCount()   const { return m_frameCount; }
    int           MinSamples()   const; // min across all pixels (slow, diag only)

    // Accessors (for a future config dialog)
    float GetAlpha() const         { return m_alpha; }
    void  SetAlpha(float v)        { m_alpha = v; }
    float GetOutlierSigma() const  { return m_outlierSigma; }
    void  SetOutlierSigma(float v) { m_outlierSigma = v; }
    int   GetMaskRadius() const    { return m_maskRadius; }
    void  SetMaskRadius(int v)     { m_maskRadius = v; }
    int   GetMinSamples() const    { return m_minSamples; }
    void  SetMinSamples(int v)     { m_minSamples = v; }
    bool  GetDiagnostics() const   { return m_diagnostics; }
    void  SetDiagnostics(bool v)   { m_diagnostics = v; }
};

#endif // NOISE_STAGE_ROLLING_BG_H_INCLUDED
