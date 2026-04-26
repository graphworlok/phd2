/*
 *  noise_stage_hot_pixel.h
 *  PHD Guiding
 *
 *  Spatial transient hot-pixel rejection.
 *
 *  For every pixel, computes the median of its 3x3 neighbourhood. If the
 *  centre is brighter than the median by more than a tunable threshold
 *  (multiplicative ratio with an absolute-ADU floor), the centre is
 *  replaced by the neighbour median.
 *
 *  Why this exists alongside the rolling-background stage:
 *
 *    * Rolling background works in the *temporal* dimension and needs
 *      several frames before its per-pixel sigma is reliable. Until
 *      then, single-frame outliers (cosmic rays, satellite pixels,
 *      transient hot pixels from temperature spikes) are not cleaned
 *      from the displayed frame even though they are correctly excluded
 *      from the model update.
 *
 *    * This stage works in the *spatial* dimension, from frame one,
 *      with no per-pixel state. A cosmic ray is typically 10-100x the
 *      local background while its 8 neighbours are at background level,
 *      so the median ratio test catches it cleanly without flagging
 *      genuine stars (whose neighbours are also bright).
 *
 *  The default ordering in the pipeline runs this stage *before*
 *  RollingBackground so cosmic-ray spikes do not have a chance to enter
 *  the EMA outlier path on the way through.
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#ifndef NOISE_STAGE_HOT_PIXEL_H_INCLUDED
#define NOISE_STAGE_HOT_PIXEL_H_INCLUDED

#include "noise_pipeline.h"

#include <cstddef>

class HotPixelStage : public NoiseReductionStage
{
    // Multiplicative threshold: flag a pixel when its value exceeds
    // local median * m_ratio.
    float m_ratio;

    // Absolute floor: also require centre > median + m_absOffset (in
    // raw ADU, 16-bit scale). Prevents flagging in very dark regions
    // where the multiplicative ratio is unstable.
    int   m_absOffset;

    // Optional cap on number of pixels replaced per frame; if exceeded
    // we assume something is wrong (e.g. defocus, cloud, lens flap)
    // and bail out so we don't stomp on a real frame. 0 = no cap.
    int   m_maxReplacements;

    // Diagnostics on/off (writes a single line per frame to debug log).
    bool  m_diagnostics;

    // Captured during Apply for the diagnostic line.
    size_t m_diagFlagged;
    size_t m_diagAborted;

public:
    HotPixelStage();

    wxString Name() const override { return wxT("HotPixel"); }

    bool Apply(usImage& img, const PlateSolveResult *solve) override;
    // Observe is a no-op for this stage; the spatial filter is stateless.

    void LoadConfig() override;
    void SaveConfig() override;

    // Accessors for the future config dialog row.
    float GetRatio() const            { return m_ratio; }
    void  SetRatio(float v)           { m_ratio = v; }
    int   GetAbsOffset() const        { return m_absOffset; }
    void  SetAbsOffset(int v)         { m_absOffset = v; }
    int   GetMaxReplacements() const  { return m_maxReplacements; }
    void  SetMaxReplacements(int v)   { m_maxReplacements = v; }
    bool  GetDiagnostics() const      { return m_diagnostics; }
    void  SetDiagnostics(bool v)      { m_diagnostics = v; }
};

#endif // NOISE_STAGE_HOT_PIXEL_H_INCLUDED
