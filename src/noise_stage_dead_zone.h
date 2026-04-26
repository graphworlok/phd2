/*
 *  noise_stage_dead_zone.h
 *  PHD Guiding
 *
 *  Pipeline stage that applies a per-camera DeadZoneMap to each frame.
 *  Loads the mask lazily on first Apply (so a freshly-rebuilt map can
 *  be picked up without a profile reload) and rebinds whenever the
 *  current camera tag changes.
 *
 *  Place after RollingBackground in the pipeline so dead pixels go to
 *  zero (or to the post-subtraction frame median) rather than to a
 *  random raw-sensor value that the star detector might latch onto.
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#ifndef NOISE_STAGE_DEAD_ZONE_H_INCLUDED
#define NOISE_STAGE_DEAD_ZONE_H_INCLUDED

#include "noise_pipeline.h"
#include "dead_zone_map.h"

class NoiseStageDeadZone : public NoiseReductionStage
{
    DeadZoneMap m_map;
    wxString    m_loadedTag;       // tag the in-memory map was loaded for
    bool        m_loadAttempted;   // true if we've tried for m_loadedTag
    bool        m_fillWithMedian;  // false = zero, true = frame median
    bool        m_diagnostics;

public:
    NoiseStageDeadZone();

    wxString Name() const override { return wxT("DeadZone"); }

    bool Apply(usImage& img, const PlateSolveResult *solve) override;
    void LoadConfig() override;
    void SaveConfig() override;
    void Reset() override;

    // Force a reload of the mask on next Apply (e.g. after the user
    // builds a new mask).
    void Invalidate();

    // Adopt an externally-built map and (optionally) persist it to
    // disk under the current camera+mode tag.
    bool AdoptAndSave(DeadZoneMap map, bool saveToDisk);

    // Drop the saved file for the current camera+mode and clear the
    // in-memory map. Returns true if a file was deleted.
    bool ClearCurrent();

    DeadZoneStats CurrentStats() const { return m_map.Stats(); }

    bool  GetFillWithMedian() const   { return m_fillWithMedian; }
    void  SetFillWithMedian(bool v)   { m_fillWithMedian = v; }
    bool  GetDiagnostics() const      { return m_diagnostics; }
    void  SetDiagnostics(bool v)      { m_diagnostics = v; }

    // Used by the UI to render the file path of the currently active
    // mask (whether one is loaded or not).
    static wxString CurrentMapPath();
    static wxString CurrentCameraTag();
};

#endif // NOISE_STAGE_DEAD_ZONE_H_INCLUDED
