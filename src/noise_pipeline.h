/*
 *  noise_pipeline.h
 *  PHD Guiding
 *
 *  Frame-level noise reduction pipeline.
 *
 *  A NoisePipeline holds an ordered list of NoiseReductionStage objects.
 *  For each incoming frame, every enabled stage is given the chance to
 *  transform the frame (Apply) and then to update its own internal state
 *  from the (possibly already transformed) frame (Observe).
 *
 *  Stages are independent. New techniques (per-row bias, PSF-subtracted
 *  stacking, hot-pixel-only masking, etc.) can be added without touching
 *  the existing stages or the pipeline itself.
 *
 *  If plate solving has produced a PlateSolveResult for the current
 *  frame, it may be passed in so stages can exploit catalog star
 *  positions. All stages must also be usable without a solution.
 *
 *  Copyright (c) 2026 PHD2 Contributors
 *  All rights reserved. Distributed under the BSD licence (see LICENSE).
 */

#ifndef NOISE_PIPELINE_H_INCLUDED
#define NOISE_PIPELINE_H_INCLUDED

#include <memory>
#include <vector>

struct PlateSolveResult; // from astrometry_solver.h

class NoiseReductionStage
{
public:
    virtual ~NoiseReductionStage() = default;

    // Human readable identifier, also used as config key suffix.
    virtual wxString Name() const = 0;

    virtual bool IsEnabled() const { return m_enabled; }
    virtual void SetEnabled(bool enabled) { m_enabled = enabled; }

    // Transform the frame in place. Returns true if modified.
    virtual bool Apply(usImage& img, const PlateSolveResult *solve) = 0;

    // Observe a frame to update internal state. Called after Apply().
    virtual void Observe(const usImage& img, const PlateSolveResult *solve) {}

    virtual void Reset() {}
    virtual void LoadConfig() {}
    virtual void SaveConfig() {}

protected:
    bool m_enabled = false;
};

class NoisePipeline
{
    std::vector<std::unique_ptr<NoiseReductionStage>> m_stages;

public:
    NoisePipeline();
    ~NoisePipeline();

    // Takes ownership.
    void AddStage(std::unique_ptr<NoiseReductionStage> stage);

    // Process one frame. `solve` may be nullptr - stages fall back to
    // pure-image heuristics.
    void Process(usImage& img, const PlateSolveResult *solve = nullptr);

    size_t StageCount() const { return m_stages.size(); }
    NoiseReductionStage *GetStage(size_t i) const;
    NoiseReductionStage *FindStage(const wxString& name) const;

    void LoadConfig();
    void SaveConfig();
    void ResetAll();
};

// Global singleton, mirroring pConfig / pFrame / Debug pattern.
extern NoisePipeline *pNoise;

void InitNoisePipeline();
void ShutdownNoisePipeline();

#endif // NOISE_PIPELINE_H_INCLUDED
