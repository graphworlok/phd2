/*
 *  noise_pipeline_tap.h
 *  PHD Guiding
 *
 *  Thread-safe shared buffer that captures (raw, denoised) image pairs
 *  as they flow through the noise reduction pipeline. Used by the live
 *  noise viewer dialog to show before/after frames in real time.
 *
 *  The camera worker thread submits copies via Submit(); the GUI thread
 *  drains them via TryFetch(). When no viewer is active, IsActive()
 *  returns false and Submit() is a cheap no-op so the steady-state
 *  guiding loop pays nothing.
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#ifndef NOISE_PIPELINE_TAP_H_INCLUDED
#define NOISE_PIPELINE_TAP_H_INCLUDED

class usImage;

class NoisePipelineTap
{
public:
    // Reference-counted activation. Each open viewer should call
    // Activate() once on construction and Deactivate() once on close.
    static void Activate();
    static void Deactivate();

    // Fast-path check used by the camera worker before doing the
    // (relatively expensive) deep copy of the raw frame.
    static bool IsActive();

    // Worker-thread side: deposit copies of raw and denoised frames.
    // No-op when no viewer is active.
    static void Submit(const usImage& raw, const usImage& denoised);

    // GUI-thread side: copies the latest pair into `raw_out` /
    // `denoised_out` if the generation counter has advanced since
    // `lastSeen`. Updates `lastSeen` on success. Returns false when no
    // new frame is available.
    static bool TryFetch(usImage& raw_out, usImage& denoised_out,
                         unsigned& lastSeen);
};

#endif // NOISE_PIPELINE_TAP_H_INCLUDED
