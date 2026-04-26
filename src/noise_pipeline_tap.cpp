/*
 *  noise_pipeline_tap.cpp
 *  PHD Guiding
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#include "phd.h"
#include "noise_pipeline_tap.h"
#include "usImage.h"

#include <wx/thread.h>

namespace
{

// All access to the buffer goes through s_lock. s_activeRefs is
// duplicated into an atomic so IsActive() can avoid the lock on the
// hot path.
wxCriticalSection s_lock;
volatile int      s_activeRefs = 0;
unsigned          s_generation = 0;
usImage           s_raw;
usImage           s_denoised;
bool              s_haveData   = false;

} // namespace

void NoisePipelineTap::Activate()
{
    wxCriticalSectionLocker lck(s_lock);
    ++s_activeRefs;
}

void NoisePipelineTap::Deactivate()
{
    wxCriticalSectionLocker lck(s_lock);
    if (--s_activeRefs <= 0)
    {
        s_activeRefs = 0;
        // Release any held data so a subsequent Activate() doesn't
        // hand the next viewer a stale frame.
        s_haveData = false;
    }
}

bool NoisePipelineTap::IsActive()
{
    // Reading int is atomic on the platforms PHD2 supports; the worst
    // case for a torn read is one missed or one extra Submit, neither
    // of which corrupts the buffer.
    return s_activeRefs > 0;
}

void NoisePipelineTap::Submit(const usImage& raw, const usImage& denoised)
{
    if (!IsActive())
        return;
    wxCriticalSectionLocker lck(s_lock);
    if (s_activeRefs <= 0)
        return; // raced against Deactivate
    if (raw.ImageData == nullptr || denoised.ImageData == nullptr)
        return;
    // CopyFrom returns true on error (PHD2 convention).
    if (s_raw.CopyFrom(raw))
        return;
    if (s_denoised.CopyFrom(denoised))
        return;
    ++s_generation;
    s_haveData = true;
}

bool NoisePipelineTap::TryFetch(usImage& raw_out, usImage& denoised_out,
                                unsigned& lastSeen)
{
    wxCriticalSectionLocker lck(s_lock);
    if (!s_haveData || s_generation == lastSeen)
        return false;
    if (raw_out.CopyFrom(s_raw))
        return false;
    if (denoised_out.CopyFrom(s_denoised))
        return false;
    lastSeen = s_generation;
    return true;
}
