/*
 *  dead_zone_map.h
 *  PHD Guiding
 *
 *  Per-camera "dead zone" mask: connected regions of pixels that an
 *  extended dark-frame calibration has shown to be too noisy to be
 *  worth using.
 *
 *  Distinct from the existing DefectMap (image_math.h):
 *
 *    * DefectMap is a sparse list of *individual* hot/cold pixels
 *      detected by comparing a single master dark to its neighbours.
 *      It targets isolated defects.
 *
 *    * DeadZoneMap targets *regions*, identified by per-pixel temporal
 *      variance across many dark frames. Random-telegraph-signal (RTS)
 *      pixels, amp-glow corners, and edge-banding clusters are
 *      typically what gets flagged. Single-pixel hot defects fall to
 *      the BPM path; we ignore anything below `minRegionPx`.
 *
 *  The mask is bound to the camera's hardware id AND its current
 *  capture mode (pixel format), mirroring the rolling-background
 *  model: a dead-zone profile measured in YUYV is not portable to
 *  MJPEG, since MJPEG's JPEG quantisation will smear single-pixel
 *  noise into 8x8 blocks.
 *
 *  Storage: a single uint8 image (0 = ok, 1 = dead) saved as a
 *  multi-extension FITS file in the dark library directory.
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#ifndef DEAD_ZONE_MAP_H_INCLUDED
#define DEAD_ZONE_MAP_H_INCLUDED

#include <cstddef>
#include <cstdint>
#include <vector>

class usImage;

// Compact summary returned to the UI.
struct DeadZoneStats
{
    size_t maskedPixels = 0;
    size_t regionCount  = 0;
    int    width        = 0;
    int    height       = 0;
};

class DeadZoneMap
{
public:
    DeadZoneMap();

    // Replaces the current mask. Geometry is taken from `mask.size()`
    // and the supplied width.
    void Set(int width, int height, std::vector<uint8_t> mask, size_t regionCount);

    void Clear();
    bool IsEmpty() const { return m_mask.empty(); }
    int Width() const { return m_width; }
    int Height() const { return m_height; }

    // True if (x, y) is inside a dead zone. Returns false for
    // out-of-range coordinates so callers don't need to bounds-check.
    bool IsDead(int x, int y) const;

    DeadZoneStats Stats() const;

    // Apply the mask to img in-place. fillWithMedian=true substitutes
    // the per-frame median for masked pixels (preserves visual
    // brightness without disturbing star detection); =false zeroes
    // them, which is what a star detector wants.
    bool Apply(usImage& img, bool fillWithMedian) const;

    // Persistence. The tag should be the result of
    // GuideCamera::HardwareIdToFileTag, optionally extended with mode.
    // Returns true on success. Silent on missing files (no error).
    bool SaveToDisk(const wxString& camTag) const;
    bool LoadFromDisk(const wxString& camTag);

    // Where the FITS file lives for a given tag. Empty on empty tag.
    static wxString MapFilePath(const wxString& camTag);

    // Read-only access for the pipeline stage.
    const std::vector<uint8_t>& MaskRef() const { return m_mask; }

private:
    int m_width = 0;
    int m_height = 0;
    size_t m_regionCount = 0;
    std::vector<uint8_t> m_mask;
};

// Streaming builder. Frames are pushed in via AddFrame; Finalize
// produces a mask. Designed so the caller can drive it from inside an
// existing capture loop (e.g. the darks dialog) without buffering all
// frames in memory.
class DeadZoneBuilder
{
public:
    struct Params
    {
        // A pixel is flagged when its temporal stdev across the
        // captured frames exceeds (median_stdev * sigmaMultiplier)
        // AND its mean exceeds the per-frame median plus
        // absoluteOffset. The combined test rejects pixels that are
        // either too noisy *or* too bright on average.
        float sigmaMultiplier = 4.0f;
        int   absoluteOffset  = 200;       // ADU, 16-bit scale

        // Connected-region size constraints. Anything smaller is
        // treated as a single-pixel defect (BPM territory) and
        // dropped. Anything bigger than a fraction of the frame is
        // assumed to be a calibration accident.
        int   minRegionPx     = 4;
        float maxRegionFrac   = 0.20f;     // 20% of the frame area

        // Whether to grow each surviving region by one pixel before
        // emitting the mask. A 1-pixel halo catches slightly-noisy
        // neighbours of the worst pixels and gives the star detector
        // a clean buffer.
        bool  dilateOnePixel  = true;
    };

    DeadZoneBuilder();

    // Begin a new build for a frame of (w, h) pixels. Resets state.
    void Begin(int w, int h);

    // Push one frame. Geometry must match Begin's. Returns false if
    // the geometry doesn't match (frame is then ignored).
    bool AddFrame(const usImage& frame);

    // Number of frames accumulated so far.
    int FrameCount() const { return m_frameCount; }

    // Apply Params and produce the mask. Empty frame count produces
    // an empty mask. Output is *moved* into `out`.
    bool Finalize(const Params& params, DeadZoneMap *out) const;

private:
    int m_width = 0;
    int m_height = 0;
    int m_frameCount = 0;

    // Welford accumulators (per pixel). m_mean holds the running
    // mean, m_M2 the sum of squared deltas; stdev = sqrt(M2 / (n-1)).
    std::vector<double> m_mean;
    std::vector<double> m_M2;
};

#endif // DEAD_ZONE_MAP_H_INCLUDED
