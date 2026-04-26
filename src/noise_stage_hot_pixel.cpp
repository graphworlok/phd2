/*
 *  noise_stage_hot_pixel.cpp
 *  PHD Guiding
 *
 *  Spatial 3x3 median-based hot-pixel / cosmic-ray rejection.
 *
 *  Algorithm: for each interior pixel, take the median of the 3x3
 *  neighbourhood (including the centre - cheaper to sort 9 elements
 *  than to special-case the centre, and the result is the same when the
 *  centre is not an outlier). If the centre exceeds
 *
 *      max(median * ratio, median + absOffset)
 *
 *  it is treated as an isolated bright impulse and replaced with the
 *  median. The combined ratio + absolute test means we don't replace
 *  pixels in dark regions where the multiplicative ratio is unstable
 *  (median may be 0 or 1 ADU) and we don't fire on bright nebulosity
 *  where stars genuinely sit on a pedestal of bright neighbours.
 *
 *  Border pixels (1-pixel ring) are passed through unchanged - cosmic
 *  rays at the edge are statistically rare and writing border-aware
 *  median code is not worth the complexity.
 *
 *  Frame-level safety: if the number of pixels we'd replace exceeds
 *  m_maxReplacements (a tunable cap), we abort the filter for that frame
 *  and leave the original data intact. Sky conditions like rapid cloud
 *  edges, lens dew, or a focuser drift can produce many bright spots
 *  per frame that look local but really are signal we want to keep.
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#include "phd.h"
#include "noise_stage_hot_pixel.h"
#include "camera.h"

#include <algorithm>
#include <array>

HotPixelStage::HotPixelStage()
    : m_ratio(3.0f),
      m_absOffset(200),
      m_maxReplacements(5000),
      m_diagnostics(false),
      m_diagFlagged(0),
      m_diagAborted(0)
{
    // Opt-in. The pipeline will only run this stage if the user enables
    // it, even though running it is cheap (a single pass with no state).
    m_enabled = false;
}

// Layered config (mirrors RollingBackgroundStage). Profile-scoped key is
// the default; per-camera key, when a hardware id is available, takes
// precedence. SaveConfig writes both so the profile copy remains a
// useful default when the camera id can't be resolved later.
namespace
{

wxString PerCameraPrefix()
{
    if (!pCamera)
        return wxString();
    const wxString tag = GuideCamera::HardwareIdToFileTag(pCamera->GetHardwareId());
    if (tag.empty())
        return wxString();
    return wxT("/NoisePipeline/Cameras/") + tag + wxT("/HotPixel/");
}

} // namespace

void HotPixelStage::LoadConfig()
{
    const wxString pp = wxT("/NoisePipeline/HotPixel/");
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

    m_enabled         = loadBool(wxT("Enabled"),         false);
    m_ratio           = (float) loadDbl(wxT("Ratio"),    m_ratio);
    m_absOffset       = loadInt(wxT("AbsOffset"),        m_absOffset);
    m_maxReplacements = loadInt(wxT("MaxReplacements"),  m_maxReplacements);
    m_diagnostics     = loadBool(wxT("Diagnostics"),     false);
}

void HotPixelStage::SaveConfig()
{
    const wxString pp = wxT("/NoisePipeline/HotPixel/");
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

    saveBool(wxT("Enabled"),         m_enabled);
    saveDbl (wxT("Ratio"),           m_ratio);
    saveInt (wxT("AbsOffset"),       m_absOffset);
    saveInt (wxT("MaxReplacements"), m_maxReplacements);
    saveBool(wxT("Diagnostics"),     m_diagnostics);
}

bool HotPixelStage::Apply(usImage& img, const PlateSolveResult * /*solve*/)
{
    m_diagFlagged = 0;
    m_diagAborted = 0;

    if (!m_enabled || !img.ImageData)
        return false;

    const int W = img.Size.GetWidth();
    const int H = img.Size.GetHeight();
    if (W < 3 || H < 3)
        return false;

    unsigned short *p = img.ImageData;
    const float ratio = (m_ratio > 1.0f) ? m_ratio : 1.0f;
    const int   absOff = (m_absOffset > 0) ? m_absOffset : 0;

    // Two-pass: pass one counts how many pixels would be replaced; if
    // the count exceeds the cap, we bail out without touching the frame.
    // This is the cheaper safety strategy than scribbling into a copy
    // of the buffer and discarding it on overage.
    auto wouldReplace = [&](int x, int y) -> bool {
        const unsigned short c = p[y * W + x];
        std::array<unsigned short, 9> n;
        int k = 0;
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
                n[k++] = p[(y + dy) * W + (x + dx)];
        std::nth_element(n.begin(), n.begin() + 4, n.end());
        const unsigned short med = n[4];
        // threshold = max(med*ratio, med+absOff), clamped to 16-bit.
        const float thrRatio = (float) med * ratio;
        const float thrAbs   = (float) med + (float) absOff;
        const float thr      = std::max(thrRatio, thrAbs);
        return (float) c > thr;
    };

    if (m_maxReplacements > 0)
    {
        size_t flagged = 0;
        for (int y = 1; y < H - 1 && flagged <= (size_t) m_maxReplacements; ++y)
            for (int x = 1; x < W - 1; ++x)
                if (wouldReplace(x, y))
                    if (++flagged > (size_t) m_maxReplacements)
                        break;

        if (flagged > (size_t) m_maxReplacements)
        {
            m_diagAborted = flagged;
            if (m_diagnostics)
            {
                Debug.Write(wxString::Format(
                    "HotPixel: aborted, would replace > %d pixels\n",
                    m_maxReplacements));
            }
            return false;
        }
    }

    // Pass two: do the actual replacement. We must read from a snapshot
    // so a flagged pixel doesn't poison the median for its neighbour to
    // the right. Two adjacent hot pixels are rare but possible; without
    // the snapshot the second one would see the corrected first one in
    // its own median window and might no longer be flagged correctly.
    std::vector<unsigned short> src(p, p + (size_t) W * (size_t) H);
    size_t replaced = 0;
    for (int y = 1; y < H - 1; ++y)
    {
        for (int x = 1; x < W - 1; ++x)
        {
            const unsigned short c = src[(size_t) y * W + x];
            std::array<unsigned short, 9> n;
            int k = 0;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx)
                    n[k++] = src[(size_t)(y + dy) * W + (x + dx)];
            std::nth_element(n.begin(), n.begin() + 4, n.end());
            const unsigned short med = n[4];
            const float thr = std::max((float) med * ratio,
                                       (float) med + (float) absOff);
            if ((float) c > thr)
            {
                p[(size_t) y * W + x] = med;
                ++replaced;
            }
        }
    }
    m_diagFlagged = replaced;

    if (m_diagnostics)
    {
        Debug.Write(wxString::Format(
            "HotPixel: replaced %zu pixels (ratio=%.2f, absOff=%d)\n",
            replaced, m_ratio, m_absOffset));
    }

    return replaced > 0;
}
