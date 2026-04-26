/*
 *  noise_stage_dead_zone.cpp
 *  PHD Guiding
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#include "phd.h"
#include "noise_stage_dead_zone.h"
#include "camera.h"

#include <wx/filefn.h>

NoiseStageDeadZone::NoiseStageDeadZone()
    : m_loadAttempted(false),
      m_fillWithMedian(false),
      m_diagnostics(false)
{
    m_enabled = false; // opt-in
}

namespace
{

wxString PerCameraPrefix()
{
    if (!pCamera)
        return wxString();
    const wxString tag = GuideCamera::HardwareIdToFileTag(pCamera->GetHardwareId());
    if (tag.empty())
        return wxString();
    return wxT("/NoisePipeline/Cameras/") + tag + wxT("/DeadZone/");
}

} // namespace

wxString NoiseStageDeadZone::CurrentCameraTag()
{
    if (!pCamera)
        return wxString();
    wxString id = pCamera->GetHardwareId();
    if (id.empty())
        return wxString();
    const wxString mode = pCamera->GetCurrentModeTag();
    if (!mode.empty())
        id += wxT("@") + mode;
    return GuideCamera::HardwareIdToFileTag(id);
}

wxString NoiseStageDeadZone::CurrentMapPath()
{
    return DeadZoneMap::MapFilePath(CurrentCameraTag());
}

void NoiseStageDeadZone::LoadConfig()
{
    const wxString pp = wxT("/NoisePipeline/DeadZone/");
    const wxString cp = PerCameraPrefix();

    auto loadBool = [&](const wxString& key, bool def) -> bool {
        bool v = pConfig->Profile.GetBoolean(pp + key, def);
        if (!cp.empty())
            v = pConfig->Profile.GetBoolean(cp + key, v);
        return v;
    };

    m_enabled         = loadBool(wxT("Enabled"),         false);
    m_fillWithMedian  = loadBool(wxT("FillWithMedian"),  false);
    m_diagnostics     = loadBool(wxT("Diagnostics"),     false);
}

void NoiseStageDeadZone::SaveConfig()
{
    const wxString pp = wxT("/NoisePipeline/DeadZone/");
    const wxString cp = PerCameraPrefix();

    auto saveBool = [&](const wxString& key, bool v) {
        pConfig->Profile.SetBoolean(pp + key, v);
        if (!cp.empty())
            pConfig->Profile.SetBoolean(cp + key, v);
    };

    saveBool(wxT("Enabled"),         m_enabled);
    saveBool(wxT("FillWithMedian"),  m_fillWithMedian);
    saveBool(wxT("Diagnostics"),     m_diagnostics);
}

void NoiseStageDeadZone::Reset()
{
    // Reset is the user-visible "discard local state" hook. We don't
    // delete the on-disk map here - that's what ClearCurrent() is for.
    // Just force a re-load so any external rebuild is picked up.
    m_loadAttempted = false;
    m_loadedTag.clear();
    m_map.Clear();
}

void NoiseStageDeadZone::Invalidate()
{
    m_loadAttempted = false;
}

bool NoiseStageDeadZone::AdoptAndSave(DeadZoneMap map, bool saveToDisk)
{
    m_map = std::move(map);
    m_loadedTag = CurrentCameraTag();
    m_loadAttempted = true;
    if (saveToDisk && !m_loadedTag.empty())
        return m_map.SaveToDisk(m_loadedTag);
    return true;
}

bool NoiseStageDeadZone::ClearCurrent()
{
    const wxString tag = CurrentCameraTag();
    bool removed = false;
    if (!tag.empty())
    {
        const wxString path = DeadZoneMap::MapFilePath(tag);
        if (!path.empty() && wxFileExists(path))
            removed = wxRemoveFile(path);
    }
    m_map.Clear();
    m_loadedTag.clear();
    m_loadAttempted = true;
    return removed;
}

bool NoiseStageDeadZone::Apply(usImage& img, const PlateSolveResult * /*solve*/)
{
    if (!m_enabled || !img.ImageData)
        return false;

    const wxString tag = CurrentCameraTag();
    if (tag != m_loadedTag)
    {
        m_loadAttempted = false;
        m_loadedTag = tag;
        m_map.Clear();
    }

    if (!m_loadAttempted)
    {
        m_loadAttempted = true;
        if (!tag.empty())
        {
            if (!m_map.LoadFromDisk(tag))
            {
                if (m_diagnostics)
                {
                    Debug.Write(wxString::Format(
                        "DeadZone: no saved mask for tag '%s'\n", tag));
                }
            }
        }
    }

    if (m_map.IsEmpty())
        return false;

    const bool changed = m_map.Apply(img, m_fillWithMedian);
    if (m_diagnostics && changed)
    {
        const DeadZoneStats s = m_map.Stats();
        Debug.Write(wxString::Format(
            "DeadZone: masked %zu px in %zu regions (fill=%s)\n",
            s.maskedPixels, s.regionCount,
            m_fillWithMedian ? "median" : "zero"));
    }
    return changed;
}
