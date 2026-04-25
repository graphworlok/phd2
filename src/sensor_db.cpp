/*
 *  sensor_db.cpp
 *  PHD Guiding
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#include "phd.h"
#include "sensor_db.h"

#include <wx/stdpaths.h>
#include <wx/filename.h>
#include <wx/textfile.h>

#include <algorithm>

namespace
{

// Cached database, populated by Reload(). Ordered alphabetically by
// name once Reload() finishes.
std::vector<Sensor> s_db;
bool                s_loaded = false;

wxString FindSensorXml()
{
    wxString p = wxStandardPaths::Get().GetDataDir() + "/sensors.xml";
    if (wxFileExists(p))
        return p;
    p = wxFileName(wxStandardPaths::Get().GetExecutablePath()).GetPath() +
        "/sensors.xml";
    if (wxFileExists(p))
        return p;
    return wxEmptyString;
}

// Extract text between <tag> and </tag> on a single line. Same shape as
// the helper in cam_v4l2.cpp; duplicated here to keep this file
// platform-independent (the V4L2 backend isn't compiled on Windows).
bool XmlLineContent(const wxString& line, const wxString& tag, wxString *value)
{
    const wxString open  = "<"  + tag + ">";
    const wxString close = "</" + tag + ">";
    int s = line.Find(open);
    if (s == wxNOT_FOUND)
        return false;
    s += open.length();
    int e = line.Find(close);
    if (e == wxNOT_FOUND || e < s)
        return false;
    *value = line.Mid(s, e - s).Trim(true).Trim(false);
    return true;
}

} // namespace

bool SensorDatabase::Reload()
{
    s_db.clear();
    s_loaded = true;

    const wxString path = FindSensorXml();
    if (path.empty())
    {
        Debug.AddLine("SensorDB: sensors.xml not found");
        return false;
    }

    wxTextFile f;
    if (!f.Open(path))
    {
        Debug.AddLine(wxString::Format("SensorDB: cannot open %s", path));
        return false;
    }

    Sensor cur;
    bool inSensor = false;

    for (wxString line = f.GetFirstLine(); !f.Eof(); line = f.GetNextLine())
    {
        wxString t = line.Trim(true).Trim(false);

        if (t.Contains("<sensor>"))
        {
            cur = Sensor();
            inSensor = true;
            continue;
        }
        if (t.Contains("</sensor>"))
        {
            if (inSensor && !cur.name.empty())
                s_db.push_back(cur);
            inSensor = false;
            continue;
        }
        if (!inSensor)
            continue;

        wxString val;
        if      (XmlLineContent(t, "name",         &val)) cur.name         = val;
        else if (XmlLineContent(t, "manufacturer", &val)) cur.manufacturer = val;
        else if (XmlLineContent(t, "notes",        &val)) cur.notes        = val;
        else if (XmlLineContent(t, "pixel_size_um", &val))
        {
            double d = 0.0; val.ToDouble(&d);
            cur.pixelSizeUm = d;
        }
        else if (XmlLineContent(t, "pixel_array_w", &val))
        {
            long n = 0; val.ToLong(&n);
            cur.arrayWidth = (int) n;
        }
        else if (XmlLineContent(t, "pixel_array_h", &val))
        {
            long n = 0; val.ToLong(&n);
            cur.arrayHeight = (int) n;
        }
        else if (XmlLineContent(t, "bit_depth", &val))
        {
            long n = 0; val.ToLong(&n);
            cur.bitDepth = (int) n;
        }
        else if (XmlLineContent(t, "color_array", &val))
        {
            cur.colorArray = ParseColorArray(val);
        }
        else if (XmlLineContent(t, "full_well_e", &val))
        {
            double d = 0.0; val.ToDouble(&d);
            cur.fullWellE = d;
        }
        else if (XmlLineContent(t, "read_noise_e", &val))
        {
            double d = 0.0; val.ToDouble(&d);
            cur.readNoiseE = d;
        }
    }

    std::sort(s_db.begin(), s_db.end(),
              [](const Sensor& a, const Sensor& b) {
                  return a.name.CmpNoCase(b.name) < 0;
              });

    Debug.AddLine(wxString::Format("SensorDB: loaded %zu sensors from %s",
                                   s_db.size(), path));
    return true;
}

namespace
{
void EnsureLoaded()
{
    if (!s_loaded)
        SensorDatabase::Reload();
}
} // namespace

size_t SensorDatabase::Size()
{
    EnsureLoaded();
    return s_db.size();
}

const Sensor *SensorDatabase::Find(const wxString& name)
{
    if (name.empty())
        return nullptr;
    EnsureLoaded();
    for (const Sensor& s : s_db)
        if (s.name == name)
            return &s;
    return nullptr;
}

const Sensor *SensorDatabase::FindNoCase(const wxString& name)
{
    if (name.empty())
        return nullptr;
    EnsureLoaded();
    for (const Sensor& s : s_db)
        if (s.name.CmpNoCase(name) == 0)
            return &s;
    return nullptr;
}

wxArrayString SensorDatabase::GetAllNames()
{
    EnsureLoaded();
    wxArrayString names;
    names.reserve(s_db.size());
    for (const Sensor& s : s_db)
        names.Add(s.name);
    return names;
}

SensorColorArray SensorDatabase::ParseColorArray(const wxString& s)
{
    const wxString u = s.Upper().Trim(true).Trim(false);
    if (u == "MONO" || u == "MONOCHROME") return SensorColorArray::Mono;
    if (u == "RGGB") return SensorColorArray::RGGB;
    if (u == "BGGR") return SensorColorArray::BGGR;
    if (u == "GBRG") return SensorColorArray::GBRG;
    if (u == "GRBG") return SensorColorArray::GRBG;
    return SensorColorArray::Unknown;
}

wxString SensorDatabase::ColorArrayToString(SensorColorArray ca)
{
    switch (ca)
    {
    case SensorColorArray::Mono: return wxT("mono");
    case SensorColorArray::RGGB: return wxT("RGGB");
    case SensorColorArray::BGGR: return wxT("BGGR");
    case SensorColorArray::GBRG: return wxT("GBRG");
    case SensorColorArray::GRBG: return wxT("GRBG");
    case SensorColorArray::Unknown:
    default: return wxT("");
    }
}
