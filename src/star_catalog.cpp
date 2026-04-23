/*
 *  star_catalog.cpp
 *  PHD Guiding
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#include "phd.h"
#include "star_catalog.h"

#include <wx/textfile.h>
#include <wx/tokenzr.h>
#include <wx/stdpaths.h>
#include <wx/filename.h>

#include <cmath>
#include <limits>

StarCatalog *pStarCatalog = nullptr;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static double SphericalDistDeg(double ra1, double dec1, double ra2, double dec2)
{
    const double D2R = M_PI / 180.0;
    const double c = std::cos(dec1 * D2R) * std::cos(dec2 * D2R) *
                         std::cos((ra1 - ra2) * D2R)
                   + std::sin(dec1 * D2R) * std::sin(dec2 * D2R);
    if (c >= 1.0)  return 0.0;
    if (c <= -1.0) return 180.0;
    return std::acos(c) / D2R;
}

static wxString Trimmed(const wxString& s)
{
    wxString r = s;
    r.Trim(true).Trim(false);
    return r;
}

static bool LooksNumeric(const wxString& s)
{
    if (s.empty()) return true;
    double d;
    return s.ToCDouble(&d);
}

static wxString FindCatalogFile()
{
    // Same lookup strategy as uvc_cameras.xml: installed share dir first,
    // then alongside the executable (for dev/non-installed builds).
    wxString p = wxStandardPaths::Get().GetDataDir() + "/bright_stars.csv";
    if (wxFileExists(p))
        return p;

    wxFileName exe(wxStandardPaths::Get().GetExecutablePath());
    p = exe.GetPath() + "/bright_stars.csv";
    if (wxFileExists(p))
        return p;

    return wxEmptyString;
}

// ---------------------------------------------------------------------------
// StarCatalog
// ---------------------------------------------------------------------------

bool StarCatalog::LoadCSV(const wxString& path)
{
    m_entries.clear();
    m_loaded = false;

    wxTextFile file;
    if (!file.Open(path))
        return false;

    bool firstData = true;

    for (size_t i = 0; i < file.GetLineCount(); ++i)
    {
        wxString line = Trimmed(file.GetLine(i));
        if (line.empty() || line.StartsWith(wxT("#")))
            continue;

        wxStringTokenizer tk(line, wxT(","), wxTOKEN_RET_EMPTY_ALL);
        wxString tok[8];
        int n = 0;
        while (tk.HasMoreTokens() && n < 8)
            tok[n++] = Trimmed(tk.GetNextToken());
        if (n < 2)
            continue;

        // Auto-skip a header line if present.
        if (firstData)
        {
            firstData = false;
            if (!LooksNumeric(tok[0]) || !LooksNumeric(tok[1]))
                continue;
        }

        StarCatalogEntry e;
        e.vmag = std::numeric_limits<float>::quiet_NaN();

        if (!tok[0].ToCDouble(&e.ra_deg))  continue;
        if (!tok[1].ToCDouble(&e.dec_deg)) continue;
        if (n >= 3 && !tok[2].empty())
        {
            double v;
            if (tok[2].ToCDouble(&v)) e.vmag = (float) v;
        }
        if (n >= 4) e.hd       = tok[3];
        if (n >= 5) e.hip      = tok[4];
        if (n >= 6) e.name     = tok[5];
        if (n >= 7) e.bayer    = tok[6];
        if (n >= 8) e.spectrum = tok[7];

        m_entries.push_back(e);
    }

    m_loaded = !m_entries.empty();
    return m_loaded;
}

const StarCatalogEntry *StarCatalog::FindNearest(double ra, double dec,
                                                 double tol_arcsec) const
{
    const double tol_deg = tol_arcsec / 3600.0;
    const StarCatalogEntry *best = nullptr;
    double best_dist = tol_deg;

    for (const auto& e : m_entries)
    {
        // Cheap declination pre-filter.
        if (std::fabs(e.dec_deg - dec) > tol_deg)
            continue;
        const double d = SphericalDistDeg(ra, dec, e.ra_deg, e.dec_deg);
        if (d < best_dist)
        {
            best_dist = d;
            best = &e;
        }
    }
    return best;
}

wxString StarCatalog::FormatLabel(const StarCatalogEntry& e, bool withMag)
{
    wxString s;
    if      (!e.name.empty())  s = e.name;
    else if (!e.bayer.empty()) s = e.bayer;
    else if (!e.hd.empty())    s = wxT("HD ")  + e.hd;
    else if (!e.hip.empty())   s = wxT("HIP ") + e.hip;

    if (withMag && std::isfinite((double) e.vmag))
    {
        if (!s.empty()) s += wxT(" ");
        s += wxString::Format(wxT("(%.1fm)"), e.vmag);
    }
    return s;
}

// ---------------------------------------------------------------------------
// Init / shutdown
// ---------------------------------------------------------------------------

void InitStarCatalog()
{
    if (pStarCatalog)
        return;
    pStarCatalog = new StarCatalog();

    wxString p = FindCatalogFile();
    if (p.empty())
    {
        Debug.AddLine(wxT("StarCatalog: bright_stars.csv not found; "
                          "plate-solve overlay will show bare index IDs"));
        return;
    }

    if (pStarCatalog->LoadCSV(p))
        Debug.AddLine(wxString::Format(wxT("StarCatalog: loaded %zu entries from %s"),
                                       pStarCatalog->Size(), p));
    else
        Debug.AddLine(wxString::Format(wxT("StarCatalog: failed to parse %s"), p));
}

void ShutdownStarCatalog()
{
    delete pStarCatalog;
    pStarCatalog = nullptr;
}
