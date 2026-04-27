/*
 *  sim_skycatalog.cpp
 *  PHD Guiding
 *
 *  Linux-only: depends on the astrometry.net `query-starkd` tool
 *  and on cfitsio for parsing its output.
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#include "phd.h"
#include "sim_skycatalog.h"

#if defined(__linux__) && !defined(__APPLE__)

#include <fitsio.h>
#include <wx/dir.h>
#include <wx/filename.h>
#include <wx/utils.h>

#include <cmath>
#include <limits>

namespace {

// Approximate field-size lower bound (arcmin) for each astrometry.net
// scale band 00..19. Lower NN = finer / deeper, higher NN = wider.
// Source: astrometry.net "Which Index Files Do I Need" page.
static const double kScaleArcmin[20] = {
    2.0,    2.8,    4.0,    5.6,    8.0,
    11.0,   16.0,   22.0,   30.0,   42.0,
    60.0,   85.0,   120.0,  170.0,  240.0,
    340.0,  480.0,  680.0,  1000.0, 1400.0
};

// Pull the scale band number out of an astrometry index filename like
// "index-2mass-04.fits" or "index-2mass-04-37.fits".
// Returns -1 if the name doesn't follow the convention.
static int ScaleBandFromName(const wxString& path)
{
    wxString name = wxFileName(path).GetName();    // strip extension
    long nn;

    // If the name ends in -MM (HEALPix tile), strip it first.
    long mm;
    wxString tail = name.AfterLast('-');
    wxString head = name.BeforeLast('-');
    if (tail.ToLong(&mm) && head.AfterLast('-').ToLong(&nn))
    {
        if (nn >= 0 && nn < 20) return (int) nn;
    }
    // No tile suffix: NN is the trailing number.
    if (name.AfterLast('-').ToLong(&nn))
    {
        if (nn >= 0 && nn < 20) return (int) nn;
    }
    return -1;
}

// Walk dir for index-*.fits and pick the densest band whose
// nominal field size is large enough to span the query radius
// (so quad sizes overlap the region) but not so wide that we
// pull a sparse wide-field index when a deeper one is available.
static wxString PickIndexInDir(const wxString& dir, double radius_deg)
{
    wxArrayString files;
    wxDir::GetAllFiles(dir, &files, wxT("index-*.fits"), wxDIR_FILES);
    if (files.IsEmpty())
        return wxEmptyString;

    const double radius_arcmin = radius_deg * 60.0;
    int      best_nn = -1;
    wxString best;

    for (size_t i = 0; i < files.size(); ++i)
    {
        int nn = ScaleBandFromName(files[i]);
        if (nn < 0)
            continue;
        const double band = kScaleArcmin[nn];
        // Accept files whose field band brackets the query radius
        // within roughly half-octave to three-octaves.
        if (band >= radius_arcmin * 0.5 && band <= radius_arcmin * 8.0)
        {
            // Largest-NN-that-fits gives us the deepest applicable
            // catalog at the right scale.
            if (nn > best_nn)
            {
                best_nn = nn;
                best = files[i];
            }
        }
    }
    if (best.IsEmpty())
        best = files[0];        // last-ditch fallback
    return best;
}

static bool RunQueryStarkd(const wxString& indexFile,
                           double ra, double dec, double radius_deg,
                           const wxString& outFits)
{
    wxString cmd = wxString::Format(
        wxT("query-starkd \"%s\" -r %.6f -d %.6f -R %.6f -o \"%s\""),
        indexFile, ra, dec, radius_deg, outFits);
    Debug.AddLine(wxT("SimSkyCatalog: ") + cmd);
    long rc = wxExecute(cmd, wxEXEC_SYNC | wxEXEC_HIDE_CONSOLE);
    return rc == 0;
}

// Try the column names that astrometry.net's various star kdtrees use
// for the brightness tagalong column.
static int FindMagColumn(fitsfile *fp)
{
    static const char *cands[] = {
        "MAG",  "JMAG", "Jmag", "VMAG", "Vmag",
        "HMAG", "Hmag", "KMAG", "Kmag",
        "mag_J","mag_V",
        nullptr
    };
    for (const char **c = cands; *c; ++c)
    {
        int col = 0;
        int s = 0;
        fits_get_colnum(fp, CASEINSEN, (char *) *c, &col, &s);
        if (s == 0 && col > 0)
            return col;
    }
    return -1;
}

struct SkyEntry
{
    double ra, dec;
    float  mag;
};

static bool ReadStarsFits(const wxString& path,
                          double magLimit,
                          std::vector<SkyEntry>& out)
{
    fitsfile *fp = nullptr;
    int status = 0;

    if (fits_open_table(&fp, (const char *) path.fn_str(), READONLY, &status))
    {
        Debug.AddLine(wxString::Format(
            wxT("SimSkyCatalog: cannot open %s (cfitsio status=%d)"),
            path, status));
        return false;
    }

    long nrows = 0;
    fits_get_num_rows(fp, &nrows, &status);

    int col_ra  = 0;
    int col_dec = 0;
    fits_get_colnum(fp, CASEINSEN, (char *) "RA",  &col_ra,  &status);
    fits_get_colnum(fp, CASEINSEN, (char *) "DEC", &col_dec, &status);
    int col_mag = (status == 0) ? FindMagColumn(fp) : -1;

    if (status != 0 || col_ra <= 0 || col_dec <= 0)
    {
        Debug.AddLine(wxT("SimSkyCatalog: query output missing RA/DEC columns"));
        fits_close_file(fp, &status);
        return false;
    }

    out.reserve(out.size() + (size_t) nrows);
    for (long r = 1; r <= nrows; ++r)
    {
        double ra = 0, dec = 0;
        float  mag = std::numeric_limits<float>::quiet_NaN();
        int    s = 0;
        fits_read_col(fp, TDOUBLE, col_ra,  r, 1, 1, nullptr, &ra,  nullptr, &s);
        fits_read_col(fp, TDOUBLE, col_dec, r, 1, 1, nullptr, &dec, nullptr, &s);
        if (col_mag > 0)
            fits_read_col(fp, TFLOAT, col_mag, r, 1, 1, nullptr, &mag, nullptr, &s);
        if (s != 0)
            continue;
        if (std::isfinite(mag) && mag > magLimit)
            continue;

        SkyEntry e;
        e.ra  = ra;
        e.dec = dec;
        e.mag = mag;
        out.push_back(e);
    }

    fits_close_file(fp, &status);
    return !out.empty();
}

// Forward TAN (gnomonic) projection of (RA, Dec) -> 0-based (px, py).
// Same canonical convention as solve-field's WCS output: north up,
// east left, with rotation applied via a CD matrix in degrees/pixel.
//
// Returns false if the point is on the far hemisphere from the
// reference direction.
static bool ProjectRaDec(double ra_deg, double dec_deg,
                         double crval1_deg, double crval2_deg,
                         double crpix1_zero, double crpix2_zero,
                         double cd[2][2],
                         double *px, double *py)
{
    const double D2R = M_PI / 180.0;
    const double ra0  = crval1_deg * D2R;
    const double dec0 = crval2_deg * D2R;
    const double ra1  = ra_deg     * D2R;
    const double dec1 = dec_deg    * D2R;

    const double cos_d1 = cos(dec1), sin_d1 = sin(dec1);
    const double cos_d0 = cos(dec0), sin_d0 = sin(dec0);
    const double cos_dr = cos(ra1 - ra0);

    const double denom = sin_d1 * sin_d0 + cos_d1 * cos_d0 * cos_dr;
    if (denom <= 0.0)
        return false;

    const double xi  = (cos_d1 * sin(ra1 - ra0)) / denom * (180.0 / M_PI);
    const double eta = ((sin_d1 * cos_d0 - cos_d1 * sin_d0 * cos_dr) / denom)
                       * (180.0 / M_PI);

    const double det = cd[0][0] * cd[1][1] - cd[0][1] * cd[1][0];
    if (fabs(det) < 1e-30)
        return false;

    const double ci00 =  cd[1][1] / det;
    const double ci01 = -cd[0][1] / det;
    const double ci10 = -cd[1][0] / det;
    const double ci11 =  cd[0][0] / det;

    *px = crpix1_zero + ci00 * xi + ci01 * eta;
    *py = crpix2_zero + ci10 * xi + ci11 * eta;
    return true;
}

} // anon

bool BuildSimSkyField(const wxString& indexPath,
                      double ra_deg, double dec_deg,
                      int frameW, int frameH,
                      double scale_asec,
                      double cam_angle_deg,
                      double magLimit,
                      int border_pix,
                      std::vector<SimSkyPixelStar>& outStars)
{
    outStars.clear();
    if (frameW <= 0 || frameH <= 0 || scale_asec <= 0.0)
        return false;

    // Query radius = half-diagonal of the frame, with 10% slack for
    // rotation and edge-case TAN distortion.
    const double half_diag_pix = 0.5 * std::sqrt((double) frameW * frameW
                                                 + (double) frameH * frameH);
    const double radius_deg    = (half_diag_pix * scale_asec / 3600.0) * 1.1;

    // Resolve the index file (single-file or pick from a directory).
    wxString file = indexPath;
    if (wxDirExists(indexPath))
    {
        file = PickIndexInDir(indexPath, radius_deg);
        if (file.IsEmpty())
        {
            Debug.AddLine(wxT("SimSkyCatalog: no index files in ") + indexPath);
            return false;
        }
    }
    if (!wxFileExists(file))
    {
        Debug.AddLine(wxT("SimSkyCatalog: index file not found: ") + file);
        return false;
    }

    // Run query-starkd, parse its FITS output.
    wxString tmp = wxFileName::CreateTempFileName(wxT("phd2_skycat_"));
    if (wxFileExists(tmp))
        wxRemoveFile(tmp);              // CreateTempFileName leaves a 0-byte file
    tmp += wxT(".fits");

    if (!RunQueryStarkd(file, ra_deg, dec_deg, radius_deg, tmp))
    {
        Debug.AddLine(wxT("SimSkyCatalog: query-starkd subprocess failed"));
        if (wxFileExists(tmp)) wxRemoveFile(tmp);
        return false;
    }

    std::vector<SkyEntry> raw;
    bool readOk = ReadStarsFits(tmp, magLimit, raw);
    wxRemoveFile(tmp);
    if (!readOk)
        return false;

    // Build the forward WCS used for projection. North-up east-left
    // with rotation cam_angle East of North, identical convention to
    // solve-field's emitted WCS so a sim->solve roundtrip recovers
    // the same pointing/orientation.
    const double s   = scale_asec / 3600.0;     // degrees per pixel
    const double th  = cam_angle_deg * M_PI / 180.0;
    const double cd[2][2] = {
        { -s * cos(th),  s * sin(th) },
        {  s * sin(th),  s * cos(th) }
    };
    const double crpix1_zero = frameW / 2.0;    // 0-based reference pixel
    const double crpix2_zero = frameH / 2.0;

    outStars.reserve(raw.size());
    for (size_t i = 0; i < raw.size(); ++i)
    {
        double px = 0, py = 0;
        if (!ProjectRaDec(raw[i].ra, raw[i].dec,
                          ra_deg, dec_deg,
                          crpix1_zero, crpix2_zero,
                          (double (*)[2]) cd,
                          &px, &py))
            continue;
        if (px < border_pix || px >= frameW - border_pix) continue;
        if (py < border_pix || py >= frameH - border_pix) continue;

        SimSkyPixelStar s;
        s.pixel_x = px;
        s.pixel_y = py;
        s.vmag    = raw[i].mag;
        s.ra_deg  = raw[i].ra;
        s.dec_deg = raw[i].dec;
        outStars.push_back(s);
    }

    Debug.AddLine(wxString::Format(
        wxT("SimSkyCatalog: %zu/%zu stars in frame "
            "(R=%.3f deg, magLim=%.1f, idx=%s)"),
        outStars.size(), raw.size(), radius_deg, magLimit, file));

    return !outStars.empty();
}

#else  // non-Linux: stub

bool BuildSimSkyField(const wxString& /*indexPath*/,
                      double /*ra_deg*/, double /*dec_deg*/,
                      int /*frameW*/, int /*frameH*/,
                      double /*scale_asec*/,
                      double /*cam_angle_deg*/,
                      double /*magLimit*/,
                      int /*border_pix*/,
                      std::vector<SimSkyPixelStar>& outStars)
{
    outStars.clear();
    return false;
}

#endif // __linux__ && !__APPLE__
