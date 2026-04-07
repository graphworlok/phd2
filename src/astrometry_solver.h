/*
 *  astrometry_solver.h
 *  PHD Guiding
 *
 *  Real-time plate solving via astrometry.net solve-field.
 *
 *  solve-field produces a .wcs (FITS) and a .corr (FITS binary table) file.
 *  Star catalog IDs come from the .corr INDEX_ID column (Tycho-2 / 2MASS).
 *
 *  The executable path and extra arguments are stored in pConfig->Global
 *  ("/PlateSolve/AstrometryPath" and "/PlateSolve/AstrometryExtra").
 *  Scale hint (arcsec/pixel) is passed directly to PlateSolveThread.
 *
 *  Copyright (c) 2024 PHD2 Contributors – BSD licence.
 */

#ifndef ASTROMETRY_SOLVER_H_INCLUDED
#define ASTROMETRY_SOLVER_H_INCLUDED

#include <vector>

// ---------------------------------------------------------------------------
// Data structures produced by the solver
// ---------------------------------------------------------------------------

struct SolvedStar
{
    double   pixel_x;    // 0-based image coordinates
    double   pixel_y;
    double   ra;         // degrees J2000
    double   dec;        // degrees J2000
    float    flux;
    wxString catalog_id; // Tycho-2 / 2MASS identifier, or "" if unavailable
};

struct WCSSolution
{
    bool   valid;
    double crpix1, crpix2;  // reference pixel (1-based FITS convention)
    double crval1, crval2;  // RA, Dec of reference pixel (degrees)
    double cd[2][2];        // CD matrix: degrees per pixel
    double pixScale;        // arcsec/pixel
    double rotation;        // degrees E of N
    int    imageW, imageH;

    // Project RA/Dec (degrees J2000) → 0-based pixel.
    bool RaDecToPixel(double ra, double dec, double *px, double *py) const;
};

struct PlateSolveResult
{
    bool                    success;
    wxString                errorMsg;
    WCSSolution             wcs;
    std::vector<SolvedStar> stars;
};

// ---------------------------------------------------------------------------
// Background solver thread
// ---------------------------------------------------------------------------

// Spawns solve-field as a subprocess, parses its output, and posts
// EVT_PLATE_SOLVE_COMPLETE to pFrame.  The payload is a heap-allocated
// PlateSolveResult* that pFrame's handler must delete.
class PlateSolveThread : public wxThread
{
    usImage *m_image;
    double   m_scaleLow;
    double   m_scaleHigh;

    bool WriteImageFits(const wxString& path);
    bool RunAstrometryNet(const wxString& fitsPath, const wxString& outDir,
                          wxString *wcsPath, wxString *corrPath);
    bool ParseWcs(const wxString& wcsPath, WCSSolution *wcs);
    bool ParseCorr(const wxString& corrPath, std::vector<SolvedStar> *stars);

public:
    PlateSolveThread(const usImage& image, double scaleLow, double scaleHigh);
    virtual ~PlateSolveThread();
    wxThread::ExitCode Entry() override;
};

wxDECLARE_EVENT(EVT_PLATE_SOLVE_COMPLETE, wxThreadEvent);

// ---------------------------------------------------------------------------
// Overlay drawing
// ---------------------------------------------------------------------------

void DrawPlateSolveOverlay(wxDC& dc,
                           const PlateSolveResult& result,
                           double scaleFactor,
                           int imageW, int imageH);

#endif // ASTROMETRY_SOLVER_H_INCLUDED
