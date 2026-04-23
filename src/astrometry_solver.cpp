/*
 *  astrometry_solver.cpp
 *  PHD Guiding
 *
 *  Plate solving via astrometry.net solve-field.
 *
 *  Copyright (c) 2024 PHD2 Contributors – BSD licence.
 */

#include "phd.h"

#ifdef __linux__

# include "astrometry_solver.h"
# include "fitsiowrap.h"
# include "fitsio.h"
# include "star_catalog.h"

# include <wx/filename.h>
# include <wx/stdpaths.h>
# include <wx/utils.h>
# include <wx/dir.h>
# include <wx/file.h>

# include <cmath>

// ---------------------------------------------------------------------------
// Custom event
// ---------------------------------------------------------------------------

wxDEFINE_EVENT(EVT_PLATE_SOLVE_COMPLETE, wxThreadEvent);

// ---------------------------------------------------------------------------
// WCSSolution::RaDecToPixel  –  TAN (gnomonic) projection
// ---------------------------------------------------------------------------

bool WCSSolution::RaDecToPixel(double ra, double dec,
                                double *px, double *py) const
{
    const double D2R = M_PI / 180.0;

    double ra0  = crval1 * D2R, dec0 = crval2 * D2R;
    double ra1  = ra    * D2R, dec1 = dec    * D2R;

    double cos_d1 = cos(dec1), sin_d1 = sin(dec1);
    double cos_d0 = cos(dec0), sin_d0 = sin(dec0);
    double cos_dr = cos(ra1 - ra0);

    double denom = sin_d1 * sin_d0 + cos_d1 * cos_d0 * cos_dr;
    if (denom <= 0.0)
        return false;

    double xi  =  (cos_d1 * sin(ra1 - ra0)) / denom * (180.0 / M_PI);
    double eta = ((sin_d1 * cos_d0 - cos_d1 * sin_d0 * cos_dr) / denom)
                 * (180.0 / M_PI);

    double det = cd[0][0] * cd[1][1] - cd[0][1] * cd[1][0];
    if (fabs(det) < 1e-30)
        return false;

    double ci[2][2] = {
        {  cd[1][1] / det, -cd[0][1] / det },
        { -cd[1][0] / det,  cd[0][0] / det }
    };

    *px = (crpix1 - 1.0) + ci[0][0] * xi + ci[0][1] * eta;
    *py = (crpix2 - 1.0) + ci[1][0] * xi + ci[1][1] * eta;
    return true;
}

// ---------------------------------------------------------------------------
// PlateSolveThread
// ---------------------------------------------------------------------------

PlateSolveThread::PlateSolveThread(const usImage& image,
                                   double scaleLow, double scaleHigh)
    : wxThread(wxTHREAD_DETACHED),
      m_image(new usImage()),
      m_scaleLow(scaleLow),
      m_scaleHigh(scaleHigh)
{
    m_image->Init(image.Size);
    memcpy(m_image->ImageData, image.ImageData,
           image.NPixels * sizeof(unsigned short));
}

PlateSolveThread::~PlateSolveThread()
{
    delete m_image;
}

wxThread::ExitCode PlateSolveThread::Entry()
{
    PlateSolveResult *result = new PlateSolveResult();
    result->success   = false;
    result->wcs.valid = false;

    wxString outDir = wxFileName::GetTempDir() + "/phd2_solve";
    if (!wxDirExists(outDir))
        wxMkdir(outDir);

    wxString fitsPath = outDir + "/frame.fits";

    if (!WriteImageFits(fitsPath))
    {
        result->errorMsg = "Failed to write temporary FITS file";
        goto done;
    }

    {
        wxString wcsPath, corrPath;
        if (!RunAstrometryNet(fitsPath, outDir, &wcsPath, &corrPath))
        {
            result->errorMsg = "solve-field failed or timed out";
            goto done;
        }
        if (!wxFileExists(wcsPath))
        {
            result->errorMsg = "No WCS solution produced (field not solved)";
            goto done;
        }
        if (!ParseWcs(wcsPath, &result->wcs))
        {
            result->errorMsg = "Failed to parse WCS solution";
            goto done;
        }
        if (wxFileExists(corrPath))
            ParseCorr(corrPath, &result->stars);

        // Cross-match each matched star against the local catalog so the
        // overlay can show real names / HD / HIP / magnitude rather than
        // the bare solve-field INDEX_ID.
        if (pStarCatalog && pStarCatalog->IsLoaded())
        {
            for (SolvedStar& s : result->stars)
            {
                const StarCatalogEntry *m = pStarCatalog->FindNearest(s.ra, s.dec, 15.0);
                if (!m) continue;
                s.name     = m->name;
                s.bayer    = m->bayer;
                s.hd       = m->hd;
                s.hip      = m->hip;
                s.spectrum = m->spectrum;
                s.vmag     = m->vmag;
            }
        }

        result->success = true;
    }

done:
    wxThreadEvent *evt = new wxThreadEvent(EVT_PLATE_SOLVE_COMPLETE);
    evt->SetPayload<PlateSolveResult *>(result);
    wxQueueEvent(pFrame, evt);
    return (wxThread::ExitCode) 0;
}

// ---------------------------------------------------------------------------
// Write the guide frame to a temporary FITS file
// ---------------------------------------------------------------------------

bool PlateSolveThread::WriteImageFits(const wxString& path)
{
    fitsfile *fptr = nullptr;
    int status = 0;

    PHD_fits_create_file(&fptr, path, true, &status);
    if (status)
        return false;

    long dims[2] = { m_image->Size.GetWidth(), m_image->Size.GetHeight() };
    fits_create_img(fptr, USHORT_IMG, 2, dims, &status);
    fits_write_img(fptr, TUSHORT, 1, (long) m_image->NPixels,
                   m_image->ImageData, &status);
    PHD_fits_close_file(fptr);
    return status == 0;
}

// ---------------------------------------------------------------------------
// Run astrometry.net solve-field
// ---------------------------------------------------------------------------

bool PlateSolveThread::RunAstrometryNet(const wxString& fitsPath,
                                         const wxString& outDir,
                                         wxString *wcsPath,
                                         wxString *corrPath)
{
    wxString exe = pConfig->Global.GetString("/PlateSolve/AstrometryPath", "solve-field");
    if (exe.empty())
        exe = "solve-field";

    *wcsPath  = outDir + "/frame.wcs";
    *corrPath = outDir + "/frame.corr";

    wxString cmd = exe;
    cmd += " --no-plots --overwrite --crpix-center";
    cmd += " --corr "    + *corrPath;
    cmd += " --wcs "     + *wcsPath;
    cmd += " --new-fits none --rdls none --match none";
    cmd += " --solved none --index-xyls none -O";

    if (m_scaleLow > 0.0 && m_scaleHigh > 0.0)
        cmd += wxString::Format(" --scale-low %.4f --scale-high %.4f"
                                " --scale-units arcsecperpix",
                                m_scaleLow, m_scaleHigh);

    wxString extra = pConfig->Global.GetString("/PlateSolve/AstrometryExtra", "");
    if (!extra.empty())
        cmd += " " + extra;

    cmd += " " + fitsPath;

    Debug.AddLine(wxString::Format("PlateSolve: %s", cmd));
    long ret = wxExecute(cmd, wxEXEC_SYNC | wxEXEC_HIDE_CONSOLE);
    return ret == 0;
}

// ---------------------------------------------------------------------------
// Parse astrometry.net .wcs FITS file
// ---------------------------------------------------------------------------

bool PlateSolveThread::ParseWcs(const wxString& wcsPath, WCSSolution *wcs)
{
    fitsfile *fptr = nullptr;
    int status = 0;

    PHD_fits_open_diskfile(&fptr, wcsPath, READONLY, &status);
    if (status) return false;

    double crpix1 = 0, crpix2 = 0, crval1 = 0, crval2 = 0;
    double cd1_1  = 0, cd1_2  = 0, cd2_1  = 0, cd2_2  = 0;
    int    naxis1 = 0, naxis2 = 0;
    char   dummy[FLEN_VALUE];

    fits_read_key(fptr, TDOUBLE, "CRPIX1", &crpix1, dummy, &status);
    fits_read_key(fptr, TDOUBLE, "CRPIX2", &crpix2, dummy, &status);
    fits_read_key(fptr, TDOUBLE, "CRVAL1", &crval1, dummy, &status);
    fits_read_key(fptr, TDOUBLE, "CRVAL2", &crval2, dummy, &status);
    fits_read_key(fptr, TDOUBLE, "CD1_1",  &cd1_1,  dummy, &status);
    fits_read_key(fptr, TDOUBLE, "CD1_2",  &cd1_2,  dummy, &status);
    fits_read_key(fptr, TDOUBLE, "CD2_1",  &cd2_1,  dummy, &status);
    fits_read_key(fptr, TDOUBLE, "CD2_2",  &cd2_2,  dummy, &status);
    fits_read_key(fptr, TINT, "IMAGEW", &naxis1, dummy, &status);
    if (status) { status = 0; naxis1 = m_image->Size.GetWidth(); }
    fits_read_key(fptr, TINT, "IMAGEH", &naxis2, dummy, &status);
    if (status) { status = 0; naxis2 = m_image->Size.GetHeight(); }

    PHD_fits_close_file(fptr);
    if (status && status != END_OF_FILE) return false;

    wcs->crpix1  = crpix1; wcs->crpix2  = crpix2;
    wcs->crval1  = crval1; wcs->crval2  = crval2;
    wcs->cd[0][0] = cd1_1; wcs->cd[0][1] = cd1_2;
    wcs->cd[1][0] = cd2_1; wcs->cd[1][1] = cd2_2;
    wcs->imageW  = naxis1; wcs->imageH  = naxis2;
    wcs->pixScale = sqrt(fabs(cd1_1 * cd2_2 - cd1_2 * cd2_1)) * 3600.0;
    wcs->rotation = atan2(cd2_1, cd2_2) * (180.0 / M_PI);
    wcs->valid    = true;
    return true;
}

// ---------------------------------------------------------------------------
// Parse .corr FITS binary table
// ---------------------------------------------------------------------------

bool PlateSolveThread::ParseCorr(const wxString& corrPath,
                                  std::vector<SolvedStar> *stars)
{
    fitsfile *fptr = nullptr;
    int status = 0;

    PHD_fits_open_diskfile(&fptr, corrPath, READONLY, &status);
    if (status) return false;

    int hdutype = 0;
    fits_movabs_hdu(fptr, 2, &hdutype, &status);
    if (status || hdutype != BINARY_TBL) { PHD_fits_close_file(fptr); return false; }

    long nrows = 0;
    fits_get_num_rows(fptr, &nrows, &status);
    if (status || nrows <= 0) { PHD_fits_close_file(fptr); return false; }

    int col_fx = 0, col_fy = 0, col_ira = 0, col_idec = 0;
    int col_flux = 0, col_id = 0;

    fits_get_colnum(fptr, CASEINSEN, const_cast<char *>("FIELD_X"),   &col_fx,   &status); status = 0;
    fits_get_colnum(fptr, CASEINSEN, const_cast<char *>("FIELD_Y"),   &col_fy,   &status); status = 0;
    fits_get_colnum(fptr, CASEINSEN, const_cast<char *>("INDEX_RA"),  &col_ira,  &status); status = 0;
    fits_get_colnum(fptr, CASEINSEN, const_cast<char *>("INDEX_DEC"), &col_idec, &status); status = 0;
    fits_get_colnum(fptr, CASEINSEN, const_cast<char *>("FLUX"),      &col_flux, &status); status = 0;
    fits_get_colnum(fptr, CASEINSEN, const_cast<char *>("INDEX_ID"),  &col_id,   &status); status = 0;

    stars->reserve((size_t) nrows);
    for (long row = 1; row <= nrows; row++)
    {
        SolvedStar s;
        double fx = 0, fy = 0, ira = 0, idec = 0;
        float  flux = 0;
        long long index_id = 0;
        int anynul = 0;

        if (col_fx)   fits_read_col(fptr, TDOUBLE,   col_fx,   row, 1, 1, nullptr, &fx,       &anynul, &status);
        if (col_fy)   fits_read_col(fptr, TDOUBLE,   col_fy,   row, 1, 1, nullptr, &fy,       &anynul, &status);
        if (col_ira)  fits_read_col(fptr, TDOUBLE,   col_ira,  row, 1, 1, nullptr, &ira,      &anynul, &status);
        if (col_idec) fits_read_col(fptr, TDOUBLE,   col_idec, row, 1, 1, nullptr, &idec,     &anynul, &status);
        if (col_flux) fits_read_col(fptr, TFLOAT,    col_flux, row, 1, 1, nullptr, &flux,     &anynul, &status);
        if (col_id)   fits_read_col(fptr, TLONGLONG, col_id,   row, 1, 1, nullptr, &index_id, &anynul, &status);
        status = 0;

        s.pixel_x    = fx - 1.0; // 1-based → 0-based
        s.pixel_y    = fy - 1.0;
        s.ra         = ira;
        s.dec        = idec;
        s.flux       = flux;
        s.catalog_id = wxString::Format("%lld", index_id);
        stars->push_back(s);
    }

    PHD_fits_close_file(fptr);
    return !stars->empty();
}

// ---------------------------------------------------------------------------
// DrawPlateSolveOverlay
// ---------------------------------------------------------------------------

void DrawPlateSolveOverlay(wxDC& dc,
                           const PlateSolveResult& result,
                           double scaleFactor,
                           int imageW, int imageH)
{
    if (!result.success || !result.wcs.valid)
        return;

    const WCSSolution& wcs = result.wcs;

    // --- Star circles ---
    dc.SetBrush(*wxTRANSPARENT_BRUSH);
    wxFont labelFont(7, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    dc.SetFont(labelFont);

    for (const SolvedStar& star : result.stars)
    {
        int sx = (int)(star.pixel_x * scaleFactor);
        int sy = (int)(star.pixel_y * scaleFactor);
        if (sx < 0 || sy < 0 || sx >= imageW || sy >= imageH)
            continue;

        float normFlux = (star.flux > 0.0f) ? star.flux : 1.0f;
        int radius = wxMax(4, wxMin(14, (int)(3.0 + log10(normFlux + 1.0) * 2.5)));

        // Named / catalog-matched stars get a brighter circle to stand
        // out from the unmatched solve-field stars.
        const bool named = !star.name.empty() || !star.bayer.empty() ||
                           !star.hd.empty()   || !star.hip.empty();
        dc.SetPen(wxPen(named ? wxColour(255, 220, 80)
                              : wxColour(0,   200, 220), 1, wxPENSTYLE_SOLID));
        dc.DrawCircle(sx, sy, radius);

        // Build up to two label lines. Primary = the friendliest name we
        // have. Secondary = a cross-reference designation + magnitude.
        wxString line1, line2;
        if      (!star.name.empty())  line1 = star.name;
        else if (!star.bayer.empty()) line1 = star.bayer;
        else if (!star.hd.empty())    line1 = wxT("HD ")  + star.hd;
        else if (!star.hip.empty())   line1 = wxT("HIP ") + star.hip;
        else                          line1 = star.catalog_id;

        // Second line: the *other* designation (if we had a name, show
        // HD/HIP; if we had Bayer, show HD; etc.), plus magnitude.
        wxString xref;
        if (!star.name.empty() || !star.bayer.empty())
        {
            if      (!star.hd.empty())  xref = wxT("HD ")  + star.hd;
            else if (!star.hip.empty()) xref = wxT("HIP ") + star.hip;
        }
        if (std::isfinite((double) star.vmag))
        {
            if (!xref.empty()) xref += wxT("  ");
            xref += wxString::Format(wxT("%.1fm"), star.vmag);
        }
        line2 = xref;

        if (!line1.empty())
        {
            dc.SetTextForeground(named ? wxColour(255, 220, 80)
                                       : wxColour(0,   200, 220));
            dc.DrawText(line1, sx + radius + 2, sy - 2);
            if (!line2.empty())
            {
                dc.SetTextForeground(wxColour(180, 180, 180));
                dc.DrawText(line2, sx + radius + 2, sy + 10);
            }
        }
    }

    // --- Field info panel ---
    double raH  = wcs.crval1 / 15.0;
    int    raHr = (int) raH; double raM = (raH - raHr) * 60.0;
    int    raMn = (int) raM; double raS = (raM - raMn) * 60.0;
    double absDec = fabs(wcs.crval2);
    char   dSign  = wcs.crval2 >= 0 ? '+' : '-';
    int    decD   = (int) absDec; double decAM = (absDec - decD) * 60.0;
    int    decMn  = (int) decAM;  double decS  = (decAM - decMn) * 60.0;

    wxArrayString lines;
    lines.Add(wxString::Format("RA   %02dh %02dm %05.2fs",         raHr, raMn, raS));
    lines.Add(wxString::Format("Dec  %c%02d\xb0 %02d\' %04.1f\"",  dSign, decD, decMn, decS));
    lines.Add(wxString::Format("Scale %.2f\"/px",                  wcs.pixScale));
    lines.Add(wxString::Format("Rot   %.1f\xb0",                   wcs.rotation));
    lines.Add(wxString::Format("Stars %d",                         (int) result.stars.size()));

    wxFont infoFont(9, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD);
    dc.SetFont(infoFont);

    int lineH = 16, padX = 8, padY = 8, panelW = 0;
    for (const wxString& l : lines)
    {
        wxCoord w2, h2; dc.GetTextExtent(l, &w2, &h2);
        if (w2 > panelW) panelW = w2;
    }
    panelW += padX * 2;
    int panelH = lineH * (int) lines.size() + padY;

    dc.SetPen(wxPen(wxColour(0, 0, 0, 180), 1));
    dc.SetBrush(wxBrush(wxColour(0, 0, 0, 160)));
    dc.DrawRectangle(4, 4, panelW, panelH);
    dc.SetTextForeground(wxColour(200, 255, 200));
    int y = padY;
    for (const wxString& l : lines) { dc.DrawText(l, padX + 4, y); y += lineH; }

    dc.SetFont(wxFont(9, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD));
    dc.SetTextForeground(wxColour(100, 255, 100));
    wxString tag = _("SOLVED");
    wxCoord tw, th; dc.GetTextExtent(tag, &tw, &th);
    dc.DrawText(tag, imageW - tw - 8, 8);
}

#endif // __linux__
