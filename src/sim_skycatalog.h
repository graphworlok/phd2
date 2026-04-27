/*
 *  sim_skycatalog.h
 *  PHD Guiding
 *
 *  Populates the camera simulator with stars drawn from real
 *  astrometry.net index (.fits) files. Catalog (RA, Dec, magnitude)
 *  tuples are projected through a forward TAN WCS to produce
 *  pixel-space positions for the simulated frame, allowing
 *  end-to-end testing of plate solving and catalog-tagging against
 *  representative sky data.
 *
 *  query-starkd is invoked as a subprocess; the resulting FITS
 *  binary table is parsed via cfitsio. Both are Linux-only in PHD2,
 *  matching astrometry_solver.cpp's build constraint - the function
 *  returns false on other platforms and the caller falls back to
 *  the synthetic star generator.
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#ifndef SIM_SKYCATALOG_H_INCLUDED
#define SIM_SKYCATALOG_H_INCLUDED

#include <vector>

struct SimSkyPixelStar
{
    double pixel_x;   // 0-based image coordinate
    double pixel_y;
    float  vmag;      // catalog magnitude (J / V / H depending on index),
                      // NaN if absent
    double ra_deg;    // J2000 (kept for diagnostics / future overlay)
    double dec_deg;
};

// Build a simulated star field by querying an astrometry.net star
// kdtree for stars near (ra_deg, dec_deg), then projecting them
// through a TAN WCS sized to (frameW, frameH) at the supplied
// pixel scale and rotation.
//
//   indexPath    - a single .fits index file or a directory holding
//                  index-*.fits files. In the directory case the
//                  densest scale band that still spans the requested
//                  query radius is selected.
//   ra_deg/dec   - J2000 field center (degrees)
//   frameW,H     - simulated frame dimensions (pixels)
//   scale_asec   - pixel scale (arc-seconds per pixel)
//   cam_angle    - rotation East of North (degrees)
//   magLimit     - drop entries fainter than this (NaN-mag rows kept)
//   border_pix   - reject stars whose projection falls within this
//                  many pixels of the frame edge
//   outStars     - cleared then filled; in-frame pixel coords
//
// Returns true on success. Logs progress / failure via Debug.AddLine.
bool BuildSimSkyField(const wxString& indexPath,
                      double ra_deg, double dec_deg,
                      int frameW, int frameH,
                      double scale_asec,
                      double cam_angle_deg,
                      double magLimit,
                      int border_pix,
                      std::vector<SimSkyPixelStar>& outStars);

#endif // SIM_SKYCATALOG_H_INCLUDED
