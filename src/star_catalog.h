/*
 *  star_catalog.h
 *  PHD Guiding
 *
 *  Lightweight local star catalog for plate-solve overlay tagging.
 *
 *  Loads a CSV of (ra, dec, vmag, hd, hip, name, bayer, spectrum)
 *  entries and supports nearest-star lookup by J2000 (ra, dec) within
 *  an angular tolerance. Used to decorate SolvedStar entries produced
 *  by the plate solver with human-readable names and magnitudes.
 *
 *  The CSV file is bright_stars.csv; PHD2 ships a small starter set
 *  and users may extend it or replace it with a larger catalog
 *  exported from VizieR/SIMBAD.
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#ifndef STAR_CATALOG_H_INCLUDED
#define STAR_CATALOG_H_INCLUDED

#include <vector>

struct StarCatalogEntry
{
    double   ra_deg;   // J2000
    double   dec_deg;  // J2000
    float    vmag;     // NaN if unknown
    wxString hd;       // numeric string, or empty
    wxString hip;      // numeric string, or empty
    wxString name;     // "Sirius", or empty
    wxString bayer;    // "alpha CMa", or empty
    wxString spectrum; // "A1V", or empty
};

class StarCatalog
{
    std::vector<StarCatalogEntry> m_entries;
    bool m_loaded = false;

public:
    StarCatalog() = default;

    // Load from CSV. Lines starting with # are ignored. First non-comment
    // line may be a header (auto-detected: skipped if its first two
    // fields are non-numeric).
    bool LoadCSV(const wxString& path);

    bool   IsLoaded() const { return m_loaded; }
    size_t Size()     const { return m_entries.size(); }

    // Find nearest entry within tol_arcsec of (ra_deg, dec_deg).
    // Returns nullptr if no match.
    const StarCatalogEntry *FindNearest(double ra_deg, double dec_deg,
                                        double tol_arcsec = 15.0) const;

    // Build a short human-readable label (name, else Bayer, else HD/HIP),
    // optionally appending magnitude.
    static wxString FormatLabel(const StarCatalogEntry& e, bool withMag = true);
};

extern StarCatalog *pStarCatalog;

void InitStarCatalog();
void ShutdownStarCatalog();

#endif // STAR_CATALOG_H_INCLUDED
