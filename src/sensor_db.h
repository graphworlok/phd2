/*
 *  sensor_db.h
 *  PHD Guiding
 *
 *  Image-sensor database. Loaded from sensors.xml in the data dir at
 *  startup. Provides:
 *
 *    * Lookup of physical / electrical sensor properties (pixel size,
 *      array geometry, bit depth, full well, read noise) from a sensor
 *      part-number string.
 *
 *    * Enumeration of available sensor names for UI population.
 *
 *  The database is intentionally backend-agnostic: cameras whose SDK
 *  reports a sensor name can pass it straight through; cameras that
 *  don't know their sensor (typical for UVC) let the user pick.
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#ifndef SENSOR_DB_H_INCLUDED
#define SENSOR_DB_H_INCLUDED

#include <wx/string.h>
#include <wx/arrstr.h>
#include <vector>

enum class SensorColorArray
{
    Unknown = 0,
    Mono,
    RGGB,
    BGGR,
    GBRG,
    GRBG,
};

struct Sensor
{
    wxString          name;            // canonical key, e.g. "Sony IMX462"
    wxString          manufacturer;
    double            pixelSizeUm;     // 0.0 = unknown
    int               arrayWidth;      // 0 = unknown
    int               arrayHeight;     // 0 = unknown
    int               bitDepth;        // native ADC; 0 = unknown
    SensorColorArray  colorArray;
    double            fullWellE;       // 0.0 = unknown
    double            readNoiseE;      // 0.0 = unknown
    wxString          notes;

    Sensor()
        : pixelSizeUm(0.0), arrayWidth(0), arrayHeight(0), bitDepth(0),
          colorArray(SensorColorArray::Unknown),
          fullWellE(0.0), readNoiseE(0.0)
    {}

    bool IsKnown() const { return !name.empty(); }
    bool IsColor() const { return colorArray != SensorColorArray::Unknown &&
                                 colorArray != SensorColorArray::Mono; }
};

// Singleton-ish wrapper. The first call to Get() loads sensors.xml from
// the data directory; subsequent calls return the cached vector. Thread
// safety: all access happens on the GUI thread (config dialogs).
class SensorDatabase
{
public:
    // Reload from disk. Called automatically by Get() on first use; the
    // caller may invoke this explicitly to pick up a manually-edited
    // file. Returns true if the file was opened (even an empty/malformed
    // file is "true" - the caller can still query an empty database).
    static bool Reload();

    // Total number of entries known.
    static size_t Size();

    // Find a sensor by exact name match (case-sensitive). Returns a
    // pointer into the cached vector; lifetime is the program. Returns
    // nullptr when no match.
    static const Sensor *Find(const wxString& name);

    // Find a sensor by case-insensitive name match (preferred when the
    // name comes from a free-form database like uvc_cameras.xml whose
    // strings may not match exactly).
    static const Sensor *FindNoCase(const wxString& name);

    // Sorted list of all known names. Suitable for wxChoice population.
    // The first call may trigger a reload.
    static wxArrayString GetAllNames();

    // Convert a color-array string ("mono", "RGGB", ...) to enum, and back.
    static SensorColorArray ParseColorArray(const wxString& s);
    static wxString         ColorArrayToString(SensorColorArray ca);
};

#endif // SENSOR_DB_H_INCLUDED
