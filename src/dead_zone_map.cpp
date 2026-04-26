/*
 *  dead_zone_map.cpp
 *  PHD Guiding
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#include "phd.h"
#include "dead_zone_map.h"
#include "fitsiowrap.h"

#include <fitsio.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <queue>

DeadZoneMap::DeadZoneMap() = default;

void DeadZoneMap::Set(int width, int height, std::vector<uint8_t> mask, size_t regionCount)
{
    m_width = width;
    m_height = height;
    m_regionCount = regionCount;
    m_mask = std::move(mask);
}

void DeadZoneMap::Clear()
{
    m_width = 0;
    m_height = 0;
    m_regionCount = 0;
    m_mask.clear();
}

bool DeadZoneMap::IsDead(int x, int y) const
{
    if (m_mask.empty())
        return false;
    if ((unsigned) x >= (unsigned) m_width || (unsigned) y >= (unsigned) m_height)
        return false;
    return m_mask[(size_t) y * (size_t) m_width + (size_t) x] != 0;
}

DeadZoneStats DeadZoneMap::Stats() const
{
    DeadZoneStats s;
    s.width = m_width;
    s.height = m_height;
    s.regionCount = m_regionCount;
    for (uint8_t v : m_mask)
        if (v) ++s.maskedPixels;
    return s;
}

bool DeadZoneMap::Apply(usImage& img, bool fillWithMedian) const
{
    if (m_mask.empty() || !img.ImageData)
        return false;
    const int W = img.Size.GetWidth();
    const int H = img.Size.GetHeight();
    if (W != m_width || H != m_height)
        return false;

    unsigned short fill = 0;
    if (fillWithMedian)
    {
        // Median over the *unmasked* pixels only - using the masked
        // pixels would bias the median toward whatever those pixels
        // happen to read, which is the value we don't trust.
        std::vector<unsigned short> samples;
        samples.reserve((size_t) W * (size_t) H / 2);
        for (size_t i = 0, n = (size_t) W * (size_t) H; i < n; ++i)
            if (!m_mask[i])
                samples.push_back(img.ImageData[i]);
        if (!samples.empty())
        {
            const size_t mid = samples.size() / 2;
            std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
            fill = samples[mid];
        }
    }

    size_t replaced = 0;
    for (size_t i = 0, n = (size_t) W * (size_t) H; i < n; ++i)
    {
        if (m_mask[i])
        {
            img.ImageData[i] = fill;
            ++replaced;
        }
    }
    return replaced > 0;
}

wxString DeadZoneMap::MapFilePath(const wxString& camTag)
{
    if (camTag.empty())
        return wxString();
    return MyFrame::GetDarksDir() + PATHSEPSTR +
           wxT("PHD2_dead_zone_") + camTag + wxT(".fit");
}

bool DeadZoneMap::SaveToDisk(const wxString& camTag) const
{
    if (camTag.empty() || m_mask.empty())
        return false;
    const wxString path = MapFilePath(camTag);
    if (path.empty())
        return false;

    fitsfile *fptr = nullptr;
    int status = 0;
    PHD_fits_create_file(&fptr, path, true, &status);
    if (status)
        return false;

    long primAxes[1] = { 0 };
    fits_create_img(fptr, BYTE_IMG, 0, primAxes, &status);

    int width  = m_width;
    int height = m_height;
    int regions = (int) m_regionCount;
    fits_write_key(fptr, TINT, "MASKW", &width,  "Mask width (px)",  &status);
    fits_write_key(fptr, TINT, "MASKH", &height, "Mask height (px)", &status);
    fits_write_key(fptr, TINT, "REGIONS", &regions,
                   "Connected dead regions", &status);

    long axes[2] = { (long) width, (long) height };
    long fpixel[2] = { 1, 1 };
    long npix = (long) width * (long) height;
    fits_create_img(fptr, BYTE_IMG, 2, axes, &status);
    char extName[FLEN_VALUE] = "MASK";
    fits_write_key(fptr, TSTRING, "EXTNAME", extName, 0, &status);
    fits_write_pix(fptr, TBYTE, fpixel, npix,
                   (void *) m_mask.data(), &status);

    PHD_fits_close_file(fptr);
    if (status)
    {
        Debug.Write(wxString::Format(
            "DeadZone: SaveToDisk - fitsio status=%d for %s\n", status, path));
        return false;
    }
    Debug.Write(wxString::Format(
        "DeadZone: saved mask %dx%d (%d regions) to %s\n",
        width, height, regions, path));
    return true;
}

bool DeadZoneMap::LoadFromDisk(const wxString& camTag)
{
    if (camTag.empty())
        return false;
    const wxString path = MapFilePath(camTag);
    if (path.empty() || !wxFileExists(path))
        return false;

    fitsfile *fptr = nullptr;
    int status = 0;
    if (PHD_fits_open_diskfile(&fptr, path, READONLY, &status) || status)
        return false;

    int width = 0, height = 0, regions = 0;
    char dummy[FLEN_COMMENT] = { 0 };
    fits_read_key(fptr, TINT, "MASKW", &width,  dummy, &status);
    fits_read_key(fptr, TINT, "MASKH", &height, dummy, &status);
    int rs = 0;
    fits_read_key(fptr, TINT, "REGIONS", &regions, dummy, &rs);

    int hdutype = 0;
    fits_movnam_hdu(fptr, IMAGE_HDU, (char *) "MASK", 0, &status);
    if (status)
    {
        PHD_fits_close_file(fptr);
        return false;
    }
    fits_get_hdu_type(fptr, &hdutype, &status);

    long npix = (long) width * (long) height;
    std::vector<uint8_t> mask((size_t) npix, (uint8_t) 0);
    long fpixel[2] = { 1, 1 };
    fits_read_pix(fptr, TBYTE, fpixel, npix, nullptr,
                  mask.data(), nullptr, &status);
    PHD_fits_close_file(fptr);
    if (status)
    {
        Debug.Write(wxString::Format(
            "DeadZone: LoadFromDisk - fitsio status=%d for %s\n",
            status, path));
        return false;
    }

    m_width = width;
    m_height = height;
    m_regionCount = (size_t) (rs == 0 ? regions : 0);
    m_mask = std::move(mask);
    Debug.Write(wxString::Format(
        "DeadZone: loaded mask %dx%d (%zu regions) from %s\n",
        width, height, m_regionCount, path));
    return true;
}

// ---------------------------------------------------------------------------
// DeadZoneBuilder
// ---------------------------------------------------------------------------

DeadZoneBuilder::DeadZoneBuilder() = default;

void DeadZoneBuilder::Begin(int w, int h)
{
    m_width = w;
    m_height = h;
    m_frameCount = 0;
    const size_t n = (size_t) w * (size_t) h;
    m_mean.assign(n, 0.0);
    m_M2.assign(n, 0.0);
}

bool DeadZoneBuilder::AddFrame(const usImage& frame)
{
    if (!frame.ImageData)
        return false;
    if (frame.Size.GetWidth() != m_width || frame.Size.GetHeight() != m_height)
        return false;

    ++m_frameCount;
    const size_t n = (size_t) m_width * (size_t) m_height;
    const double k = (double) m_frameCount;
    for (size_t i = 0; i < n; ++i)
    {
        const double x = (double) frame.ImageData[i];
        const double delta = x - m_mean[i];
        m_mean[i] += delta / k;
        const double delta2 = x - m_mean[i];
        m_M2[i] += delta * delta2;
    }
    return true;
}

namespace
{

// Median of a copy of `v`. Modifies a temp; original is untouched.
double medianCopy(const std::vector<double>& v)
{
    if (v.empty())
        return 0.0;
    std::vector<double> tmp(v);
    const size_t mid = tmp.size() / 2;
    std::nth_element(tmp.begin(), tmp.begin() + mid, tmp.end());
    return tmp[mid];
}

// Connected-components labelling on a uint8 candidate mask. Each
// connected blob (4-connectivity) gets sized; blobs outside the
// [minPx, maxPx] range are zeroed out of the mask. Returns the number
// of surviving regions. Done in-place to avoid a second buffer.
size_t LabelAndFilter(std::vector<uint8_t>& mask,
                      int W, int H,
                      int minPx, int maxPx)
{
    std::vector<int> label((size_t) W * (size_t) H, 0);
    int nextLabel = 0;
    std::vector<size_t> sizes;
    sizes.push_back(0); // label 0 unused

    std::queue<std::pair<int, int>> q;
    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            const size_t idx = (size_t) y * (size_t) W + (size_t) x;
            if (!mask[idx] || label[idx] != 0)
                continue;
            ++nextLabel;
            sizes.push_back(0);
            q.push({ x, y });
            label[idx] = nextLabel;
            while (!q.empty())
            {
                std::pair<int, int> cur = q.front();
                q.pop();
                const int cx = cur.first;
                const int cy = cur.second;
                ++sizes[nextLabel];
                static const int dx[4] = { -1, 1, 0, 0 };
                static const int dy[4] = { 0, 0, -1, 1 };
                for (int n = 0; n < 4; ++n)
                {
                    const int nx = cx + dx[n];
                    const int ny = cy + dy[n];
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H)
                        continue;
                    const size_t ni = (size_t) ny * (size_t) W + (size_t) nx;
                    if (mask[ni] && label[ni] == 0)
                    {
                        label[ni] = nextLabel;
                        q.push({ nx, ny });
                    }
                }
            }
        }
    }

    size_t survived = 0;
    std::vector<bool> keep(sizes.size(), false);
    for (size_t i = 1; i < sizes.size(); ++i)
    {
        if ((int) sizes[i] >= minPx && (int) sizes[i] <= maxPx)
        {
            keep[i] = true;
            ++survived;
        }
    }
    for (size_t i = 0, n = (size_t) W * (size_t) H; i < n; ++i)
    {
        if (label[i] == 0 || !keep[(size_t) label[i]])
            mask[i] = 0;
    }
    return survived;
}

void DilateOnce(std::vector<uint8_t>& mask, int W, int H)
{
    std::vector<uint8_t> out(mask.size(), 0);
    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            const size_t idx = (size_t) y * (size_t) W + (size_t) x;
            if (mask[idx])
            {
                out[idx] = 1;
                continue;
            }
            // 4-neighbour dilation; cheaper than 8 and visually adequate.
            if (x > 0     && mask[idx - 1])         { out[idx] = 1; continue; }
            if (x + 1 < W && mask[idx + 1])         { out[idx] = 1; continue; }
            if (y > 0     && mask[idx - W])         { out[idx] = 1; continue; }
            if (y + 1 < H && mask[idx + W])         { out[idx] = 1; continue; }
        }
    }
    mask.swap(out);
}

} // namespace

bool DeadZoneBuilder::Finalize(const Params& params, DeadZoneMap *out) const
{
    if (!out || m_frameCount < 2 || m_width <= 0 || m_height <= 0)
        return false;

    const size_t n = (size_t) m_width * (size_t) m_height;

    // Per-pixel stdev. Use n-1 for unbiased estimate.
    std::vector<double> stdev(n, 0.0);
    const double divisor = (double) (m_frameCount - 1);
    for (size_t i = 0; i < n; ++i)
        stdev[i] = std::sqrt(m_M2[i] / divisor);

    const double medStdev = medianCopy(stdev);
    const double medMean  = medianCopy(m_mean);

    const float ratio = std::max(params.sigmaMultiplier, 1.0f);
    const double thrStdev = medStdev * (double) ratio;
    const double thrMean  = medMean + (double) std::max(0, params.absoluteOffset);

    // Build the candidate mask: pixels that are either very noisy OR
    // significantly hotter than the median dark level.
    std::vector<uint8_t> mask(n, 0);
    for (size_t i = 0; i < n; ++i)
    {
        if (stdev[i] > thrStdev || m_mean[i] > thrMean)
            mask[i] = 1;
    }

    const int frameArea = m_width * m_height;
    const int maxRegionPx =
        (int) std::lround((double) frameArea * (double) std::max(0.01f, params.maxRegionFrac));
    const int minRegionPx = std::max(1, params.minRegionPx);

    const size_t regions = LabelAndFilter(mask, m_width, m_height,
                                          minRegionPx, maxRegionPx);

    if (params.dilateOnePixel)
        DilateOnce(mask, m_width, m_height);

    out->Set(m_width, m_height, std::move(mask), regions);

    Debug.Write(wxString::Format(
        "DeadZone: built mask from %d frames, medStdev=%.2f, "
        "medMean=%.1f, %zu regions survived\n",
        m_frameCount, medStdev, medMean, regions));
    return true;
}
