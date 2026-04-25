/*
 *  cam_v4l2.cpp
 *  PHD Guiding
 *
 *  UVC camera support via the V4L2 (Video4Linux2) kernel API.
 *  Supports any UVC-compliant webcam or astronomy camera that exposes a
 *  /dev/videoN device node.
 *
 *  Two optional data files are loaded at connect time from the PHD2 data
 *  directory (or the directory containing the phd2 executable):
 *
 *    uvc_cameras.xml  – maps USB VID:PID to sensor name and pixel size
 *    uvc_config.ini   – presets capture options globally and per device
 *
 *  Copyright (c) 2024 PHD2 Contributors
 *  All rights reserved.
 *
 *  This source code is distributed under the following "BSD" license
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *    Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *    Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *    Neither the name of Craig Stark, Stark Labs nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "phd.h"

#ifdef V4L2_CAMERA

# include "cam_v4l2.h"

# include <wx/stdpaths.h>
# include <wx/dir.h>
# include <wx/filename.h>
# include <wx/textfile.h>
# include <wx/tokenzr.h>

# include <linux/videodev2.h>
# include <fcntl.h>
# include <unistd.h>
# include <algorithm>
# include <cerrno>
# include <climits>
# include <cstring>
# include <sys/ioctl.h>
# include <sys/mman.h>
# include <sys/select.h>
# include <sys/stat.h>

# if defined(HAVE_LIBJPEG)
#  include <csetjmp>
#  include <cstdio>
extern "C" {
#  include <jpeglib.h>
}

namespace
{

// Custom error manager so a corrupt MJPEG frame doesn't kill the
// process via the libjpeg default error_exit (which calls exit()).
struct JpegErrMgr
{
    struct jpeg_error_mgr pub;
    std::jmp_buf          jb;
};

extern "C" void PhdJpegErrorExit(j_common_ptr cinfo)
{
    JpegErrMgr *err = reinterpret_cast<JpegErrMgr *>(cinfo->err);
    char msg[JMSG_LENGTH_MAX];
    (*cinfo->err->format_message)(cinfo, msg);
    Debug.AddLine(wxString::Format("V4L2: libjpeg error: %s", msg));
    std::longjmp(err->jb, 1);
}

extern "C" void PhdJpegOutputMessage(j_common_ptr) { /* swallow */ }

// Decode a JPEG/MJPEG byte buffer into a contiguous 8-bit grayscale
// buffer of width*height bytes. Returns true on success. The caller
// guarantees outBytes has room for width*height bytes.
//
// V4L2's MJPEG payload is JPEG with the standard quantisation and
// Huffman tables present in each frame (UVC class spec 1.5 mandates
// this), so we can pass the buffer straight to libjpeg without the
// out-of-band-table reconstruction some embedded MJPEG variants need.
static bool DecodeMjpegToGray(const unsigned char *src, size_t srcSize,
                              unsigned int width, unsigned int height,
                              unsigned char *outBytes)
{
    struct jpeg_decompress_struct cinfo;
    JpegErrMgr err;

    cinfo.err = jpeg_std_error(&err.pub);
    err.pub.error_exit     = PhdJpegErrorExit;
    err.pub.output_message = PhdJpegOutputMessage;

    if (setjmp(err.jb))
    {
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, const_cast<unsigned char *>(src),
                 (unsigned long) srcSize);

    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK)
    {
        jpeg_destroy_decompress(&cinfo);
        return false;
    }
    cinfo.out_color_space = JCS_GRAYSCALE;
    cinfo.dct_method      = JDCT_IFAST;     // speed > exactness for guiding

    if (!jpeg_start_decompress(&cinfo))
    {
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    // Validate decoded geometry matches what V4L2 negotiated.
    if (cinfo.output_width != width || cinfo.output_height != height ||
        cinfo.output_components != 1)
    {
        Debug.AddLine(wxString::Format(
            "V4L2: MJPEG geometry mismatch: %ux%u/%d vs expected %ux%u",
            cinfo.output_width, cinfo.output_height,
            cinfo.output_components, width, height));
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    while (cinfo.output_scanline < cinfo.output_height)
    {
        unsigned char *rows[1];
        rows[0] = outBytes + (size_t) cinfo.output_scanline * (size_t) width;
        jpeg_read_scanlines(&cinfo, rows, 1);
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    return true;
}

} // namespace
# endif // HAVE_LIBJPEG

// Default number of mmap capture buffers
static const int V4L2_DEFAULT_NUM_BUFFERS = 4;

// Profile key prefix for per-device settings
static const char *const V4L2_CFG_PFX = "/camera/v4l2";

// ---------------------------------------------------------------------------
// UVCDeviceConfig defaults
// ---------------------------------------------------------------------------

UVCDeviceConfig::UVCDeviceConfig()
{
    // Compiled-in format preference order: YUYV first (required by UVC spec),
    // then greyscale variants, MJPEG last.
    preferredFormats[0] = V4L2_PIX_FMT_YUYV;
    preferredFormats[1] = V4L2_PIX_FMT_GREY;
    preferredFormats[2] = V4L2_PIX_FMT_Y16;
    preferredFormats[3] = V4L2_PIX_FMT_MJPEG;
    preferredFormats[4] = 0;

    width        = 0;   // let driver choose
    height       = 0;   // let driver choose
    numBuffers   = -1;  // use V4L2_DEFAULT_NUM_BUFFERS
    exposureMode = UVC_EXPOSURE_DEFAULT; // do not touch
    gain         = -1;  // do not preset
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static int xioctl(int fd, unsigned long request, void *arg)
{
    int r;
    do
    {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

// Query a V4L2 control's availability and range.
// Returns true and fills *info if the control is available and enabled.
static bool QueryCtrl(int fd, unsigned int cid, V4L2CtrlInfo *info)
{
    struct v4l2_queryctrl qc;
    memset(&qc, 0, sizeof(qc));
    qc.id = cid;
    if (xioctl(fd, VIDIOC_QUERYCTRL, &qc) < 0 ||
        (qc.flags & V4L2_CTRL_FLAG_DISABLED))
    {
        info->available = false;
        return false;
    }
    info->available = true;
    info->minimum   = qc.minimum;
    info->maximum   = qc.maximum;
    info->step      = qc.step;
    info->def       = qc.default_value;
    return true;
}

// Extract the /dev/videoN path from a camId.
// Handles:
//   "/dev/video0"                 -> "/dev/video0"
//   "Logitech C270 (/dev/video0)" -> "/dev/video0"
//   "0"                           -> "/dev/video0"
static wxString DeviceNodeFromId(const wxString& camId)
{
    if (camId.StartsWith("/dev/"))
        return camId;

    int pos = camId.Find("/dev/video");
    if (pos != wxNOT_FOUND)
    {
        wxString tail = camId.Mid(pos);
        while (!tail.empty() && (tail.Last() == ')' || tail.Last() == ' '))
            tail = tail.Left(tail.length() - 1);
        return tail;
    }

    long n = 0;
    if (camId.ToLong(&n))
        return wxString::Format("/dev/video%ld", n);

    return camId;
}

// Map a format name string (case-insensitive) to a V4L2 fourcc code.
// Returns 0 for unknown names.
static unsigned int FourccFromName(const wxString& name)
{
    wxString u = name.Upper().Trim(true).Trim(false);
    if (u == "YUYV" || u == "YUY2") return V4L2_PIX_FMT_YUYV;
    if (u == "MJPEG" || u == "MJPG") return V4L2_PIX_FMT_MJPEG;
    if (u == "GREY" || u == "GRAY" || u == "Y8") return V4L2_PIX_FMT_GREY;
    if (u == "Y16")  return V4L2_PIX_FMT_Y16;
    if (u == "YVYU") return V4L2_PIX_FMT_YVYU;
    if (u == "UYVY") return V4L2_PIX_FMT_UYVY;
    return 0;
}

// Parse a location hint for a data file: checks the PHD2 data directory
// then falls back to the directory containing the executable.
static wxString FindDataFile(const wxString& filename)
{
    wxString path = wxStandardPaths::Get().GetDataDir() + "/" + filename;
    if (wxFileExists(path))
        return path;
    wxString exeDir = wxFileName(wxStandardPaths::Get().GetExecutablePath()).GetPath();
    path = exeDir + "/" + filename;
    if (wxFileExists(path))
        return path;
    return wxEmptyString;
}

// Extract text between <tag> and </tag> on a single line.
static bool XmlLineContent(const wxString& line, const wxString& tag, wxString *value)
{
    wxString open  = "<"  + tag + ">";
    wxString close = "</" + tag + ">";
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

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

CameraV4L2::CameraV4L2()
    : m_fd(-1),
      m_pixelFormat(0),
      m_captureWidth(0),
      m_captureHeight(0),
      m_capVersion(0),
      m_capFlags(0),
      m_usbVendorId(0),
      m_usbProductId(0),
      m_dbPixelSizeUm(0.0)
{
    Connected = false;
    Name = _T("V4L2 Camera");
    FrameSize = wxSize(640, 480);
    m_hasGuideOutput = false;
    HasGainControl = false;
    HasShutter = false;
    HasSubframes = false;
    HasFrameLimiting = false;
    HasCooler = false;
    HasBayer = false;
    MaxHwBinning = 1;
    PropertyDialogType = PROPDLG_WHEN_CONNECTED;
}

CameraV4L2::~CameraV4L2()
{
    if (Connected)
        Disconnect();
}

// ---------------------------------------------------------------------------
// BitsPerPixel
// ---------------------------------------------------------------------------

wxByte CameraV4L2::BitsPerPixel()
{
    return 16;
}

// ---------------------------------------------------------------------------
// EnumCameras
// ---------------------------------------------------------------------------

bool CameraV4L2::EnumCameras(wxArrayString& names, wxArrayString& ids)
{
    std::vector<UVCCameraInfo> db;
    LoadUvcDatabase(db);

    wxDir sysDir("/sys/class/video4linux");
    if (!sysDir.IsOpened())
    {
        Debug.AddLine("V4L2: /sys/class/video4linux not accessible");
        return false;
    }

    wxString entry;
    bool found = sysDir.GetFirst(&entry, "video*", wxDIR_DIRS | wxDIR_FILES);
    while (found)
    {
        wxString devPath = "/dev/" + entry;

        int fd = open(devPath.fn_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0)
        {
            found = sysDir.GetNext(&entry);
            continue;
        }

        struct v4l2_capability cap;
        memset(&cap, 0, sizeof(cap));
        bool isCapture = false;
        if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == 0)
        {
            isCapture = (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE) &&
                        (cap.device_caps & V4L2_CAP_STREAMING);
        }
        close(fd);

        if (isCapture)
        {
            wxString friendlyName;
            unsigned int vid = 0, pid = 0;
            if (!db.empty() && GetUsbIds(devPath, &vid, &pid))
            {
                UVCCameraInfo info;
                if (LookupUvcCamera(vid, pid, db, &info))
                    friendlyName = wxString::Format("%s [%s] (%s)",
                        info.name, info.sensor, devPath);
            }
            if (friendlyName.empty())
                friendlyName = wxString::Format("%s (%s)",
                    wxString::FromAscii((const char *)cap.card), devPath);

            names.Add(friendlyName);
            ids.Add(devPath);
        }

        found = sysDir.GetNext(&entry);
    }

    return !ids.empty();
}

// ---------------------------------------------------------------------------
// Connect
// ---------------------------------------------------------------------------

bool CameraV4L2::Connect(const wxString& camId)
{
    // camId is the device path stored by the gear dialog (e.g. "/dev/video0").
    // On first connect it may be empty if the user never clicked the select
    // button — fall back to extracting the path from the camera's Name string
    // (which was set by GuideCameraList from the display name
    // "PC Camera: (/dev/video0)") or from camId directly.
    wxString devPath = DeviceNodeFromId(camId.empty() ? Name : camId);

    if (!OpenDevice(devPath))
    {
        pFrame->Alert(wxString::Format(_("Cannot open V4L2 device %s: %s"),
            devPath, wxString::FromUTF8(strerror(errno))));
        return true;
    }

    // Query capability
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    if (xioctl(m_fd, VIDIOC_QUERYCAP, &cap) < 0)
    {
        pFrame->Alert(_("V4L2: VIDIOC_QUERYCAP failed"));
        CloseDevice();
        return true;
    }
    if (!(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE))
    {
        pFrame->Alert(wxString::Format(_("%s is not a video capture device"), devPath));
        CloseDevice();
        return true;
    }
    if (!(cap.device_caps & V4L2_CAP_STREAMING))
    {
        pFrame->Alert(wxString::Format(_("%s does not support streaming I/O"), devPath));
        CloseDevice();
        return true;
    }

    // Cache capability strings so the property dialog can show them
    // without re-querying.
    m_capDriver  = wxString::FromAscii((const char *) cap.driver);
    m_capCard    = wxString::FromAscii((const char *) cap.card);
    m_capBusInfo = wxString::FromAscii((const char *) cap.bus_info);
    m_capVersion = cap.version;
    m_capFlags   = cap.device_caps;

    // Resolve USB IDs, look up the camera database, then load config.
    // Database lookup happens first so name/sensor are available to the
    // config loader for [name:...] and [sensor:...] section matching.
    unsigned int vid = 0, pid = 0;
    GetUsbIds(devPath, &vid, &pid);
    m_usbVendorId  = vid;
    m_usbProductId = pid;
    m_usbSerial    = ReadUsbSerial(devPath);

    wxString dbCameraName, dbSensorName;
    {
        std::vector<UVCCameraInfo> db;
        if (LoadUvcDatabase(db))
        {
            UVCCameraInfo info;
            if (LookupUvcCamera(vid, pid, db, &info))
            {
                dbCameraName = info.name;
                dbSensorName = info.sensor;
                Name = info.name;
                m_dbCameraName  = info.name;
                m_dbSensorName  = info.sensor;
                m_dbPixelSizeUm = info.pixelSizeUm;
                if (info.pixelSizeUm > 0.0)
                    SetCameraPixelSize(info.pixelSizeUm);
                Debug.AddLine(wxString::Format(
                    "V4L2: matched USB %04x:%04x => %s, sensor=%s, pixel=%.2f um",
                    vid, pid, info.name, info.sensor, info.pixelSizeUm));
            }
            else
            {
                m_dbPixelSizeUm = 0.0;
            }
        }
    }

    // Enumerate every pixel format the device claims to support so the
    // info dialog can show which formats are available (vs the one PHD2
    // actually selected).
    m_supportedFormats.clear();
    {
        struct v4l2_fmtdesc fmtd;
        memset(&fmtd, 0, sizeof(fmtd));
        fmtd.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        for (fmtd.index = 0; xioctl(m_fd, VIDIOC_ENUM_FMT, &fmtd) == 0; fmtd.index++)
            m_supportedFormats.push_back(fmtd.pixelformat);
    }

    LoadUvcConfig(vid, pid, dbCameraName, dbSensorName, &m_config);

    // Apply pixel format preference order from config.
    // Resolution: profile-saved value takes priority over uvc_config.ini.
    m_pixelFormat = 0;
    unsigned int savedW = (unsigned int) pConfig->Profile.GetInt(
        wxString(V4L2_CFG_PFX) + "/width", 0);
    unsigned int savedH = (unsigned int) pConfig->Profile.GetInt(
        wxString(V4L2_CFG_PFX) + "/height", 0);
    unsigned int reqW = savedW > 0 ? savedW :
                        (m_config.width  > 0 ? m_config.width  : 640);
    unsigned int reqH = savedH > 0 ? savedH :
                        (m_config.height > 0 ? m_config.height : 480);

    for (int i = 0; m_config.preferredFormats[i]; i++)
    {
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = reqW;
        fmt.fmt.pix.height      = reqH;
        fmt.fmt.pix.pixelformat = m_config.preferredFormats[i];
        fmt.fmt.pix.field       = V4L2_FIELD_ANY;
        if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) == 0 &&
            fmt.fmt.pix.pixelformat == m_config.preferredFormats[i])
        {
            m_pixelFormat   = fmt.fmt.pix.pixelformat;
            m_captureWidth  = fmt.fmt.pix.width;
            m_captureHeight = fmt.fmt.pix.height;
            break;
        }
    }
    if (m_pixelFormat == 0)
    {
        pFrame->Alert(_("V4L2: no supported pixel format found on this device"));
        CloseDevice();
        return true;
    }

    Debug.AddLine(wxString::Format("V4L2: using format %4.4s %ux%u on %s",
        (const char *)&m_pixelFormat, m_captureWidth, m_captureHeight, devPath));

    // Enumerate supported resolutions for the chosen pixel format
    EnumResolutions();

    // Query available controls and their ranges
    QueryCtrl(m_fd, V4L2_CID_EXPOSURE_AUTO,     &m_ctrlExposureAuto);
    QueryCtrl(m_fd, V4L2_CID_EXPOSURE_ABSOLUTE, &m_ctrlExposureAbs);
    QueryCtrl(m_fd, V4L2_CID_GAIN,              &m_ctrlGain);
    QueryCtrl(m_fd, V4L2_CID_GAMMA,             &m_ctrlGamma);
    QueryCtrl(m_fd, V4L2_CID_BRIGHTNESS,        &m_ctrlBrightness);
    QueryCtrl(m_fd, V4L2_CID_CONTRAST,          &m_ctrlContrast);

    HasGainControl = m_ctrlGain.available;

    Debug.AddLine(wxString::Format(
        "V4L2: controls: exp_auto=%d exp_abs=%d gain=%d gamma=%d brightness=%d contrast=%d",
        m_ctrlExposureAuto.available, m_ctrlExposureAbs.available,
        m_ctrlGain.available, m_ctrlGamma.available,
        m_ctrlBrightness.available, m_ctrlContrast.available));

    // Apply exposure mode: prefer profile-saved setting, fall back to uvc_config preset
    {
        int savedMode = pConfig->Profile.GetInt(
            wxString(V4L2_CFG_PFX) + "/exposure_mode", -1);
        UVCExposureMode expMode = (savedMode == 1) ? UVC_EXPOSURE_MANUAL :
                                  (savedMode == 2) ? UVC_EXPOSURE_AUTO   :
                                  m_config.exposureMode;
        if (m_ctrlExposureAuto.available && expMode != UVC_EXPOSURE_DEFAULT)
        {
            SetControl(V4L2_CID_EXPOSURE_AUTO,
                       (expMode == UVC_EXPOSURE_AUTO) ? V4L2_EXPOSURE_AUTO
                                                      : V4L2_EXPOSURE_MANUAL);
            Debug.AddLine(wxString::Format("V4L2: exposure mode set to %s",
                expMode == UVC_EXPOSURE_AUTO ? "auto" : "manual"));
        }
    }

    // Apply preset gain (from uvc_config, not yet overridden by PHD2 gain slider)
    if (m_ctrlGain.available && m_config.gain >= 0)
    {
        TrySetGain(m_config.gain);
        Debug.AddLine(wxString::Format("V4L2: preset gain %d applied", m_config.gain));
    }

    // Apply profile-saved gamma / brightness / contrast
    if (m_ctrlGamma.available)
    {
        int v = pConfig->Profile.GetInt(wxString(V4L2_CFG_PFX) + "/gamma", INT_MIN);
        if (v != INT_MIN)
            TrySetGamma(v);
    }
    if (m_ctrlBrightness.available)
    {
        int v = pConfig->Profile.GetInt(wxString(V4L2_CFG_PFX) + "/brightness", INT_MIN);
        if (v != INT_MIN)
            TrySetBrightness(v);
    }
    if (m_ctrlContrast.available)
    {
        int v = pConfig->Profile.GetInt(wxString(V4L2_CFG_PFX) + "/contrast", INT_MIN);
        if (v != INT_MIN)
            TrySetContrast(v);
    }

    int numBufs = (m_config.numBuffers > 0) ? m_config.numBuffers
                                             : V4L2_DEFAULT_NUM_BUFFERS;
    if (!InitMmap(numBufs))
    {
        CloseDevice();
        return true;
    }
    if (!StartStreaming())
    {
        UninitMmap();
        CloseDevice();
        return true;
    }

    FrameSize = wxSize(m_captureWidth, m_captureHeight);
    m_devicePath = devPath;

    // Build a stable hardware id so the dark library / BPM / per-camera
    // calibration files can verify they belong to this physical device.
    // Format: "USB:VVVV:PPPP" or "USB:VVVV:PPPP:<serial>" if a serial
    // string is exposed by the kernel for this USB device.
    {
        wxString hwId;
        if (m_usbVendorId != 0 || m_usbProductId != 0)
        {
            hwId = wxString::Format("USB:%04x:%04x",
                                    m_usbVendorId, m_usbProductId);
            if (!m_usbSerial.empty())
                hwId += wxT(":") + m_usbSerial;
        }
        m_hardwareId = hwId;
        Debug.AddLine(wxString::Format("V4L2: hardware id = '%s'",
            m_hardwareId.empty() ? wxString("<none>") : m_hardwareId));
    }

    Connected = true;
    return false;
}

wxString CameraV4L2::GetHardwareId() const
{
    return m_hardwareId;
}

// Forward declaration - definition near end of file with the other
// fourcc helpers.
static wxString FourccToString(unsigned int f);

wxString CameraV4L2::GetCurrentModeTag() const
{
    if (!Connected || m_pixelFormat == 0)
        return wxEmptyString;
    // FourccToString already replaces non-printable bytes with '?'; the
    // sanitiser in the caller will then turn those into '_'. The 4-char
    // ASCII fourcc is enough to discriminate every UVC pixel format.
    return FourccToString(m_pixelFormat);
}

wxString CameraV4L2::ReadUsbSerial(const wxString& devicePath)
{
    wxString nodeName = wxFileName(devicePath).GetFullName();
    wxString sysBase  = "/sys/class/video4linux/" + nodeName + "/device";

    // Walk up to the USB device directory looking for a "serial" attr.
    for (int depth = 0; depth < 6; depth++)
    {
        wxString f = sysBase + "/serial";
        if (wxFileExists(f))
        {
            wxString s;
            if (wxFile(f).ReadAll(&s))
            {
                s.Trim(true).Trim(false);
                // Strip whitespace and unprintable chars; some devices
                // pad with NULs.
                wxString clean;
                for (size_t i = 0; i < s.length(); ++i)
                {
                    wxChar c = s[i];
                    if (c >= 0x20 && c < 0x7f)
                        clean += c;
                }
                return clean;
            }
            break;
        }
        sysBase += "/..";
    }
    return wxEmptyString;
}

// ---------------------------------------------------------------------------
// Disconnect
// ---------------------------------------------------------------------------

bool CameraV4L2::Disconnect()
{
    StopStreaming();
    UninitMmap();
    CloseDevice();
    m_hardwareId.clear();
    m_capDriver.clear();
    m_capCard.clear();
    m_capBusInfo.clear();
    m_capVersion = 0;
    m_capFlags = 0;
    m_usbVendorId = 0;
    m_usbProductId = 0;
    m_usbSerial.clear();
    m_dbCameraName.clear();
    m_dbSensorName.clear();
    m_dbPixelSizeUm = 0.0;
    m_supportedFormats.clear();
    Connected = false;
    return false;
}

// ---------------------------------------------------------------------------
// Capture
// ---------------------------------------------------------------------------

bool CameraV4L2::Capture(usImage& img, const CaptureParams& captureParams)
{
    int duration = captureParams.duration;

    if (img.Init(m_captureWidth, m_captureHeight))
    {
        pFrame->Alert(_("Memory allocation error"));
        return true;
    }
    img.Clear();

    // Set exposure: only override hardware when in manual (or default) mode.
    // If the profile says "auto", let the camera pick exposure.
    {
        int savedMode = pConfig->Profile.GetInt(
            wxString(V4L2_CFG_PFX) + "/exposure_mode", -1);
        bool isAuto = (savedMode == 2) ||
                      (savedMode < 0 && m_config.exposureMode == UVC_EXPOSURE_AUTO);
        if (!isAuto)
            TrySetExposure(duration);
    }

    // Apply per-capture gain from PHD2 if not overridden by a preset
    if (m_ctrlGain.available && m_config.gain < 0)
        TrySetGain(captureParams.gain);

    // Accumulate frames for the requested duration.
    // Use a heap-allocated intermediate buffer to avoid large stack allocations
    // (a 1280x960 frame is ~2.4 MB — far too large for the stack).
    const unsigned int npixels = m_captureWidth * m_captureHeight;
    std::vector<unsigned short> frameBuf(npixels);

    wxStopWatch sw;
    bool firstFrame = true;
    do
    {
        if (!GrabFrame(frameBuf.data(), npixels))
            return true;

        if (firstFrame)
        {
            memcpy(img.ImageData, frameBuf.data(), npixels * sizeof(unsigned short));
            firstFrame = false;
        }
        else
        {
            for (unsigned int i = 0; i < npixels; i++)
            {
                unsigned int sum = (unsigned int) img.ImageData[i]
                                 + (unsigned int) frameBuf[i];
                img.ImageData[i] = (unsigned short)(sum > 65535 ? 65535 : sum);
            }
        }
    } while (sw.Time() < duration);

    // Standard post-capture hook: runs dark/defect correction and the
    // noise-reduction pipeline. Every other camera backend calls this;
    // the V4L2 backend was missing it, which is why pNoise->Process
    // never fired for UVC cameras.
    SubtractDark(img);

    img.CalcStats();
    return false;
}

// ---------------------------------------------------------------------------
// GetDevicePixelSize
// ---------------------------------------------------------------------------

bool CameraV4L2::GetDevicePixelSize(double *devPixelSize)
{
    double ps = GetCameraPixelSize();
    if (ps == UnknownPixelSize)
        return false;
    *devPixelSize = ps;
    return true;
}

// ---------------------------------------------------------------------------
// OpenDevice / CloseDevice
// ---------------------------------------------------------------------------

bool CameraV4L2::OpenDevice(const wxString& path)
{
    m_fd = open(path.fn_str(), O_RDWR | O_NONBLOCK);
    return m_fd >= 0;
}

void CameraV4L2::CloseDevice()
{
    if (m_fd >= 0)
    {
        close(m_fd);
        m_fd = -1;
    }
}

// ---------------------------------------------------------------------------
// mmap buffer management
// ---------------------------------------------------------------------------

bool CameraV4L2::InitMmap(int numBuffers)
{
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = (unsigned int) numBuffers;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(m_fd, VIDIOC_REQBUFS, &req) < 0)
    {
        pFrame->Alert(_("V4L2: VIDIOC_REQBUFS failed"));
        return false;
    }
    if (req.count < 2)
    {
        pFrame->Alert(_("V4L2: insufficient buffer memory"));
        return false;
    }

    m_buffers.resize(req.count);
    for (unsigned int i = 0; i < req.count; i++)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;

        if (xioctl(m_fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            pFrame->Alert(_("V4L2: VIDIOC_QUERYBUF failed"));
            for (unsigned int j = 0; j < i; j++)
                munmap(m_buffers[j].start, m_buffers[j].length);
            m_buffers.clear();
            return false;
        }

        m_buffers[i].length = buf.length;
        m_buffers[i].start  = mmap(NULL, buf.length,
                                    PROT_READ | PROT_WRITE,
                                    MAP_SHARED,
                                    m_fd, buf.m.offset);
        if (m_buffers[i].start == MAP_FAILED)
        {
            pFrame->Alert(_("V4L2: mmap failed"));
            for (unsigned int j = 0; j < i; j++)
                munmap(m_buffers[j].start, m_buffers[j].length);
            m_buffers.clear();
            return false;
        }
    }
    return true;
}

void CameraV4L2::UninitMmap()
{
    for (auto& buf : m_buffers)
        if (buf.start && buf.start != MAP_FAILED)
            munmap(buf.start, buf.length);
    m_buffers.clear();
}

// ---------------------------------------------------------------------------
// Streaming start / stop
// ---------------------------------------------------------------------------

bool CameraV4L2::StartStreaming()
{
    for (unsigned int i = 0; i < m_buffers.size(); i++)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (xioctl(m_fd, VIDIOC_QBUF, &buf) < 0)
        {
            pFrame->Alert(_("V4L2: VIDIOC_QBUF failed"));
            return false;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(m_fd, VIDIOC_STREAMON, &type) < 0)
    {
        pFrame->Alert(_("V4L2: VIDIOC_STREAMON failed"));
        return false;
    }
    return true;
}

void CameraV4L2::StopStreaming()
{
    if (m_fd < 0)
        return;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(m_fd, VIDIOC_STREAMOFF, &type);
}

// ---------------------------------------------------------------------------
// GrabFrame
// ---------------------------------------------------------------------------

bool CameraV4L2::GrabFrame(unsigned short *dst, unsigned int npixels)
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(m_fd, &fds);
    struct timeval tv = { 5, 0 };
    int ret = select(m_fd + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0)
    {
        Debug.AddLine(wxString::Format("V4L2: select() returned %d (%s)",
            ret, strerror(errno)));
        return false;
    }

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(m_fd, VIDIOC_DQBUF, &buf) < 0)
    {
        Debug.AddLine(wxString::Format("V4L2: VIDIOC_DQBUF failed: %s",
            strerror(errno)));
        return false;
    }

    if (buf.index >= m_buffers.size())
    {
        Debug.AddLine(wxString::Format("V4L2: buffer index %u out of range",
            buf.index));
        return false;
    }

    ConvertToGray16(m_buffers[buf.index].start, buf.bytesused, dst, npixels);

    if (xioctl(m_fd, VIDIOC_QBUF, &buf) < 0)
    {
        Debug.AddLine(wxString::Format("V4L2: VIDIOC_QBUF re-enqueue failed: %s",
            strerror(errno)));
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Pixel format conversion
// ---------------------------------------------------------------------------

void CameraV4L2::ConvertToGray16(const void *src, unsigned int srcSize,
                                  unsigned short *dst, unsigned int npixels) const
{
    const unsigned char *s = static_cast<const unsigned char *>(src);

    if (m_pixelFormat == V4L2_PIX_FMT_GREY)
    {
        for (unsigned int i = 0; i < npixels && i < srcSize; i++)
            dst[i] = (unsigned short) s[i] << 8;
    }
    else if (m_pixelFormat == V4L2_PIX_FMT_Y16)
    {
        const unsigned short *s16 = static_cast<const unsigned short *>(src);
        unsigned int count = srcSize / 2;
        for (unsigned int i = 0; i < npixels && i < count; i++)
            dst[i] = s16[i];
    }
    else if (m_pixelFormat == V4L2_PIX_FMT_YUYV ||
             m_pixelFormat == V4L2_PIX_FMT_YVYU)
    {
        // Y is at byte 0 and 2 of each 4-byte macro-pixel
        unsigned int srcPixels = srcSize / 2;
        for (unsigned int i = 0; i < npixels && i < srcPixels; i++)
            dst[i] = (unsigned short) s[i * 2] << 8;
    }
    else if (m_pixelFormat == V4L2_PIX_FMT_UYVY)
    {
        // Y is at byte 1 and 3 of each 4-byte macro-pixel
        unsigned int srcPixels = srcSize / 2;
        for (unsigned int i = 0; i < npixels && i < srcPixels; i++)
            dst[i] = (unsigned short) s[i * 2 + 1] << 8;
    }
    else if (m_pixelFormat == V4L2_PIX_FMT_MJPEG)
    {
# if defined(HAVE_LIBJPEG)
        // Decode in-place to an 8-bit grayscale scratch buffer, then
        // upscale into the 16-bit destination. The scratch buffer lives
        // for the duration of the call - one allocation per frame is OK
        // since MJPEG decode dominates the cost anyway.
        std::vector<unsigned char> gray((size_t) m_captureWidth *
                                        (size_t) m_captureHeight);
        if (DecodeMjpegToGray(static_cast<const unsigned char *>(src), srcSize,
                              m_captureWidth, m_captureHeight, gray.data()))
        {
            const size_t n = std::min<size_t>(npixels, gray.size());
            for (size_t i = 0; i < n; ++i)
                dst[i] = (unsigned short) gray[i] << 8;
            if (n < npixels)
                std::memset(dst + n, 0, (npixels - n) * sizeof(unsigned short));
        }
        else
        {
            std::memset(dst, 0, npixels * sizeof(unsigned short));
        }
# else
        Debug.AddLine("V4L2: MJPEG capture but libjpeg not available - "
                      "rebuild with libjpeg-turbo to enable");
        std::memset(dst, 0, npixels * sizeof(unsigned short));
# endif
    }
    else
    {
        memset(dst, 0, npixels * sizeof(unsigned short));
    }
}

// ---------------------------------------------------------------------------
// V4L2 control helpers
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Generic control get / set
// ---------------------------------------------------------------------------

int CameraV4L2::GetControl(unsigned int cid)
{
    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = cid;
    if (xioctl(m_fd, VIDIOC_G_CTRL, &ctrl) < 0)
        return INT_MIN;
    return ctrl.value;
}

bool CameraV4L2::SetControl(unsigned int cid, int val)
{
    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id    = cid;
    ctrl.value = val;
    return xioctl(m_fd, VIDIOC_S_CTRL, &ctrl) == 0;
}

// ---------------------------------------------------------------------------
// Per-control set helpers
// ---------------------------------------------------------------------------

bool CameraV4L2::TrySetExposure(int durationMs)
{
    if (!m_ctrlExposureAuto.available && !m_ctrlExposureAbs.available)
        return false;

    if (m_ctrlExposureAuto.available)
        SetControl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);

    if (!m_ctrlExposureAbs.available)
        return false;

    // V4L2_CID_EXPOSURE_ABSOLUTE is in units of 100 µs
    int val = durationMs * 10;
    val = std::max((int)m_ctrlExposureAbs.minimum,
                   std::min((int)m_ctrlExposureAbs.maximum, val));
    return SetControl(V4L2_CID_EXPOSURE_ABSOLUTE, val);
}

bool CameraV4L2::TrySetGain(int gain)
{
    if (!m_ctrlGain.available)
        return false;

    long range = m_ctrlGain.maximum - m_ctrlGain.minimum;
    long val   = m_ctrlGain.minimum + (long)(gain * range / 100);
    return SetControl(V4L2_CID_GAIN, (int) val);
}

bool CameraV4L2::TrySetGamma(int val)
{
    if (!m_ctrlGamma.available)
        return false;
    val = std::max(m_ctrlGamma.minimum, std::min(m_ctrlGamma.maximum, val));
    bool ok = SetControl(V4L2_CID_GAMMA, val);
    Debug.AddLine(wxString::Format("V4L2: set gamma=%d %s", val, ok ? "ok" : strerror(errno)));
    return ok;
}

bool CameraV4L2::TrySetBrightness(int val)
{
    if (!m_ctrlBrightness.available)
        return false;
    val = std::max(m_ctrlBrightness.minimum, std::min(m_ctrlBrightness.maximum, val));
    bool ok = SetControl(V4L2_CID_BRIGHTNESS, val);
    Debug.AddLine(wxString::Format("V4L2: set brightness=%d %s", val, ok ? "ok" : strerror(errno)));
    return ok;
}

bool CameraV4L2::TrySetContrast(int val)
{
    if (!m_ctrlContrast.available)
        return false;
    val = std::max(m_ctrlContrast.minimum, std::min(m_ctrlContrast.maximum, val));
    bool ok = SetControl(V4L2_CID_CONTRAST, val);
    Debug.AddLine(wxString::Format("V4L2: set contrast=%d %s", val, ok ? "ok" : strerror(errno)));
    return ok;
}

// ---------------------------------------------------------------------------
// Resolution enumeration and switching
// ---------------------------------------------------------------------------

void CameraV4L2::EnumResolutions()
{
    m_resolutions.clear();

    struct v4l2_frmsizeenum fse;
    memset(&fse, 0, sizeof(fse));
    fse.pixel_format = m_pixelFormat;
    fse.index = 0;

    while (xioctl(m_fd, VIDIOC_ENUM_FRAMESIZES, &fse) == 0)
    {
        if (fse.type == V4L2_FRMSIZE_TYPE_DISCRETE)
        {
            V4L2Resolution r = { fse.discrete.width, fse.discrete.height };
            m_resolutions.push_back(r);
        }
        else
        {
            // Stepwise or continuous: add common sizes that fall within range
            const struct { unsigned int w, h; } common[] = {
                { 320, 240 }, { 640, 480 }, { 800, 600 },
                { 1024, 768 }, { 1280, 720 }, { 1280, 960 },
                { 1920, 1080 }, { 1920, 1200 }, { 0, 0 }
            };
            unsigned int minW = fse.stepwise.min_width;
            unsigned int maxW = fse.stepwise.max_width;
            unsigned int minH = fse.stepwise.min_height;
            unsigned int maxH = fse.stepwise.max_height;
            for (int i = 0; common[i].w; i++)
            {
                if (common[i].w >= minW && common[i].w <= maxW &&
                    common[i].h >= minH && common[i].h <= maxH)
                {
                    V4L2Resolution r = { common[i].w, common[i].h };
                    m_resolutions.push_back(r);
                }
            }
            break; // only one entry for stepwise/continuous
        }
        fse.index++;
    }

    // Ensure the current resolution is in the list (driver may have snapped it)
    bool found = false;
    for (const auto& r : m_resolutions)
        if (r.width == m_captureWidth && r.height == m_captureHeight)
            { found = true; break; }
    if (!found)
    {
        V4L2Resolution r = { m_captureWidth, m_captureHeight };
        m_resolutions.insert(m_resolutions.begin(), r);
    }

    Debug.AddLine(wxString::Format("V4L2: %zu resolution(s) enumerated",
        m_resolutions.size()));
}

bool CameraV4L2::SetResolution(unsigned int width, unsigned int height)
{
    Debug.AddLine(wxString::Format("V4L2: switching resolution to %ux%u", width, height));

    StopStreaming();
    UninitMmap();

    // Release kernel-side buffer allocations.  Without this, most drivers
    // reject VIDIOC_S_FMT silently because buffer slots are still allocated.
    {
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count  = 0;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(m_fd, VIDIOC_REQBUFS, &req);
    }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = width;
    fmt.fmt.pix.height      = height;
    fmt.fmt.pix.pixelformat = m_pixelFormat;
    fmt.fmt.pix.field       = V4L2_FIELD_ANY;

    if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0 ||
        fmt.fmt.pix.pixelformat != m_pixelFormat)
    {
        Debug.AddLine(wxString::Format("V4L2: VIDIOC_S_FMT failed: %s", strerror(errno)));
        // Restore previous resolution so the stream can restart
        fmt.fmt.pix.width  = m_captureWidth;
        fmt.fmt.pix.height = m_captureHeight;
        fmt.fmt.pix.pixelformat = m_pixelFormat;
        xioctl(m_fd, VIDIOC_S_FMT, &fmt);
    }

    m_captureWidth  = fmt.fmt.pix.width;
    m_captureHeight = fmt.fmt.pix.height;
    FrameSize = wxSize(m_captureWidth, m_captureHeight);

    Debug.AddLine(wxString::Format("V4L2: resolution now %ux%u",
        m_captureWidth, m_captureHeight));

    int numBufs = (m_config.numBuffers > 0) ? m_config.numBuffers
                                             : V4L2_DEFAULT_NUM_BUFFERS;
    if (!InitMmap(numBufs))
    {
        pFrame->Alert(_("V4L2: failed to reinitialise buffers after resolution change"));
        return false;
    }
    if (!StartStreaming())
    {
        pFrame->Alert(_("V4L2: failed to restart streaming after resolution change"));
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// USB ID lookup from sysfs
// ---------------------------------------------------------------------------

bool CameraV4L2::GetUsbIds(const wxString& devicePath,
                            unsigned int *vendorId, unsigned int *productId)
{
    wxString nodeName = wxFileName(devicePath).GetFullName();
    wxString sysBase  = "/sys/class/video4linux/" + nodeName + "/device";

    for (int depth = 0; depth < 6; depth++)
    {
        wxString vendorFile  = sysBase + "/idVendor";
        wxString productFile = sysBase + "/idProduct";

        if (wxFileExists(vendorFile) && wxFileExists(productFile))
        {
            wxString vStr, pStr;
            if (wxFile(vendorFile).ReadAll(&vStr) &&
                wxFile(productFile).ReadAll(&pStr))
            {
                unsigned long v = 0, p = 0;
                if (vStr.Trim().ToULong(&v, 16) && pStr.Trim().ToULong(&p, 16))
                {
                    *vendorId  = (unsigned int) v;
                    *productId = (unsigned int) p;
                    return true;
                }
            }
            break;
        }
        sysBase += "/..";
    }
    return false;
}

// ---------------------------------------------------------------------------
// uvc_cameras.xml  –  sensor name / pixel size database
// ---------------------------------------------------------------------------

bool CameraV4L2::LoadUvcDatabase(std::vector<UVCCameraInfo>& db)
{
    wxString xmlPath = FindDataFile("uvc_cameras.xml");
    if (xmlPath.empty())
    {
        Debug.AddLine("V4L2: uvc_cameras.xml not found");
        return false;
    }

    wxTextFile f;
    if (!f.Open(xmlPath))
    {
        Debug.AddLine(wxString::Format("V4L2: cannot open %s", xmlPath));
        return false;
    }

    UVCCameraInfo cur;
    bool inCamera = false;

    for (wxString line = f.GetFirstLine(); !f.Eof(); line = f.GetNextLine())
    {
        wxString t = line.Trim(true).Trim(false);

        if (t.Contains("<camera>"))
        {
            cur = UVCCameraInfo();
            inCamera = true;
            continue;
        }
        if (t.Contains("</camera>"))
        {
            if (inCamera && cur.vendorId && cur.productId)
                db.push_back(cur);
            inCamera = false;
            continue;
        }
        if (!inCamera)
            continue;

        wxString val;
        if (XmlLineContent(t, "usb_vendor_id", &val))
        {
            unsigned long v = 0; val.ToULong(&v, 0);
            cur.vendorId = (unsigned int) v;
        }
        else if (XmlLineContent(t, "usb_product_id", &val))
        {
            unsigned long v = 0; val.ToULong(&v, 0);
            cur.productId = (unsigned int) v;
        }
        else if (XmlLineContent(t, "name",          &val)) cur.name        = val;
        else if (XmlLineContent(t, "sensor",         &val)) cur.sensor      = val;
        else if (XmlLineContent(t, "pixel_size_um",  &val))
        {
            double d = 0.0; val.ToDouble(&d);
            cur.pixelSizeUm = d;
        }
    }

    Debug.AddLine(wxString::Format("V4L2: loaded %zu entries from uvc_cameras.xml",
        db.size()));
    return !db.empty();
}

bool CameraV4L2::LookupUvcCamera(unsigned int vendorId, unsigned int productId,
                                  const std::vector<UVCCameraInfo>& db,
                                  UVCCameraInfo *result)
{
    for (const auto& entry : db)
    {
        if (entry.vendorId == vendorId && entry.productId == productId)
        {
            *result = entry;
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// uvc_config.ini  –  preset configuration loader
//
// Format:
//   # comment
//   [global]          <- applies to all devices
//   [VID:PID]         <- overrides for a specific device (hex, e.g. 046d:0825)
//
// Recognised keys (any can be set to "ignore" to leave that option untouched):
//   preferred_formats = YUYV,GREY,Y16,MJPEG
//   width             = 640
//   height            = 480
//   num_buffers       = 4
//   exposure_mode     = auto | manual | ignore
//   gain              = 0-100 | ignore
// ---------------------------------------------------------------------------

// Apply a single key=value pair to cfg.  Returns false if the key is unknown.
static bool ApplyConfigKey(const wxString& key, const wxString& val,
                           UVCDeviceConfig *cfg)
{
    wxString vl = val.Lower();

    if (key == "preferred_formats")
    {
        if (vl == "ignore")
            return true;
        unsigned int fmts[8];
        int count = 0;
        wxStringTokenizer tok(val, ",");
        while (tok.HasMoreTokens() && count < 7)
        {
            unsigned int fc = FourccFromName(tok.GetNextToken());
            if (fc)
                fmts[count++] = fc;
        }
        if (count > 0)
        {
            fmts[count] = 0;
            memcpy(cfg->preferredFormats, fmts,
                   (count + 1) * sizeof(unsigned int));
        }
        return true;
    }
    if (key == "width")
    {
        if (vl != "ignore") { long v = 0; val.ToLong(&v); cfg->width = (unsigned int) v; }
        return true;
    }
    if (key == "height")
    {
        if (vl != "ignore") { long v = 0; val.ToLong(&v); cfg->height = (unsigned int) v; }
        return true;
    }
    if (key == "num_buffers")
    {
        if (vl == "ignore") cfg->numBuffers = -1;
        else { long v = 0; val.ToLong(&v); if (v >= 2) cfg->numBuffers = (int) v; }
        return true;
    }
    if (key == "exposure_mode")
    {
        if      (vl == "auto")   cfg->exposureMode = UVC_EXPOSURE_AUTO;
        else if (vl == "manual") cfg->exposureMode = UVC_EXPOSURE_MANUAL;
        else                     cfg->exposureMode = UVC_EXPOSURE_DEFAULT;
        return true;
    }
    if (key == "gain")
    {
        if (vl == "ignore") cfg->gain = -1;
        else { long v = 0; val.ToLong(&v); cfg->gain = (int) wxMin(100L, wxMax(0L, v)); }
        return true;
    }
    return false; // unknown key
}

// Scan the open TextFile for a section whose header exactly matches
// targetSection (case-insensitive) and apply all its key=value pairs to cfg.
static void ApplySection(wxTextFile& f, const wxString& targetSection,
                         UVCDeviceConfig *cfg)
{
    if (targetSection.empty())
        return;

    bool inSection = false;

    for (wxString line = f.GetFirstLine(); !f.Eof(); line = f.GetNextLine())
    {
        wxString t = line.Trim(true).Trim(false);
        if (t.empty() || t.StartsWith("#") || t.StartsWith(";"))
            continue;

        if (t.StartsWith("[") && t.EndsWith("]"))
        {
            wxString sec = t.Mid(1, t.length() - 2).Trim(true).Trim(false).Lower();
            inSection = (sec == targetSection);
            continue;
        }

        if (!inSection)
            continue;

        int eq = t.Find('=');
        if (eq == wxNOT_FOUND)
            continue;

        wxString key = t.Left(eq).Trim(true).Trim(false).Lower();
        wxString val = t.Mid(eq + 1).Trim(true).Trim(false);
        ApplyConfigKey(key, val, cfg);
    }
}

bool CameraV4L2::LoadUvcConfig(unsigned int vendorId, unsigned int productId,
                                const wxString& cameraName,
                                const wxString& sensorName,
                                UVCDeviceConfig *cfg)
{
    *cfg = UVCDeviceConfig(); // start from compiled-in defaults

    wxString iniPath = FindDataFile("uvc_config.ini");
    if (iniPath.empty())
    {
        Debug.AddLine("V4L2: uvc_config.ini not found; using compiled-in defaults");
        return false;
    }

    wxTextFile f;
    if (!f.Open(iniPath))
    {
        Debug.AddLine(wxString::Format("V4L2: cannot open %s", iniPath));
        return false;
    }

    // Four passes in increasing specificity – later passes override earlier ones.
    //
    //  Pass 0  [global]
    //  Pass 1  [sensor:SensorName]   e.g. [sensor:OmniVision OV9715]
    //  Pass 2  [name:CameraName]     e.g. [name:Logitech C270]
    //  Pass 3  [VID:PID]             e.g. [046d:0825]
    //
    // Any pass whose target section is absent in the file is a no-op.

    const wxString passes[4] = {
        "global",
        sensorName.empty() ? wxString() : ("sensor:" + sensorName.Lower()),
        cameraName.empty() ? wxString() : ("name:"   + cameraName.Lower()),
        wxString::Format("%04x:%04x", vendorId, productId),
    };

    for (int i = 0; i < 4; i++)
        ApplySection(f, passes[i], cfg);

    Debug.AddLine(wxString::Format(
        "V4L2: config resolved for \"%s\" [%s] %04x:%04x – "
        "fmt[0]=%4.4s w=%u h=%u bufs=%d exp=%d gain=%d",
        cameraName, sensorName, vendorId, productId,
        cfg->preferredFormats[0] ? (const char *)&cfg->preferredFormats[0] : "----",
        cfg->width, cfg->height, cfg->numBuffers,
        (int)cfg->exposureMode, cfg->gain));

    return true;
}

// ---------------------------------------------------------------------------
// Property dialog – shown via Camera > Properties when connected
// ---------------------------------------------------------------------------

// Helper: create a spin-ctrl row and append it to a FlexGridSizer.
// Returns the wxSpinCtrl* (or nullptr if info.available is false).
static wxSpinCtrl *AddSpinRow(wxWindow *parent, wxFlexGridSizer *grid,
                              const wxString& label,
                              const V4L2CtrlInfo& info, int currentVal)
{
    if (!info.available)
        return nullptr;

    grid->Add(new wxStaticText(parent, wxID_ANY, label),
              wxSizerFlags().Align(wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT).Border(wxALL, 4));

    wxSpinCtrl *sc = new wxSpinCtrl(parent, wxID_ANY,
                                    wxEmptyString,
                                    wxDefaultPosition, wxDefaultSize,
                                    wxSP_ARROW_KEYS,
                                    info.minimum, info.maximum, currentVal);
    sc->SetToolTip(wxString::Format(_("Range: %d \u2013 %d  (default %d)"),
                                   info.minimum, info.maximum, info.def));
    grid->Add(sc, wxSizerFlags().Expand().Border(wxALL, 4));

    return sc;
}

class V4L2PropDlg : public wxDialog
{
    CameraV4L2 *m_cam;

    void OnReset(wxCommandEvent&);
    void OnApply(wxCommandEvent&);

public:
    // nullptr if the control is unavailable on this device
    wxChoice   *m_resolution;
    wxChoice   *m_expMode;
    wxSpinCtrl *m_brightness;
    wxSpinCtrl *m_contrast;
    wxSpinCtrl *m_gamma;

    V4L2PropDlg(wxWindow *parent, CameraV4L2 *cam);
    void ApplySettings();
};

// Render a human-readable description of V4L2_CAP_* flags.
static wxString FormatCapFlags(unsigned int f)
{
    struct { unsigned int bit; const char *name; } table[] = {
        { V4L2_CAP_VIDEO_CAPTURE,        "VIDEO_CAPTURE" },
        { V4L2_CAP_VIDEO_OUTPUT,         "VIDEO_OUTPUT" },
        { V4L2_CAP_VIDEO_OVERLAY,        "VIDEO_OVERLAY" },
        { V4L2_CAP_STREAMING,            "STREAMING" },
        { V4L2_CAP_READWRITE,            "READWRITE" },
        { V4L2_CAP_EXT_PIX_FORMAT,       "EXT_PIX_FORMAT" },
        { V4L2_CAP_VIDEO_CAPTURE_MPLANE, "VIDEO_CAPTURE_MPLANE" },
        { V4L2_CAP_DEVICE_CAPS,          "DEVICE_CAPS" },
        { V4L2_CAP_TUNER,                "TUNER" },
        { V4L2_CAP_AUDIO,                "AUDIO" },
        { V4L2_CAP_RDS_CAPTURE,          "RDS_CAPTURE" },
        { V4L2_CAP_TIMEPERFRAME,         "TIMEPERFRAME" },
    };
    wxString out;
    for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); i++)
        if (f & table[i].bit)
        {
            if (!out.empty()) out += " | ";
            out += table[i].name;
        }
    return out.empty() ? wxString("-") : out;
}

// Render fourcc as ASCII, e.g. 0x56595559 -> "YUYV"
static wxString FourccToString(unsigned int f)
{
    char s[5] = {
        (char) ((f >>  0) & 0xff),
        (char) ((f >>  8) & 0xff),
        (char) ((f >> 16) & 0xff),
        (char) ((f >> 24) & 0xff),
        0,
    };
    for (int i = 0; i < 4; i++)
        if (s[i] < 0x20 || s[i] >= 0x7f) s[i] = '?';
    return wxString::FromAscii(s);
}

V4L2PropDlg::V4L2PropDlg(wxWindow *parent, CameraV4L2 *cam)
    : wxDialog(parent, wxID_ANY, _("V4L2 Camera Properties")),
      m_cam(cam),
      m_resolution(nullptr),
      m_expMode(nullptr),
      m_brightness(nullptr),
      m_contrast(nullptr),
      m_gamma(nullptr)
{
    wxBoxSizer *top = new wxBoxSizer(wxVERTICAL);

    // -----------------------------------------------------------------
    // Camera information (read-only)
    // -----------------------------------------------------------------
    {
        wxStaticBoxSizer *infoBox =
            new wxStaticBoxSizer(wxVERTICAL, this, _("Camera information"));
        wxFlexGridSizer *ig = new wxFlexGridSizer(2, 4, 8);
        ig->AddGrowableCol(1, 1);

        auto addInfoRow = [&](const wxString& label, const wxString& value)
        {
            wxStaticText *l = new wxStaticText(infoBox->GetStaticBox(),
                                               wxID_ANY, label);
            wxStaticText *v = new wxStaticText(infoBox->GetStaticBox(),
                                               wxID_ANY,
                                               value.empty() ? wxString("-") : value);
            wxFont vf = v->GetFont();
            vf.SetFamily(wxFONTFAMILY_TELETYPE);
            v->SetFont(vf);
            ig->Add(l, 0, wxALIGN_CENTER_VERTICAL);
            ig->Add(v, 0, wxEXPAND);
        };

        addInfoRow(_("Device path:"),  cam->m_devicePath);
        addInfoRow(_("Driver:"),
                   wxString::Format("%s (kernel %u.%u.%u)",
                       cam->m_capDriver,
                       (cam->m_capVersion >> 16) & 0xff,
                       (cam->m_capVersion >>  8) & 0xff,
                        cam->m_capVersion        & 0xff));
        addInfoRow(_("Card:"),         cam->m_capCard);
        addInfoRow(_("Bus:"),          cam->m_capBusInfo);

        if (cam->m_usbVendorId || cam->m_usbProductId)
        {
            addInfoRow(_("USB ID:"),
                       wxString::Format("%04x:%04x",
                           cam->m_usbVendorId, cam->m_usbProductId));
        }
        addInfoRow(_("USB serial:"),   cam->m_usbSerial);
        addInfoRow(_("Hardware id:"),  cam->m_hardwareId);

        if (!cam->m_dbCameraName.empty())
            addInfoRow(_("Database name:"), cam->m_dbCameraName);
        if (!cam->m_dbSensorName.empty())
            addInfoRow(_("Sensor:"),        cam->m_dbSensorName);
        if (cam->m_dbPixelSizeUm > 0.0)
            addInfoRow(_("Pixel size:"),
                       wxString::Format("%.2f um", cam->m_dbPixelSizeUm));

        addInfoRow(_("Capabilities:"), FormatCapFlags(cam->m_capFlags));

        // Active format / size.
        addInfoRow(_("Active format:"),
                   wxString::Format("%s  %u x %u",
                       FourccToString(cam->m_pixelFormat),
                       cam->m_captureWidth, cam->m_captureHeight));

        // Supported pixel formats.
        wxString fmts;
        for (size_t i = 0; i < cam->m_supportedFormats.size(); i++)
        {
            if (!fmts.empty()) fmts += ", ";
            fmts += FourccToString(cam->m_supportedFormats[i]);
        }
        addInfoRow(_("Supported formats:"), fmts);

        // Available controls and their ranges.
        auto ctrlRange = [](const V4L2CtrlInfo& c) -> wxString
        {
            if (!c.available) return _("not supported");
            return wxString::Format("range %d..%d (step %d, default %d)",
                                    c.minimum, c.maximum, c.step, c.def);
        };
        addInfoRow(_("Exposure auto:"), ctrlRange(cam->m_ctrlExposureAuto));
        addInfoRow(_("Exposure abs:"),  ctrlRange(cam->m_ctrlExposureAbs));
        addInfoRow(_("Gain:"),          ctrlRange(cam->m_ctrlGain));
        addInfoRow(_("Brightness:"),    ctrlRange(cam->m_ctrlBrightness));
        addInfoRow(_("Contrast:"),      ctrlRange(cam->m_ctrlContrast));
        addInfoRow(_("Gamma:"),         ctrlRange(cam->m_ctrlGamma));

        infoBox->Add(ig, wxSizerFlags(1).Expand().Border(wxALL, 6));
        top->Add(infoBox, wxSizerFlags().Expand().Border(wxALL, 8));
    }

    wxStaticBoxSizer *adjBox =
        new wxStaticBoxSizer(wxVERTICAL, this, _("Adjustable settings"));

    wxFlexGridSizer *grid = new wxFlexGridSizer(2, 0, 0);
    grid->AddGrowableCol(1, 1);

    // Resolution
    if (!cam->m_resolutions.empty())
    {
        grid->Add(new wxStaticText(this, wxID_ANY, _("Resolution:")),
                  wxSizerFlags().Align(wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT).Border(wxALL, 4));

        wxArrayString resChoices;
        int curSel = 0;
        for (size_t i = 0; i < cam->m_resolutions.size(); i++)
        {
            const V4L2Resolution& r = cam->m_resolutions[i];
            resChoices.Add(wxString::Format("%ux%u", r.width, r.height));
            if (r.width == cam->m_captureWidth && r.height == cam->m_captureHeight)
                curSel = (int) i;
        }
        m_resolution = new wxChoice(this, wxID_ANY, wxDefaultPosition,
                                    wxDefaultSize, resChoices);
        m_resolution->SetSelection(curSel);
        grid->Add(m_resolution, wxSizerFlags().Expand().Border(wxALL, 4));
    }

    // Exposure mode
    if (cam->m_ctrlExposureAuto.available)
    {
        int saved = pConfig->Profile.GetInt(wxString(V4L2_CFG_PFX) + "/exposure_mode", -1);
        int curHw = cam->GetControl(V4L2_CID_EXPOSURE_AUTO);
        bool isAuto = (saved == 2) ||
                      (saved < 0 && curHw != INT_MIN &&
                       (curHw == V4L2_EXPOSURE_AUTO ||
                        curHw == V4L2_EXPOSURE_APERTURE_PRIORITY));

        grid->Add(new wxStaticText(this, wxID_ANY, _("Exposure Mode:")),
                  wxSizerFlags().Align(wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT).Border(wxALL, 4));

        wxArrayString choices;
        choices.Add(_("Manual (PHD2 controls duration)"));
        choices.Add(_("Auto (camera AE)"));
        m_expMode = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, choices);
        m_expMode->SetSelection(isAuto ? 1 : 0);
        m_expMode->SetToolTip(_("Manual: PHD2 sets exposure time each frame.\n"
                                "Auto: camera auto-exposure (PHD2 duration ignored)."));
        grid->Add(m_expMode, wxSizerFlags().Expand().Border(wxALL, 4));
    }

    // Brightness, Contrast, Gamma sliders
    int curBrightness = cam->m_ctrlBrightness.available
                      ? cam->GetControl(V4L2_CID_BRIGHTNESS)
                      : cam->m_ctrlBrightness.def;
    if (curBrightness == INT_MIN) curBrightness = cam->m_ctrlBrightness.def;

    int curContrast   = cam->m_ctrlContrast.available
                      ? cam->GetControl(V4L2_CID_CONTRAST)
                      : cam->m_ctrlContrast.def;
    if (curContrast == INT_MIN) curContrast = cam->m_ctrlContrast.def;

    int curGamma      = cam->m_ctrlGamma.available
                      ? cam->GetControl(V4L2_CID_GAMMA)
                      : cam->m_ctrlGamma.def;
    if (curGamma == INT_MIN) curGamma = cam->m_ctrlGamma.def;

    m_brightness = AddSpinRow(this, grid, _("Brightness:"),
                              cam->m_ctrlBrightness, curBrightness);
    m_contrast   = AddSpinRow(this, grid, _("Contrast:"),
                              cam->m_ctrlContrast,   curContrast);
    m_gamma      = AddSpinRow(this, grid, _("Gamma:"),
                              cam->m_ctrlGamma,      curGamma);

    adjBox->Add(grid, wxSizerFlags(1).Expand().Border(wxALL, 4));

    if (cam->m_resolutions.empty()         &&
        !cam->m_ctrlExposureAuto.available &&
        !cam->m_ctrlBrightness.available   &&
        !cam->m_ctrlContrast.available     &&
        !cam->m_ctrlGamma.available)
    {
        adjBox->Add(new wxStaticText(adjBox->GetStaticBox(), wxID_ANY,
                    _("No adjustable controls found on this device.")),
                    wxSizerFlags().Border(wxALL, 6));
    }

    top->Add(adjBox, wxSizerFlags().Expand().Border(wxLEFT | wxRIGHT | wxBOTTOM, 8));

    wxBoxSizer *btnRow = new wxBoxSizer(wxHORIZONTAL);
    wxButton *resetBtn = new wxButton(this, wxID_ANY, _("Reset to Defaults"));
    btnRow->Add(resetBtn, wxSizerFlags().Border(wxRIGHT, 8));
    btnRow->AddStretchSpacer();
    wxStdDialogButtonSizer *bs = new wxStdDialogButtonSizer();
    bs->AddButton(new wxButton(this, wxID_OK));
    bs->AddButton(new wxButton(this, wxID_APPLY));
    bs->AddButton(new wxButton(this, wxID_CANCEL));
    bs->Realize();
    btnRow->Add(bs);
    top->Add(btnRow, wxSizerFlags().Expand().Border(wxALL, 8));

    resetBtn->Bind(wxEVT_BUTTON, &V4L2PropDlg::OnReset, this);
    Bind(wxEVT_BUTTON, &V4L2PropDlg::OnApply, this, wxID_APPLY);

    SetSizerAndFit(top);
    Centre(wxBOTH);
}

void V4L2PropDlg::ApplySettings()
{
    Debug.AddLine("V4L2: applying property dialog settings");

    // Resolution change
    if (m_resolution)
    {
        int sel = m_resolution->GetSelection();
        if (sel >= 0 && sel < (int) m_cam->m_resolutions.size())
        {
            const V4L2Resolution& r = m_cam->m_resolutions[sel];
            if (r.width != m_cam->m_captureWidth || r.height != m_cam->m_captureHeight)
            {
                if (m_cam->SetResolution(r.width, r.height))
                {
                    pConfig->Profile.SetInt(wxString(V4L2_CFG_PFX) + "/width",  r.width);
                    pConfig->Profile.SetInt(wxString(V4L2_CFG_PFX) + "/height", r.height);
                }
            }
        }
    }

    if (m_cam->m_ctrlExposureAuto.available && m_expMode)
    {
        bool autoMode = (m_expMode->GetSelection() == 1);
        bool ok = m_cam->SetControl(V4L2_CID_EXPOSURE_AUTO,
                                    autoMode ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL);
        Debug.AddLine(wxString::Format("V4L2: set exposure_mode=%s %s",
            autoMode ? "auto" : "manual", ok ? "ok" : strerror(errno)));
        pConfig->Profile.SetInt(wxString(V4L2_CFG_PFX) + "/exposure_mode",
                                autoMode ? 2 : 1);
    }
    if (m_cam->m_ctrlBrightness.available && m_brightness)
    {
        int v = m_brightness->GetValue();
        m_cam->TrySetBrightness(v);
        pConfig->Profile.SetInt(wxString(V4L2_CFG_PFX) + "/brightness", v);
    }
    if (m_cam->m_ctrlContrast.available && m_contrast)
    {
        int v = m_contrast->GetValue();
        m_cam->TrySetContrast(v);
        pConfig->Profile.SetInt(wxString(V4L2_CFG_PFX) + "/contrast", v);
    }
    if (m_cam->m_ctrlGamma.available && m_gamma)
    {
        int v = m_gamma->GetValue();
        m_cam->TrySetGamma(v);
        pConfig->Profile.SetInt(wxString(V4L2_CFG_PFX) + "/gamma", v);
    }
}

void V4L2PropDlg::OnApply(wxCommandEvent&)
{
    ApplySettings();
}

void V4L2PropDlg::OnReset(wxCommandEvent&)
{
    // Reset resolution to current (don't auto-change on reset)
    if (m_resolution)
    {
        for (size_t i = 0; i < m_cam->m_resolutions.size(); i++)
        {
            const V4L2Resolution& r = m_cam->m_resolutions[i];
            if (r.width == m_cam->m_captureWidth && r.height == m_cam->m_captureHeight)
            {
                m_resolution->SetSelection((int) i);
                break;
            }
        }
    }
    if (m_expMode)    m_expMode->SetSelection(0);
    if (m_brightness) m_brightness->SetValue(m_cam->m_ctrlBrightness.def);
    if (m_contrast)   m_contrast->SetValue(m_cam->m_ctrlContrast.def);
    if (m_gamma)      m_gamma->SetValue(m_cam->m_ctrlGamma.def);
}

void CameraV4L2::ShowPropertyDialog()
{
    V4L2PropDlg dlg(wxGetApp().GetTopWindow(), this);

    if (dlg.ShowModal() == wxID_OK)
        dlg.ApplySettings();
}

#endif // V4L2_CAMERA
