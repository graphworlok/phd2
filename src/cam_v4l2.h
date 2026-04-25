/*
 *  cam_v4l2.h
 *  PHD Guiding
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

#ifndef CAM_V4L2_H_INCLUDED
#define CAM_V4L2_H_INCLUDED

#if defined(V4L2_CAMERA)

# include <vector>
# include <string>

struct V4L2Buffer
{
    void *start;
    size_t length;
};

struct V4L2Resolution
{
    unsigned int width;
    unsigned int height;
};

// Entry loaded from uvc_cameras.xml
struct UVCCameraInfo
{
    unsigned int vendorId;
    unsigned int productId;
    wxString name;
    wxString sensor;
    double pixelSizeUm; // 0.0 = unknown
};

// Exposure mode preset values
enum UVCExposureMode
{
    UVC_EXPOSURE_DEFAULT = 0, // do not touch the control
    UVC_EXPOSURE_AUTO    = 1, // set V4L2_EXPOSURE_AUTO
    UVC_EXPOSURE_MANUAL  = 2, // set V4L2_EXPOSURE_MANUAL
};

// Range and availability information for a single V4L2 control.
// Populated by QueryCtrl() at Connect() time.
struct V4L2CtrlInfo
{
    bool available;
    int  minimum;
    int  maximum;
    int  step;
    int  def; // default_value from the driver

    V4L2CtrlInfo() : available(false), minimum(0), maximum(0), step(1), def(0) {}
};

// Configuration loaded from uvc_config.ini.
// A [global] section sets defaults; a [VID:PID] section (e.g. [046d:0825])
// overrides individual fields for a specific device.
struct UVCDeviceConfig
{
    // Pixel formats to try, in order (V4L2 fourcc codes, 0-terminated).
    // Populated from "preferred_formats = YUYV,GREY,Y16,MJPEG"
    unsigned int preferredFormats[8];

    unsigned int width;        // 0 = let the driver choose
    unsigned int height;       // 0 = let the driver choose
    int          numBuffers;   // number of mmap capture buffers; -1 = use default
    UVCExposureMode exposureMode;
    int          gain;         // 0-100 PHD2 scale; -1 = do not preset

    UVCDeviceConfig();         // fills in compiled-in defaults
};

class V4L2PropDlg; // forward declaration for friend access

class CameraV4L2 : public GuideCamera
{
    friend class V4L2PropDlg;

    int m_fd;
    std::vector<V4L2Buffer> m_buffers;
    wxString m_devicePath;
    unsigned int m_pixelFormat;
    unsigned int m_captureWidth;
    unsigned int m_captureHeight;
    UVCDeviceConfig m_config; // resolved at Connect() time

    std::vector<V4L2Resolution> m_resolutions; // enumerated at Connect time

    // Capability info captured at Connect() time so the property dialog
    // can present a read-only "Camera information" section without
    // re-issuing ioctls.
    wxString     m_capDriver;       // cap.driver         (kernel driver)
    wxString     m_capCard;         // cap.card           (device label)
    wxString     m_capBusInfo;      // cap.bus_info       (e.g. usb-0000:00:14.0-3)
    unsigned int m_capVersion;      // cap.version
    unsigned int m_capFlags;        // cap.device_caps    (V4L2_CAP_*)
    unsigned int m_usbVendorId;     // 0 if unknown
    unsigned int m_usbProductId;    // 0 if unknown
    wxString     m_usbSerial;       // empty if unknown
    wxString     m_dbCameraName;    // from uvc_cameras.xml, empty if not in db
    wxString     m_dbSensorName;    // from uvc_cameras.xml
    double       m_dbPixelSizeUm;   // 0 if unknown
    std::vector<unsigned int> m_supportedFormats; // fourcc codes

    // Per-control availability and range (queried at Connect time)
    V4L2CtrlInfo m_ctrlExposureAuto;
    V4L2CtrlInfo m_ctrlExposureAbs;
    V4L2CtrlInfo m_ctrlGain;
    V4L2CtrlInfo m_ctrlGamma;
    V4L2CtrlInfo m_ctrlBrightness;
    V4L2CtrlInfo m_ctrlContrast;

    // -- device helpers --
    bool OpenDevice(const wxString& path);
    void CloseDevice();
    bool InitMmap(int numBuffers);
    void UninitMmap();
    bool StartStreaming();
    void StopStreaming();
    bool GrabFrame(unsigned short *dst, unsigned int npixels);
    void ConvertToGray16(const void *src, unsigned int srcSize,
                         unsigned short *dst, unsigned int npixels) const;
    bool TrySetExposure(int durationMs);
    bool TrySetGain(int gain);
    bool TrySetGamma(int val);
    bool TrySetBrightness(int val);
    bool TrySetContrast(int val);

    // Read / write a single V4L2 control by CID.
    // GetControl returns INT_MIN on failure.
    int  GetControl(unsigned int cid);
    bool SetControl(unsigned int cid, int val);

    // Enumerate discrete frame sizes for the current pixel format.
    void EnumResolutions();

    // Stop streaming, change format, restart. Returns false on failure.
    // On failure the stream is left stopped; caller should Disconnect().
    bool SetResolution(unsigned int width, unsigned int height);

    // -- USB / database helpers --
    static bool GetUsbIds(const wxString& devicePath,
                          unsigned int *vendorId, unsigned int *productId);
    static bool LoadUvcDatabase(std::vector<UVCCameraInfo>& db);
    static bool LookupUvcCamera(unsigned int vendorId, unsigned int productId,
                                const std::vector<UVCCameraInfo>& db,
                                UVCCameraInfo *result);

    // -- config helpers --
    // cameraName / sensorName come from uvc_cameras.xml; pass empty strings
    // if the device is not in the database.
    static bool LoadUvcConfig(unsigned int vendorId, unsigned int productId,
                              const wxString& cameraName,
                              const wxString& sensorName,
                              UVCDeviceConfig *cfg);

public:
    CameraV4L2();
    ~CameraV4L2();

    bool HasNonGuiCapture() override { return true; }
    wxByte BitsPerPixel() override;
    bool Connect(const wxString& camId) override;
    bool Disconnect() override;
    bool Capture(usImage& img, const CaptureParams& captureParams) override;
    bool CanSelectCamera() const override { return true; }
    bool EnumCameras(wxArrayString& names, wxArrayString& ids) override;
    bool GetDevicePixelSize(double *devPixelSize) override;
    void ShowPropertyDialog() override;
    wxString GetHardwareId() const override;

private:
    // Cached at Connect() time so GetHardwareId() is callable on any
    // thread without re-walking sysfs.
    wxString m_hardwareId;

    // Read iSerial / serial / iManufacturer-Product from sysfs walking
    // up from the video node. Returns empty if no readable serial node.
    static wxString ReadUsbSerial(const wxString& devicePath);
};

#endif // V4L2_CAMERA

#endif // CAM_V4L2_H_INCLUDED
