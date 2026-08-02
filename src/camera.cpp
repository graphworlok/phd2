/*
 *  camera.cpp
 *  PHD Guiding
 *
 *  Created by Craig Stark.
 *  Copyright (c) 2006-2010 Craig Stark.
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
 *
 */

// General camera routines not specific to any one cam

#include "phd.h"

#include "camera.h"
#include "gear_simulator.h"

#include <wx/regex.h>
#include <wx/stdpaths.h>
#ifdef __linux__
# include <stdlib.h>  // realpath()
# include <limits.h>  // PATH_MAX
#endif

static const int DefaultGuideCameraGain = 95;
static const int DefaultGuideCameraTimeoutMs = 15000;
static const bool DefaultUseSubframes = false;

const double GuideCamera::UnknownPixelSize = 0.0;

wxSize UNDEFINED_FRAME_SIZE = wxSize(0, 0);

#if defined(ATIK16)
# include "cam_atik16.h"
#endif

#if defined(IOPTRON_CAMERA)
# include "cam_ioptron.h"
#endif

#if defined(LE_SERIAL_CAMERA)
# include "cam_LESerialWebcam.h"
#endif

#if defined(LE_PARALLEL_CAMERA)
# include "cam_LEParallelwebcam.h"
#endif

#if defined(LE_LXUSB_CAMERA)
# include "cam_LELXUSBwebcam.h"
#endif

#if defined(QGUIDE)
# include "cam_qguide.h"
#endif

#if defined(CAM_QHY5)
# include "cam_qhy5.h"
#endif

#if defined(PLAYERONE_CAMERA)
# include "cam_playerone.h"
#endif

#if defined(QHY_CAMERA)
# include "cam_qhy.h"
#endif

#if defined(SVB_CAMERA)
# include "cam_svb.h"
#endif

#if defined(ZWO_ASI)
# include "cam_zwo.h"
#endif

#if defined(TOUPTEK_CAMERA)
# include "cam_touptek.h"
#endif

#if defined(SKYRAIDER_CAMERA)
# include "cam_skyraider.h"
#endif

#if defined(ALTAIR)
# include "cam_altair.h"
#endif

#if defined(ORION_DSCI)
# include "cam_StarShootDSCI.h"
#endif

#if defined(OS_PL130)
# include "cam_OSPL130.h"
#endif

#if defined(VFW_CAMERA)
# include "cam_vfw.h"
#endif

#if defined(OGMA_CAMERA)
# include "cam_ogma.h"
#endif

#if defined(OPENCV_CAMERA)
# include "cam_opencv.h"
#endif

#if defined(WDM_CAMERA)
# include "cam_wdm.h"
#endif

#if defined(STARFISH_CAMERA)
# include "cam_starfish.h"
#endif

#if defined(SXV)
# include "cam_sxv.h"
#endif

#if defined(SBIG)
# include "cam_sbig.h"
#endif

#if defined(NEB_SBIG)
# include "cam_NebSBIG.h"
#endif

#if defined(FIREWIRE_CAMERA)
# include "cam_firewire.h"
#endif

#if defined(MEADE_DSI_CAMERA)
# include "cam_MeadeDSI.h"
#endif

#if defined(MORAVIAN_CAMERA)
# include "cam_moravian.h"
#endif

#if defined(SSAG)
# include "cam_ssag.h"
#endif

#if defined(OPENSSAG_CAMERA)
# include "cam_openssag.h"
#endif

#if defined(KWIQGUIDER_CAMERA)
# include "cam_KWIQGuider.h"
#endif

#if defined(SSPIAG)
# include "cam_sspiag.h"
#endif

#if defined(INOVA_PLC)
# include "cam_INovaPLC.h"
#endif

#if defined(ASCOM_CAMERA)
# include "cam_ascom.h"
#endif

#if defined(INDI_CAMERA)
# include "cam_indi.h"
#endif

#if defined(ALPACA_CAMERA)
# include "cam_alpaca.h"
#endif

#if defined(SBIGROTATOR_CAMERA)
# include "cam_sbigrotator.h"
#endif

#if defined(V4L_CAMERA)
# include "cam_VIDEODEVICE.h"
extern "C"
{
# include <libudev.h>
}
#endif

const wxString GuideCamera::DEFAULT_CAMERA_ID = wxEmptyString;

double GuideCamera::GetProfilePixelSize()
{
    return pConfig->Profile.GetDouble("/camera/pixelsize", UnknownPixelSize);
}

GuideCamera::GuideCamera()
{
    Connected = false;
    m_hasGuideOutput = false;
    PropertyDialogType = PROPDLG_NONE;
    HasGainControl = false;
    HasShutter = false;
    ShutterClosed = false;
    HasSubframes = false;
    HasFrameLimiting = false;
    HasCooler = false;
    HasBayer = false;
    FrameSize = UNDEFINED_FRAME_SIZE;
    UseSubframes = pConfig->Profile.GetBoolean("/camera/UseSubframes", DefaultUseSubframes);
    GuideCameraGain = pConfig->Profile.GetInt("/camera/gain", DefaultGuideCameraGain);
    m_timeoutMs = pConfig->Profile.GetInt("/camera/TimeoutMs", DefaultGuideCameraTimeoutMs);
    m_saturationADU = (unsigned short) wxMin(pConfig->Profile.GetInt("/camera/SaturationADU", 0), 65535);
    m_saturationByADU = pConfig->Profile.GetBoolean("/camera/SaturationByADU", true);
    m_pixelSize = GetProfilePixelSize();
    MaxHwBinning = 1;
    HwBinning = pConfig->Profile.GetInt("/camera/binning", 1);
    SwBinning = wxClip(pConfig->Profile.GetInt("/camera/SoftwareBinning", 1), 1, (int) GuideCamera::MAX_SOFTWARE_BINNING);
    CurrentDarkFrame = nullptr;
    CurrentDefectMap = nullptr;
}

GuideCamera::~GuideCamera()
{
    ClearDarks();
    ClearDefectMap();
}

static int CompareNoCase(const wxString& first, const wxString& second)
{
    return first.CmpNoCase(second);
}

static wxString INDICamName()
{
    wxString indicam = pConfig->Profile.GetString("/indi/INDIcam", wxEmptyString);
    return indicam.empty() ? _T("INDI Camera") : wxString::Format("INDI Camera [%s]", indicam);
}

wxArrayString GuideCamera::GuideCameraList()
{
    wxArrayString CameraList;

    CameraList.Add(_("None"));
#if defined(ASCOM_CAMERA)
    wxArrayString ascomCameras = ASCOMCameraFactory::EnumAscomCameras();
    for (unsigned int i = 0; i < ascomCameras.Count(); i++)
        CameraList.Add(ascomCameras[i]);
#endif
#if defined(ATIK16)
    CameraList.Add(_T("Atik 16 series, mono"));
    CameraList.Add(_T("Atik 16 series, color"));
#endif
#if defined(ATIK_GEN3)
    CameraList.Add(_T("Atik Gen3, mono"));
    CameraList.Add(_T("Atik Gen3, color"));
#endif
#if defined(QGUIDE)
    CameraList.Add(_T("CCD Labs Q-Guider"));
#endif
#if defined(STARFISH_CAMERA)
    CameraList.Add(_T("Fishcamp Starfish"));
#endif
#if defined(INOVA_PLC)
    CameraList.Add(_T("i-Nova PLC-M"));
#endif
#if defined(IOPTRON_CAMERA)
    CameraList.Add(_T("iOptron iGuider"));
#endif
#if defined(SSAG)
    CameraList.Add(_T("StarShoot Autoguider"));
#endif
#if defined(SSPIAG)
    CameraList.Add(_T("StarShoot Planetary Imager & Autoguider"));
#endif
#if defined(OS_PL130)
    CameraList.Add(_T("Opticstar PL-130M"));
    CameraList.Add(_T("Opticstar PL-130C"));
#endif
#if defined(ORION_DSCI)
    CameraList.Add(_T("Orion StarShoot DSCI"));
#endif
#if defined(OPENSSAG_CAMERA)
    CameraList.Add(_T("Orion StarShoot Autoguider"));
#endif
#if defined(KWIQGUIDER_CAMERA)
    CameraList.Add(_T("KWIQGuider"));
#endif
#if defined(QGUIDE)
    CameraList.Add(_T("MagZero MZ-5"));
#endif
#if defined(MEADE_DSI_CAMERA)
    CameraList.Add(_T("Meade DSI I, II, or III"));
#endif
#if defined(MORAVIAN_CAMERA)
    CameraList.Add(_T("Moravian Camera"));
#endif
#if defined(PLAYERONE_CAMERA)
    CameraList.Add(_T("Player One Camera"));
#endif
#if defined(CAM_QHY5)
    CameraList.Add(_T("QHY 5"));
#endif
#if defined(QHY_CAMERA)
    CameraList.Add(_T("QHY Camera"));
#endif
#if defined(ALTAIR)
    CameraList.Add(_T("Altair Camera"));
    CameraList.Add(_T("Altair Camera (2015/2016)"));
#endif
#if defined(ZWO_ASI)
    CameraList.Add(_T("ZWO ASI Camera"));
#endif
#if defined(TOUPTEK_CAMERA)
    CameraList.Add(_T("ToupTek Camera"));
    CameraList.Add(_T("Omegon Pro Camera"));
#endif
#if defined(SKYRAIDER_CAMERA)
    CameraList.Add(_T("MallinCam SkyRaider"));
#endif
#if defined(SBIG)
    CameraList.Add(_T("SBIG"));
#endif
#if defined(SBIGROTATOR_CAMERA)
    CameraList.Add(_T("SBIG Rotator"));
#endif
#if defined(SVB_CAMERA)
    CameraList.Add(_T("Svbony Camera"));
#endif
#if defined(SXV)
    CameraList.Add(_T("Starlight Xpress SXV"));
#endif
#if defined(FIREWIRE_CAMERA)
    CameraList.Add(_T("The Imaging Source (DCAM Firewire)"));
#endif
#if defined(OGMA_CAMERA)
    CameraList.Add(_T("OGMA Camera"));
#endif
#if defined(OPENCV_CAMERA)
    CameraList.Add(_T("OpenCV webcam 1"));
    CameraList.Add(_T("OpenCV webcam 2"));
#endif
#if defined(WDM_CAMERA)
    CameraList.Add(_T("Windows WDM-style webcam camera"));
#endif
#if defined(VFW_CAMERA)
    CameraList.Add(_T("Windows VFW-style webcam camera (older & SAC8)"));
#endif
#if defined(LE_LXUSB_CAMERA)
    CameraList.Add(_T("Long exposure LXUSB webcam"));
#endif
#if defined(LE_PARALLEL_CAMERA)
    CameraList.Add(_T("Long exposure Parallel webcam"));
#endif
#if defined(LE_SERIAL_CAMERA)
    CameraList.Add(_T("Long exposure Serial webcam"));
#endif
#if defined(INDI_CAMERA)
    CameraList.Add(INDICamName());
#endif
#if defined(ALPACA_CAMERA)
    CameraList.Add(_T("Alpaca Camera (Experimental)"));
#endif
#if defined(V4L_CAMERA)
    if (true == Camera_VIDEODEVICE.ProbeDevices())
    {
        CameraList.Add(_T("V4L(2) Camera"));
    }
#endif
#if defined(SIMULATOR)
    CameraList.Add(_T("Simulator"));
#endif

#if defined(NEB_SBIG)
    CameraList.Add(_T("Guide chip on SBIG cam in Nebulosity"));
#endif

    CameraList.Sort(&CompareNoCase);

    return CameraList;
}

GuideCamera *GuideCamera::Factory(const wxString& choice)
{
    GuideCamera *pReturn = nullptr;

    try
    {
        if (choice.IsEmpty())
        {
            throw ERROR_INFO("CameraFactory called with choice.IsEmpty()");
        }

        Debug.AddLine(wxString::Format("CameraFactory(%s)", choice));

        if (false) // so else ifs can follow
        {
        }

        // Check ASCOM and INDI first since those choices may match match other choices below (like Simulator)
#if defined(ASCOM_CAMERA)
        else if (choice.Contains(_T("ASCOM")))
        {
            pReturn = ASCOMCameraFactory::MakeASCOMCamera(choice);
        }
#endif
#if defined(INDI_CAMERA)
        else if (choice.Contains(_T("INDI")))
        {
            pReturn = INDICameraFactory::MakeINDICamera();
        }
#endif
#if defined(ALPACA_CAMERA)
        else if (choice.Contains(_T("Alpaca")))
        {
            pReturn = AlpacaCameraFactory::MakeAlpacaCamera();
        }
#endif
#if defined(IOPTRON_CAMERA)
        else if (choice == _T("iOptron iGuider"))
            pReturn = IoptronCameraFactory::MakeIoptronCamera();
#endif
        else if (choice == _("None"))
            pReturn = nullptr;
        else if (choice == _T("Simulator"))
            pReturn = GearSimulator::MakeCamSimulator();
#if defined(ATIK16)
        else if (choice.StartsWith("Atik 16 series"))
        {
            bool hsmodel = false;
            bool color = choice.Find(_T("color")) != wxNOT_FOUND;
            pReturn = AtikCameraFactory::MakeAtikCamera(hsmodel, color);
        }
#endif
#if defined(ATIK_GEN3)
        else if (choice.StartsWith(_T("Atik Gen3")))
        {
            bool hsmodel = true;
            bool color = choice.Find(_T("color")) != wxNOT_FOUND;
            pReturn = AtikCameraFactory::MakeAtikCamera(hsmodel, color);
        }
#endif
#if defined(QGUIDE)
        else if (choice.Contains(_T("CCD Labs Q-Guider")))
        {
            pReturn = new CameraQGuider();
            pReturn->Name = _T("Q-Guider");
        }
        else if (choice.Contains(_T("MagZero MZ-5")))
        {
            pReturn = new CameraQGuider();
            pReturn->Name = _T("MagZero MZ-5");
        }
#endif
#if defined(PLAYERONE_CAMERA)
        else if (choice == _T("Player One Camera"))
            pReturn = PlayerOneCameraFactory::MakePlayerOneCamera();
#endif
#if defined(QHY_CAMERA)
        else if (choice == _T("QHY Camera"))
            pReturn = QHYCameraFactory::MakeQHYCamera();
#endif
#if defined(ALTAIR)
        else if (choice == _T("Altair Camera"))
            pReturn = AltairCameraFactory::MakeAltairCamera(ALTAIR_CAM_CURRENT);
        else if (choice == _T("Altair Camera (2015/2016)"))
            pReturn = AltairCameraFactory::MakeAltairCamera(ALTAIR_CAM_LEGACY);
#endif
#if defined(ZWO_ASI)
        else if (choice == _T("ZWO ASI Camera"))
            pReturn = ZWOCameraFactory::MakeZWOCamera();
#endif
#if defined(TOUPTEK_CAMERA)
        else if (choice == _T("ToupTek Camera") || choice == _T("Omegon Pro Camera"))
        {
            pReturn = ToupTekCameraFactory::MakeToupTekCamera();
        }
#endif
#if defined(SKYRAIDER_CAMERA)
        else if (choice == _T("MallinCam SkyRaider"))
            pReturn = SkyraiderCameraFactory::MakeSkyraiderCamera();
#endif
#if defined(CAM_QHY5) // must come after other QHY 5's since this pattern would match them
        else if (choice.Contains(_T("QHY 5")))
            pReturn = new CameraQHY5();
#endif
#if defined(OPENSSAG_CAMERA)
        else if (choice.Contains(_T("Orion StarShoot Autoguider")))
            pReturn = new CameraOpenSSAG();
#endif
#if defined(KWIQGUIDER_CAMERA)
        else if (choice.Contains(_T("KWIQGuider")))
            pReturn = KWIQGuiderCameraFactory::MakeKWIQGuiderCamera();
#endif
#if defined(SSAG)
        else if (choice.Contains(_T("StarShoot Autoguider")))
            pReturn = SSAGCameraFactory::MakeSSAGCamera();
#endif
#if defined(SSPIAG)
        else if (choice.Contains(_T("StarShoot Planetary Imager & Autoguider")))
            pReturn = new CameraSSPIAG();
#endif
#if defined(ORION_DSCI)
        else if (choice.Contains(_T("Orion StarShoot DSCI")))
            pReturn = new CameraStarShootDSCI();
#endif
#if defined(SVB_CAMERA)
        else if (choice == _T("Svbony Camera"))
            pReturn = SVBCameraFactory::MakeSVBCamera();
#endif
#if defined(OGMA_CAMERA)
        else if (choice == _T("OGMA Camera"))
            pReturn = OGMACameraFactory::MakeOGMACamera();
#endif
#if defined(OPENCV_CAMERA)
        else if (choice.Contains(_T("OpenCV webcam")))
        {
            int dev = 0;
            if (choice.Contains(_T("2")))
            {
                dev = 1;
            }
            pReturn = new CameraOpenCV(dev);
        }
#endif
#if defined(WDM_CAMERA)
        else if (choice.Contains(_T("Windows WDM")))
            pReturn = WDMCameraFactory::MakeWDMCamera();
#endif
#if defined(VFW_CAMERA)
        else if (choice.Contains(_T("Windows VFW")))
            pReturn = new CameraVFW();
#endif
#if defined(LE_SERIAL_CAMERA)
        else if (choice.Contains(_T("Long exposure Serial webcam")))
            pReturn = LESerialWebcamCameraFactory::MakeLESerialWebcamCamera();
#endif
#if defined(LE_PARALLEL_CAMERA)
        else if (choice.Contains(_T("Long exposure Parallel webcam")))
            pReturn = LEParallelWebcamCameraFactory::MakeLEParallelWebcamCamera();
#endif
#if defined(LE_LXUSB_CAMERA)
        else if (choice.Contains(_T("Long exposure LXUSB webcam")))
            pReturn = LELxUsbWebcamCameraFactory::MakeLELxUsbWebcamCamera();
#endif
#if defined(MEADE_DSI_CAMERA)
        else if (choice.Contains(_T("Meade DSI I, II, or III")))
            pReturn = DSICameraFactory::MakeDSICamera();
#endif
#if defined(MORAVIAN_CAMERA)
        else if (choice == _T("Moravian Camera"))
            pReturn = MoravianCameraFactory::MakeMoravianCamera();
#endif
#if defined(STARFISH_CAMERA)
        else if (choice.Contains(_T("Fishcamp Starfish")))
            pReturn = StarfishCameraFactory::MakeStarfishCamera();
#endif
#if defined(SXV)
        else if (choice.Contains(_T("Starlight Xpress SXV")))
            pReturn = SXVCameraFactory::MakeSXVCamera();
#endif
#if defined(OS_PL130)
        else if (choice.Contains(_T("Opticstar PL-130M")))
        {
            Camera_OSPL130.Color = false;
            Camera_OSPL130.Name = _T("Opticstar PL-130M");
            pReturn = new Camera_OSPL130Class();
        }
        else if (choice.Contains(_T("Opticstar PL-130C")))
        {
            Camera_OSPL130.Color = true;
            Camera_OSPL130.Name = _T("Opticstar PL-130C");
            pReturn = new Camera_OSPL130Class();
        }
#endif
#if defined(NEB_SBIG)
        else if (choice.Contains(_T("Nebulosity")))
            pReturn = new CameraNebSBIG();
#endif
#if defined(SBIGROTATOR_CAMERA)
        // must go above SBIG
        else if (choice.Contains(_T("SBIG Rotator")))
            pReturn = SBIGRotatorCameraFactory::MakeSBIGRotatorCamera();
#endif
#if defined(SBIG)
        else if (choice.Contains(_T("SBIG")))
            pReturn = SBIGCameraFactory::MakeSBIGCamera();
#endif
#if defined(FIREWIRE_CAMERA)
        else if (choice.Contains(_T("The Imaging Source (DCAM Firewire)")))
            pReturn = new CameraFirewire();
#endif
#if defined(INOVA_PLC)
        else if (choice.Contains(_T("i-Nova PLC-M")))
            pReturn = new CameraINovaPLC();
#endif
#if defined(V4L_CAMERA)
        else if (choice.Contains(_T("V4L(2) Camera")))
        {
            // There is at least ONE V4L(2) device ... let's find out exactly
            DeviceInfo *deviceInfo = nullptr;

            if (Camera_VIDEODEVICE.NumberOfDevices() == 1)
            {
                deviceInfo = Camera_VIDEODEVICE.GetDeviceAtIndex(0);

                Camera_VIDEODEVICE.SetDevice(deviceInfo->getDeviceName());
                Camera_VIDEODEVICE.SetVendor(deviceInfo->getVendorId());
                Camera_VIDEODEVICE.SetModel(deviceInfo->getModelId());

                Camera_VIDEODEVICE.Name = deviceInfo->getProduct();
            }
            else
            {
                wxArrayString choices;
                int choice = 0;

                if ((choice = wxGetSinglechoiceIndex(_("Select your camera"), _T("V4L(2) devices"),
                                                     Camera_VIDEODEVICE.GetProductArray(choices))) != -1)
                {
                    deviceInfo = Camera_VIDEODEVICE.GetDeviceAtIndex(choice);

                    Camera_VIDEODEVICE.SetDevice(deviceInfo->getDeviceName());
                    Camera_VIDEODEVICE.SetVendor(deviceInfo->getVendorId());
                    Camera_VIDEODEVICE.SetModel(deviceInfo->getModelId());

                    Camera_VIDEODEVICE.Name = deviceInfo->getProduct();
                }
                else
                {
                    throw ERROR_INFO("CameraFactory invalid V4L choice");
                }
            }

            pReturn = new Camera_VIDEODEVICEClass();
        }
#endif
        else
        {
            throw ERROR_INFO("CameraFactory: Unknown camera choice");
        }
    }
    catch (const wxString& Msg)
    {
        POSSIBLY_UNUSED(Msg);
        if (pReturn)
        {
            delete pReturn;
            pReturn = nullptr;
        }
    }

    return pReturn;
}

// Read a 2-D uint16 little-endian numpy .npy file into a flat pixel vector.
// Only supports C-order (fortran_order: False) uint16 ('<u2') arrays as written
// by cam_characterise.py --save-master.
static bool ReadNpyUint16(const wxString& path, int& width, int& height,
                           std::vector<unsigned short>& data)
{
    wxFile f;
    if (!f.Open(path, wxFile::read))
    {
        Debug.Write(wxString::Format("ReadNpy: cannot open %s\n", path));
        return false;
    }

    unsigned char magic[8];
    if (f.Read(magic, 8) != 8 || memcmp(magic, "\x93NUMPY", 6) != 0)
    {
        Debug.Write(wxString::Format("ReadNpy: bad magic in %s\n", path));
        return false;
    }

    unsigned char major = magic[6];
    uint32_t hlen = 0;
    if (major == 1)
    {
        unsigned char b[2];
        if (f.Read(b, 2) != 2) return false;
        hlen = b[0] | ((uint32_t)b[1] << 8);
    }
    else // v2+: 4-byte header length
    {
        unsigned char b[4];
        if (f.Read(b, 4) != 4) return false;
        hlen = b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    }
    if (hlen == 0 || hlen > 65536) // real npy headers are <1KB; cap corrupt lengths
    {
        Debug.Write(wxString::Format("ReadNpy: implausible header length %u in %s\n", hlen, path));
        return false;
    }

    std::string hdr(hlen, '\0');
    if (f.Read(&hdr[0], hlen) != (size_t)hlen)
    {
        Debug.Write(wxString::Format("ReadNpy: short header in %s\n", path));
        return false;
    }

    if (hdr.find("'<u2'") == std::string::npos && hdr.find("\"<u2\"") == std::string::npos)
    {
        Debug.Write(wxString::Format("ReadNpy: expected '<u2' (uint16 LE) dtype in %s\n", path));
        return false;
    }
    if (hdr.find("fortran_order': True") != std::string::npos)
    {
        Debug.Write("ReadNpy: fortran_order True not supported\n");
        return false;
    }

    // Parse "shape': (H, W)"
    size_t sp = hdr.find("shape");
    size_t op = (sp != std::string::npos) ? hdr.find('(', sp) : std::string::npos;
    size_t cp = (op != std::string::npos) ? hdr.find(')', op) : std::string::npos;
    if (cp == std::string::npos)
    {
        Debug.Write(wxString::Format("ReadNpy: cannot parse shape in %s\n", path));
        return false;
    }
    std::string sh = hdr.substr(op + 1, cp - op - 1);
    size_t comma = sh.find(',');
    if (comma == std::string::npos) return false;
    try {
        height = std::stoi(sh.substr(0, comma));
        width  = std::stoi(sh.substr(comma + 1));
    } catch (...) { return false; }
    if (width <= 0 || height <= 0 || width > 16384 || height > 16384) return false;

    size_t npix = (size_t)width * height;
    data.resize(npix);
    if (f.Read(data.data(), npix * 2) != npix * 2)
    {
        Debug.Write(wxString::Format("ReadNpy: short data in %s\n", path));
        data.clear();
        return false;
    }
    return true;
}

// JSON number grammar including scientific notation: Python's json.dump emits
// e.g. "6e-05" for any |x| < 1e-4, which a plain [0-9.]+ pattern silently fails
// to match -- and the dark-current slope is exactly that small on a low-dark-
// current sensor. wxString::ToDouble (strtod) parses the matched text fine.
#define JSON_NUM "(-?[0-9]+(?:\\.[0-9]*)?(?:[eE][-+]?[0-9]+)?)"

bool GuideCamera::LoadFpsCalibration(const wxString& path)
{
    m_fpsCalib = FpsExposureCalib();

    wxTextFile f;
    if (!f.Open(path))
    {
        Debug.Write(wxString::Format("FpsCalib: cannot open %s\n", path));
        return false;
    }
    wxString text;
    for (wxString line = f.GetFirstLine(); !f.Eof(); line = f.GetNextLine())
        text += line + "\n";
    f.Close();

    wxRegEx re_fps_max("\"fps_max_bandwidth\"\\s*:\\s*" JSON_NUM);
    wxRegEx re_limit("\"integration_limited_below_fps\"\\s*:\\s*" JSON_NUM);
    wxRegEx re_afps("\"anchors_fps\"\\s*:\\s*\\[([^\\]]+)\\]");
    wxRegEx re_aexp("\"anchors_exposure\"\\s*:\\s*\\[([^\\]]+)\\]");

    double fps_max = 0.0, fps_limit = 0.0;
    if (!re_fps_max.Matches(text) || !re_fps_max.GetMatch(text, 1).ToDouble(&fps_max) ||
        !re_limit.Matches(text) || !re_limit.GetMatch(text, 1).ToDouble(&fps_limit))
    {
        Debug.Write(wxString::Format("FpsCalib: missing scalar fields in %s\n", path));
        return false;
    }

    auto parseDoubles = [](const wxString& s, std::vector<double>& out) {
        wxStringTokenizer tok(s, ",");
        while (tok.HasMoreTokens()) {
            double v;
            if (tok.GetNextToken().Trim(true).Trim(false).ToDouble(&v))
                out.push_back(v);
        }
        return !out.empty();
    };
    auto parseInts = [](const wxString& s, std::vector<int>& out) {
        wxStringTokenizer tok(s, ",");
        while (tok.HasMoreTokens()) {
            long v;
            if (tok.GetNextToken().Trim(true).Trim(false).ToLong(&v))
                out.push_back((int)v);
        }
        return !out.empty();
    };

    std::vector<double> anchors_fps;
    std::vector<int> anchors_exp;
    if (!re_afps.Matches(text) || !parseDoubles(re_afps.GetMatch(text, 1), anchors_fps) ||
        !re_aexp.Matches(text) || !parseInts(re_aexp.GetMatch(text, 1), anchors_exp))
    {
        Debug.Write(wxString::Format("FpsCalib: missing anchor arrays in %s\n", path));
        return false;
    }
    if (anchors_fps.size() != anchors_exp.size() || anchors_fps.size() < 2)
    {
        Debug.Write("FpsCalib: anchor array size mismatch or too few points\n");
        return false;
    }

    m_fpsCalib.valid = true;
    m_fpsCalib.fps_max_bandwidth = fps_max;
    m_fpsCalib.integration_limited_below_fps = fps_limit;
    m_fpsCalib.anchors_fps = anchors_fps;
    m_fpsCalib.anchors_exp_units = anchors_exp;
    m_fpsCalib.valid_fps_min = anchors_fps.front();
    m_fpsCalib.valid_fps_max = anchors_fps.back();

    // anchors_exp_units[0] is the longest real integration (low fps anchor)
    int max_integ_ms = (int)(anchors_exp.front() * 0.1 + 0.5);

    Debug.Write(wxString::Format(
        "FpsCalib: loaded %s: fps_max=%.1f limit=%.1f %d anchors "
        "valid=[%.1f..%.1f] max_real_integration=%dms\n",
        path, fps_max, fps_limit, (int)anchors_fps.size(),
        anchors_fps.front(), anchors_fps.back(), max_integ_ms));
    return true;
}

int GuideCamera::EstimateExposureMs(double fps) const
{
    if (!m_fpsCalib.valid)
        return -1;
    if (fps >= m_fpsCalib.integration_limited_below_fps)
        return -1; // bandwidth-limited region: fps carries no exposure information

    const std::vector<double>& af = m_fpsCalib.anchors_fps;
    const std::vector<int>&    ae = m_fpsCalib.anchors_exp_units;

    // Clamp to calibrated range edges
    if (fps <= af.front())
        return (int)(ae.front() * 0.1 + 0.5);
    if (fps >= af.back())
        return (int)(ae.back() * 0.1 + 0.5);

    // Linear interpolation between anchor pair
    for (size_t i = 1; i < af.size(); i++)
    {
        if (fps <= af[i])
        {
            double denom = af[i] - af[i - 1];
            if (denom <= 0.0) // duplicate/unsorted anchors from the producer
                return (int)(ae[i] * 0.1 + 0.5);
            double t = (fps - af[i - 1]) / denom;
            double exp_units = ae[i - 1] + t * (ae[i] - ae[i - 1]);
            return (int)(exp_units * 0.1 + 0.5); // 100µs units -> ms
        }
    }
    return -1;
}

#ifdef __linux__
// Derive a stable device identifier from sysfs: vid+pid[_serial].
// Mirrors the usb_device_tag() function in cam_manager.py so that PHD2 and the
// characterisation tools use the same subdirectory name for calibration files.
static wxString GetUsbDeviceTag(const wxString& devicePath)
{
    wxString name = wxFileName(devicePath).GetFullName(); // "video0"
    wxString symlinkPath = "/sys/class/video4linux/" + name + "/device";

    char resolved[4096] = {};
    if (!realpath(symlinkPath.mb_str(), resolved))
    {
        Debug.Write(wxString::Format("DevTag: realpath failed for %s\n", symlinkPath));
        return wxEmptyString;
    }

    // resolved is the UVC interface dir; its parent is the USB device node
    wxString usbDev = wxFileName(wxString::FromUTF8(resolved)).GetPath();

    auto readSys = [&](const wxString& file) -> wxString {
        wxTextFile f;
        if (!f.Open(usbDev + "/" + file)) return wxEmptyString;
        wxString v = f.GetFirstLine(); f.Close();
        return v.Trim(true).Trim(false);
    };

    wxString vid = readSys("idVendor");
    wxString pid = readSys("idProduct");
    wxString sn  = readSys("serial");

    if (vid.IsEmpty() || pid.IsEmpty())
    {
        Debug.Write(wxString::Format("DevTag: no idVendor/idProduct under %s\n", usbDev));
        return wxEmptyString;
    }

    wxString tag = vid + pid; // e.g. "046d0825"
    if (!sn.IsEmpty())
    {
        // ASCII alnum or '-' only, exactly matching cam_manager.py's
        // re.sub(r"[^a-zA-Z0-9\-]", "_", sn). wxIsalnum is Unicode-aware and
        // would keep characters Python replaces, deriving a DIFFERENT
        // directory name for the same device.
        for (size_t i = 0; i < sn.size(); i++)
        {
            wxUniChar c = sn[i];
            bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                      (c >= '0' && c <= '9') || c == '-';
            if (!ok)
                sn[i] = '_';
        }
        tag += "_" + sn;
    }
    Debug.Write(wxString::Format("DevTag: %s -> %s\n", devicePath, tag));
    return tag;
}
#endif // __linux__

bool GuideCamera::LoadDarkCurrentModel(const wxString& path)
{
    m_darkModel = DarkCurrentModel();

    wxTextFile f;
    if (!f.Open(path))
    {
        Debug.Write(wxString::Format("DarkModel: cannot open %s\n", path));
        return false;
    }
    wxString text;
    for (wxString line = f.GetFirstLine(); !f.Eof(); line = f.GetNextLine())
        text += line + "\n";
    f.Close();

    wxRegEx re_slope("\"dark_current_adu_per_unit\"\\s*:\\s*" JSON_NUM);
    wxRegEx re_bias("\"bias_offset_adu\"\\s*:\\s*" JSON_NUM);
    wxRegEx re_units("\"exposure_max_units\"\\s*:\\s*([0-9]+)");
    wxRegEx re_ms("\"exposure_max_ms\"\\s*:\\s*" JSON_NUM);
    wxRegEx re_depth("\"pixel_depth\"\\s*:\\s*([0-9]+)");
    wxRegEx re_r2("\"r2\"\\s*:\\s*" JSON_NUM); // r2 can be negative for a bad fit
    wxRegEx re_linear("\"linear\"\\s*:\\s*(true|false)");
    wxRegEx re_verdict("\"exposure_fidelity_verdict\"\\s*:\\s*\"([^\"]+)\"");

    double slope = 0.0, bias = 0.0, exp_ms = 0.0, r2 = 0.0;
    long exp_units = 0, depth = 8;
    bool linear = false;
    wxString verdict;

    if (!re_slope.Matches(text) || !re_slope.GetMatch(text, 1).ToDouble(&slope) ||
        !re_bias.Matches(text)  || !re_bias.GetMatch(text, 1).ToDouble(&bias))
    {
        Debug.Write(wxString::Format("DarkModel: missing dark_current_fit fields in %s\n", path));
        return false;
    }
    if (re_units.Matches(text))
        re_units.GetMatch(text, 1).ToLong(&exp_units);
    if (re_ms.Matches(text))
        re_ms.GetMatch(text, 1).ToDouble(&exp_ms);
    else if (exp_units > 0)
        exp_ms = exp_units * 0.1;
    if (re_depth.Matches(text))
        re_depth.GetMatch(text, 1).ToLong(&depth);
    if (re_r2.Matches(text))
        re_r2.GetMatch(text, 1).ToDouble(&r2);
    if (re_linear.Matches(text))
        linear = (re_linear.GetMatch(text, 1) == "true");
    if (re_verdict.Matches(text))
        verdict = re_verdict.GetMatch(text, 1);

    if (exp_ms <= 0.0)
    {
        Debug.Write(wxString::Format("DarkModel: cannot determine exposure duration from %s\n", path));
        return false;
    }

    m_darkModel.valid = true;
    m_darkModel.dark_current_adu_per_unit = slope;
    m_darkModel.bias_offset_adu = bias;
    m_darkModel.exposure_max_units = (int)exp_units;
    m_darkModel.exposure_max_ms = exp_ms;
    m_darkModel.pixel_depth = (int)depth;
    m_darkModel.r2 = r2;
    m_darkModel.linear = linear;
    m_darkModel.fidelity_verdict = verdict;

    Debug.Write(wxString::Format(
        "DarkModel: loaded from %s: slope=%.5f bias=%.2f exp_max=%dms "
        "r2=%.3f linear=%s verdict=%s\n",
        path, slope, bias, (int)exp_ms, r2, linear ? "yes" : "no", verdict));
    return true;
}

// Build a usImage dark by scaling the master's dark-current component to a
// different exposure. Master pixel = bias + dark_current_pixel * max_exposure,
// so scaling (pixel - bias) by (exp/max) while holding bias fixed gives the
// physically-correct dark at `expMs` for a linear dark current. `master`,
// `bias` and the returned image must all be in the SAME pixel domain as the
// camera's live frames (the caller converts before calling).
static usImage *ScaleDark(const std::vector<unsigned short>& master, int w, int h,
                           double bias, double ratio, int expMs, int bpp)
{
    usImage *d = new usImage();
    if (d->Init(w, h))
    {
        delete d;
        return nullptr;
    }
    size_t n = (size_t)w * h;
    for (size_t i = 0; i < n; i++)
    {
        double v = bias + ((double)master[i] - bias) * ratio;
        if (v < 0.0) v = 0.0;
        else if (v > 65535.0) v = 65535.0;
        d->ImageData[i] = (unsigned short)(v + 0.5);
    }
    d->ImgExpDur    = expMs;
    d->BitsPerPixel = bpp;
    d->CalcStats();
    return d;
}

bool GuideCamera::ImportMasterDark(const wxString& npyPath)
{
    if (!m_darkModel.valid)
    {
        Debug.Write("ImportMasterDark: dark model not loaded (call LoadDarkCurrentModel first)\n");
        return false;
    }

    int w = 0, h = 0;
    std::vector<unsigned short> pixels;
    if (!ReadNpyUint16(npyPath, w, h, pixels))
        return false;

    // The file is matched to this camera by frame size in its name; a mismatch
    // means a misnamed or stale file, and an off-size dark would just sit in
    // the library failing the compatibility check on every frame.
    if (w != FrameSize.GetWidth() || h != FrameSize.GetHeight())
    {
        Debug.Write(wxString::Format(
            "ImportMasterDark: %s is %dx%d but camera frame is %dx%d -- refusing\n",
            npyPath, w, h, FrameSize.GetWidth(), FrameSize.GetHeight()));
        return false;
    }

    // Pixel domain. The .npy is stored at 16-bit full scale (8-bit luma x257),
    // but PHD2 does NOT normalise camera data by bit depth: an 8-bit camera
    // path (e.g. an INDI V4L2 CCD delivering 8-bit blobs) stores raw 0-255
    // values in the uint16 buffer, and a dark is only subtractable if it lives
    // in the SAME domain as the live frames. Convert once here. (For drivers
    // that accumulate multiple raw frames per exposure the live scale is
    // exposure-dependent and no static dark can match; the defect map, which
    // SubtractDark prefers anyway, is the correct tool there.)
    int bpp = BitsPerPixel() <= 8 ? 8 : 16; // (0/unset counts as 8-bit)
    double domainScale = (bpp == 8) ? 1.0 / 257.0 : 1.0;
    if (domainScale != 1.0)
    {
        for (size_t i = 0; i < pixels.size(); i++)
            pixels[i] = (unsigned short)((double)pixels[i] * domainScale + 0.5);
    }
    double biasDark = m_darkModel.bias_offset_adu * 257.0 * domainScale;

    int maxMs = (int)(m_darkModel.exposure_max_ms + 0.5);

    // 1. The native master, at the exposure it was actually captured at.
    usImage *master = new usImage();
    if (master->Init(w, h))
    {
        delete master;
        Debug.Write(wxString::Format("ImportMasterDark: usImage alloc failed %dx%d\n", w, h));
        return false;
    }
    memcpy(master->ImageData, pixels.data(), (size_t)w * h * sizeof(unsigned short));
    master->ImgExpDur    = maxMs;
    master->BitsPerPixel = bpp;
    master->CalcStats();
    AddDark(master);

    // 2. If the dark current is linear, synthesize scaled darks at each PHD2
    //    exposure duration shorter than the master, so SelectDark finds a close
    //    match for the user's actual guide exposure instead of subtracting an
    //    over-long master (which over-removes hot pixels into clamped holes).
    //    Scaling DOWN only (ratio < 1, interpolation); longer exposures fall
    //    back to the native master via SelectDark, since extrapolating dark
    //    current beyond what was measured — and amplifying master noise — is unsafe.
    int synth = 0;
    if (m_darkModel.linear && pFrame && maxMs > 0)
    {
        const std::vector<int>& durs = pFrame->GetExposureDurations();
        for (int expMs : durs)
        {
            if (expMs <= 0 || expMs >= maxMs)
                continue; // == maxMs already covered; > maxMs handled by fallback
            usImage *d = ScaleDark(pixels, w, h, biasDark, (double)expMs / maxMs, expMs, bpp);
            if (d)
            {
                AddDark(d);
                ++synth;
            }
        }
    }

    Debug.Write(wxString::Format(
        "ImportMasterDark: %dx%d master at %dms loaded from %s (%d-bit domain); "
        "%d scaled darks synthesized (linear=%s)\n",
        w, h, maxMs, npyPath, bpp, synth, m_darkModel.linear ? "yes" : "no"));
    return true;
}

bool GuideCamera::ImportDefectList(const wxString& path)
{
    wxTextFile f;
    if (!f.Open(path))
    {
        Debug.Write(wxString::Format("ImportDefectList: cannot open %s\n", path));
        return false;
    }

    DefectMap *map = new DefectMap();
    int count = 0;
    for (wxString line = f.GetFirstLine(); !f.Eof(); line = f.GetNextLine())
    {
        line.Trim(false);
        if (line.IsEmpty() || line.StartsWith("#"))
            continue;
        wxStringTokenizer tok(line);
        long x, y;
        if (tok.GetNextToken().ToLong(&x) && tok.GetNextToken().ToLong(&y))
        {
            map->push_back(wxPoint((int)x, (int)y));
            ++count;
        }
    }
    f.Close();

    SetDefectMap(map);
    Debug.Write(wxString::Format("ImportDefectList: %d defects loaded from %s\n", count, path));
    return true;
}

// ConnectCamera is the one place where we call camera->Connect(). Any work done here
// applies to all camera types, regardless of how the various camera sub-ckasses
// implement Connect().
bool GuideCamera::ConnectCamera(GuideCamera *camera, const wxString& cameraId)
{
    bool err = camera->Connect(cameraId);
    if (err)
        return err;
    if (camera->HasFrameLimiting)
    {
        // restore the saved limit frame (if any)
        auto binning = camera->GetBinning();
        camera->LoadLimitFrame(binning);
    }
    // Auto-load cam_characterise.py artefacts keyed by USB device ID and resolution.
    // Files live in a subdirectory named after the device (vid+pid[_serial]) so
    // cameras with the same resolution never share calibration data.
    // Layout: {PHD2_data_dir}/{dev_tag}/calib_WxH.json  (and master, defects, dark_model)
    // Falls back to {PHD2_data_dir}/ if the device tag cannot be resolved.
    wxString dataDir = MyFrame::GetDefaultFileDir();

    wxString devTag;
#ifdef __linux__
    if (cameraId.StartsWith("/dev/"))
        devTag = GetUsbDeviceTag(cameraId);
#endif
    wxString devDir = devTag.IsEmpty()
        ? dataDir
        : dataDir + PATHSEPSTR + devTag;

    wxString res = wxString::Format("%dx%d",
        camera->FrameSize.GetWidth(), camera->FrameSize.GetHeight());

    wxString calPath       = devDir + PATHSEPSTR + "calib_"      + res + ".json";
    wxString darkModelPath = devDir + PATHSEPSTR + "dark_model_" + res + ".json";
    wxString npyPath       = devDir + PATHSEPSTR + "master_"     + res + ".npy";
    wxString defectsPath   = devDir + PATHSEPSTR + "defects_"    + res + ".txt";

    if (wxFileExists(calPath))
        camera->LoadFpsCalibration(calPath);
    else
        Debug.Write(wxString::Format("FpsCalib: no calibration at %s\n", calPath));

    if (wxFileExists(darkModelPath))
    {
        camera->LoadDarkCurrentModel(darkModelPath);
        if (camera->HasDarkCurrentModel() && wxFileExists(npyPath))
            camera->ImportMasterDark(npyPath);
        else if (!wxFileExists(npyPath))
            Debug.Write(wxString::Format("DarkModel: no master dark at %s\n", npyPath));
    }
    else
        Debug.Write(wxString::Format("DarkModel: no dark model at %s\n", darkModelPath));

    if (wxFileExists(defectsPath))
        camera->ImportDefectList(defectsPath);
    else
        Debug.Write(wxString::Format("DefectMap: no defect list at %s\n", defectsPath));

    // Derive optimal exposure guidance from loaded calibration and store as a
    // human-readable note. Advisory only — nothing is enforced or changed.
    camera->m_charNote = wxEmptyString;
    if (camera->HasFpsCalibration() || camera->HasDarkCurrentModel())
    {
        wxString note;

        if (camera->HasFpsCalibration())
        {
            const FpsExposureCalib& fc = camera->m_fpsCalib;
            if (fc.anchors_exp_units.size() >= 2)
            {
                // anchors sorted fps-ascending: front = longest integration, back = shortest
                int min_ms = (int)(fc.anchors_exp_units.back()  * 0.1 + 0.5);
                int max_ms = (int)(fc.anchors_exp_units.front() * 0.1 + 0.5);
                note += wxString::Format("real integration %d-%dms", min_ms, max_ms);
            }
        }

        if (camera->HasDarkCurrentModel())
        {
            const DarkCurrentModel& dm = camera->m_darkModel;
            if (!dm.fidelity_verdict.IsEmpty())
            {
                // Classify by the discriminating keyword, not the first word: all
                // verdict strings begin "in integrating region..." / "partial:", so
                // the meaningful term ("synthetic", "sub-proportional", "REAL
                // integration") sits after the "->". Check synthetic first because
                // the "partial" verdict text also contains the word "real".
                wxString v = dm.fidelity_verdict.Lower();
                wxString key;
                if (v.Contains("synthetic"))
                    key = "SYNTHETIC (firmware faking exposure)";
                else if (v.Contains("partial") || v.Contains("sub-proportional"))
                    key = "PARTIAL (clamped/quantised integration)";
                else if (v.Contains("real integration"))
                    key = "REAL integration";
                else
                    key = dm.fidelity_verdict; // unknown form: show verbatim
                if (!note.IsEmpty()) note += ", ";
                note += "verdict: " + key;
            }
            if (dm.r2 > 0.0)
            {
                if (!note.IsEmpty()) note += ", ";
                note += wxString::Format("dark r2=%.3f", dm.r2);
                if (!dm.linear)
                    note += " (non-linear: darks not scaled)";
            }
        }

        camera->m_charNote = note;

        Debug.Write(wxString::Format(
            "*** Characterisation: %s\n"
            "*** Suggested guide exposure: within real integration range shown above\n",
            note));
    }

    return err;
}

bool GuideCamera::HandleSelectCameraButtonClick(wxCommandEvent&)
{
    return false; // not handled
}

bool GuideCamera::EnumCameras(wxArrayString& names, wxArrayString& ids)
{
    return true; // error
}

bool GuideCamera::CamConnectFailed(const wxString& errorMessage)
{
    pFrame->Alert(errorMessage);
    return true; // error
}

bool GuideCamera::SetCameraGain(int cameraGain)
{
    bool bError = false;

    try
    {
        if (cameraGain < 0)
        {
            throw ERROR_INFO("cameraGain < 0");
        }
        else if (cameraGain > 100)
        {
            throw ERROR_INFO("cameraGain > 100");
        }
        GuideCameraGain = cameraGain;
    }
    catch (const wxString& Msg)
    {
        POSSIBLY_UNUSED(Msg);
        bError = true;
        GuideCameraGain = DefaultGuideCameraGain;
    }

    pConfig->Profile.SetInt("/camera/gain", GuideCameraGain);

    return bError;
}

int GuideCamera::GetDefaultCameraGain()
{
    return DefaultGuideCameraGain;
}

bool GuideCamera::SetBinning(int binning)
{
    auto hwSwBin = GetHwAndSwBinning(binning);
    auto hwBin = hwSwBin.first;
    auto swBin = hwSwBin.second;
    return SetBinning(hwBin, swBin);
}

bool GuideCamera::SetBinning(int hwBinning, int swBinning)
{
    HwBinning = wxClip(hwBinning, 1, MaxHwBinning);
    SwBinning = wxClip(swBinning, 1, (int) MAX_SOFTWARE_BINNING);

    auto binning = GetBinning();
    Debug.Write(wxString::Format("camera: set binning = %u (hw = %u, sw = %u)\n", (unsigned int) binning,
                                 (unsigned int) HwBinning, (unsigned int) SwBinning));

    pConfig->Profile.SetInt("/camera/binning", HwBinning);
    pConfig->Profile.SetInt("/camera/SoftwareBinning", SwBinning);

    // if a Limit Frame ROI is in use, adjust it for the new binning
    if (HasFrameLimiting)
    {
        // restore the saved limit frame for this binning (if any)
        LoadLimitFrame(binning);
    }

    return false;
}

bool GuideCamera::SetLimitFrame(const wxRect& roi, int binning, wxString *errorMessage)
{
    if (pFrame->pGuider->IsCalibratingOrGuiding())
    {
        *errorMessage = "Cannot set the frame limit ROI while calibrating or guiding.";
        return true;
    }

    // Construct and store limit frames for all available binning values. This allows
    // accurate binning switching without rounding errors when switching from a higher
    // binning to a lower binning.  For example, if we have a limit frame coordinate
    // value of 101 at bin 1, after switching to bin 2 the coordinate will be 50; and
    // switching back to bin 1 we can recover the inital value of 101.
    for (auto choice : GetBinningChoices())
    {
        auto new_bin = choice.first;
        wxRect const limit_frame(roi.x * binning / new_bin, roi.y * binning / new_bin, roi.width * binning / new_bin,
                                 roi.height * binning / new_bin);
        wxString const key = wxString::Format("/camera/LimitFrameBin%d", new_bin);
        pConfig->Profile.SetRect(key, limit_frame);
    }

    // load the limit frame for the current camera binning level
    binning = GetBinning();
    LoadLimitFrame(binning);

    return false; // no error
}

void GuideCamera::LoadLimitFrame(int binning)
{
    wxString const key = wxString::Format("/camera/LimitFrameBin%d", binning);
    LimitFrame = pConfig->Profile.GetRect(key);
    Debug.Write(wxString::Format("camera: updated LimitFrame => (%d,%d),(%dx%d)\n", LimitFrame.x, LimitFrame.y,
                                 LimitFrame.width, LimitFrame.height));
}

void GuideCamera::SetTimeoutMs(int ms)
{
    static const int MIN_TIMEOUT_MS = 5000;

    m_timeoutMs = wxMax(ms, MIN_TIMEOUT_MS);

    pConfig->Profile.SetInt("/camera/TimeoutMs", m_timeoutMs);
}

void GuideCamera::SetSaturationByADU(bool saturationByADU, unsigned short saturationADU)
{
    m_saturationByADU = saturationByADU;
    pConfig->Profile.SetBoolean("/camera/SaturationByADU", saturationByADU);

    if (saturationByADU)
    {
        m_saturationADU = saturationADU;
        pConfig->Profile.SetInt("/camera/SaturationADU", saturationADU);
        Debug.Write(wxString::Format("Saturation detection set to Max-ADU value %d\n", saturationADU));
    }
    else
        Debug.Write("Saturation detection set to star-profile-mode\n");
}

bool GuideCamera::SetCameraPixelSize(double pixel_size)
{
    bool bError = false;

    try
    {
        if (pixel_size <= 0.0)
        {
            throw ERROR_INFO("pixel_size <= 0");
        }

        m_pixelSize = pixel_size;
        if (pFrame->pStatsWin)
            pFrame->pStatsWin->ResetImageSize();
    }
    catch (const wxString& Msg)
    {
        POSSIBLY_UNUSED(Msg);
        bError = true;
        m_pixelSize = UnknownPixelSize;
    }

    pConfig->Profile.SetDouble("/camera/pixelsize", m_pixelSize);

    return bError;
}

bool GuideCamera::SetCoolerOn(bool on)
{
    return true; // error
}

bool GuideCamera::SetCoolerSetpoint(double temperature)
{
    return true; // error
}

bool GuideCamera::GetCoolerStatus(bool *on, double *setpoint, double *power, double *temperature)
{
    return true; // error
}

bool GuideCamera::GetSensorTemperature(double *temperature)
{
    return true; // error
}

CameraConfigDialogPane *GuideCamera::GetConfigDialogPane(wxWindow *pParent)
{
    return new CameraConfigDialogPane(pParent, this);
}

static wxSpinCtrl *NewSpinnerInt(wxWindow *parent, int width, int val, int minval, int maxval, int inc)
{
    wxSpinCtrl *pNewCtrl = pFrame->MakeSpinCtrl(parent, wxID_ANY, _T(" "), wxDefaultPosition, wxSize(width, -1),
                                                wxSP_ARROW_KEYS, minval, maxval, val);
    pNewCtrl->SetValue(val);
    return pNewCtrl;
}

static wxSpinCtrlDouble *NewSpinnerDouble(wxWindow *parent, int width, double val, double minval, double maxval, double inc,
                                          const wxString& tooltip)
{
    wxSpinCtrlDouble *pNewCtrl = pFrame->MakeSpinCtrlDouble(parent, wxID_ANY, _T(" "), wxDefaultPosition, wxSize(width, -1),
                                                            wxSP_ARROW_KEYS, minval, maxval, val, inc);
    pNewCtrl->SetDigits(2);
    pNewCtrl->SetToolTip(tooltip);
    return pNewCtrl;
}

CameraConfigDialogPane::CameraConfigDialogPane(wxWindow *pParent, GuideCamera *pCamera)
    : ConfigDialogPane(_("Camera Settings"), pParent)
{
    m_pParent = pParent;
}

static void MakeBold(wxControl *ctrl)
{
    wxFont font = ctrl->GetFont();
    font.SetWeight(wxFONTWEIGHT_BOLD);
    ctrl->SetFont(font);
}

static void FillChoiceItems(wxChoice *listBox, const wxArrayString& opts)
{
    listBox->Clear();
    listBox->Append(opts);
}

void CameraConfigDialogPane::LayoutControls(GuideCamera *pCamera, BrainCtrlIdMap& CtrlMap)
{
    wxFlexGridSizer *pTopline = new wxFlexGridSizer(1, 3, 5, 10);
    // Generic controls
    wxSizerFlags def_flags = wxSizerFlags(0).Border(wxALL, 10).Expand();
    pTopline->Add(GetSizerCtrl(CtrlMap, AD_szNoiseReduction));
    pTopline->Add(GetSizerCtrl(CtrlMap, AD_szTimeLapse), wxSizerFlags(0).Border(wxLEFT, 110).Expand());
    this->Add(pTopline, def_flags);
    this->Add(GetSizerCtrl(CtrlMap, AD_szVariableExposureDelay), def_flags);
    this->Add(GetSizerCtrl(CtrlMap, AD_szAutoExposure), def_flags);

    this->Layout();

    // Specific controls
    wxFlexGridSizer *pDetailsSizer = new wxFlexGridSizer(6, 3, 15, 15); // Will auto-shrink to fit
    if (pCamera)
    {

        // Create all possible property controls then disable individual controls later if camera doesn't support them.  This is
        // safer for "omnibus" style drivers that handle many cameras with different capabilities.
        pDetailsSizer->Add(GetSizerCtrl(CtrlMap, AD_szPixelSize));
        pDetailsSizer->Add(GetSizerCtrl(CtrlMap, AD_szGain), wxSizerFlags(0).Border(wxLEFT, 45));
        pDetailsSizer->AddSpacer(20);
        pDetailsSizer->Add(GetSizerCtrl(CtrlMap, AD_szBinning));
        pDetailsSizer->Add(GetSizerCtrl(CtrlMap, AD_szCooler));
        pDetailsSizer->AddSpacer(20);
        pDetailsSizer->Add(GetSingleCtrl(CtrlMap, AD_cbUseSubFrames), wxSizerFlags(0).Border(wxTOP, 3));
        pDetailsSizer->Add(GetSizerCtrl(CtrlMap, AD_szCameraTimeout), wxSizerFlags(0).Border(wxLEFT, 20));
        this->Layout();
    }
    else
    {
        wxStaticText *pNoCam = new wxStaticText(m_pParent, wxID_ANY, _("No camera specified"));
        this->Add(pNoCam, wxSizerFlags().Align(wxALIGN_CENTER_HORIZONTAL));
        Layout();
    }
    if (pCamera)
    {
        this->Add(GetSizerCtrl(CtrlMap, AD_szSaturationOptions), wxSizerFlags(0).Border(wxALL, 2).Expand());
        this->Add(pDetailsSizer, wxSizerFlags(0).Border(wxALL, 10).Align(wxVERTICAL).Expand());
    }

    this->Layout();
    Fit(m_pParent);
}

CameraConfigDialogCtrlSet *GuideCamera::GetConfigDlgCtrlSet(wxWindow *pParent, GuideCamera *pCamera,
                                                            AdvancedDialog *pAdvancedDialog, BrainCtrlIdMap& CtrlMap)
{
    return new CameraConfigDialogCtrlSet(pParent, pCamera, pAdvancedDialog, CtrlMap);
}

CameraConfigDialogCtrlSet::CameraConfigDialogCtrlSet(wxWindow *pParent, GuideCamera *pCamera, AdvancedDialog *pAdvancedDialog,
                                                     BrainCtrlIdMap& CtrlMap)
    : ConfigDialogCtrlSet(pParent, pAdvancedDialog, CtrlMap), m_pUseSubframes(nullptr)
{
    int textWidth = StringWidth(_T("0000"));
    assert(pCamera);

    m_pCamera = pCamera;
    // Sub-frames
    m_pUseSubframes = new wxCheckBox(GetParentWindow(AD_cbUseSubFrames), wxID_ANY, _("Use Subframes"));
    AddCtrl(CtrlMap, AD_cbUseSubFrames, m_pUseSubframes,
            _("Check to only download subframes (ROIs). Sub-frame size is equal to search region size."));

    // Pixel size
    m_pPixelSize = NewSpinnerDouble(GetParentWindow(AD_szPixelSize), textWidth, m_pCamera->GetCameraPixelSize(), 0.0, 99.9, 0.1,
                                    _("Guide camera un-binned pixel size in microns. Used with the guide telescope focal "
                                      "length to display guiding error in arc-seconds."));
    AddLabeledCtrl(CtrlMap, AD_szPixelSize, _("Pixel size"), m_pPixelSize, "");

    // Gain control
    wxWindow *parent = GetParentWindow(AD_szGain);
    wxStaticText *label = new wxStaticText(parent, wxID_ANY, _("Camera gain") + _(": "));
    m_pCameraGain = NewSpinnerInt(parent, textWidth, 100, 0, 100, 1);
    m_pCameraGain->SetToolTip(
        /* xgettext:no-c-format */ _("Camera gain, default = 95%, lower if you experience noise or wish to guide on a very "
                                     "bright star. Not available on all cameras."));
    m_resetGain =
        new wxButton(GetParentWindow(AD_szGain), wxID_ANY, _("Reset"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_resetGain->SetToolTip(_("Reset gain to camera's default value (disabled when camera is not connected)"));
    m_resetGain->Bind(wxEVT_COMMAND_BUTTON_CLICKED,
                      [this](wxCommandEvent& evt) { m_pCameraGain->SetValue(::pCamera->GetDefaultCameraGain()); });
    wxBoxSizer *sizer = new wxBoxSizer(wxHORIZONTAL);
    sizer->Add(label, wxSizerFlags().Align(wxALIGN_CENTER_VERTICAL));
    sizer->Add(m_pCameraGain, wxSizerFlags().Align(wxALIGN_CENTER_VERTICAL));
    sizer->Add(m_resetGain, wxSizerFlags().Align(wxALIGN_CENTER_VERTICAL));
    AddGroup(CtrlMap, AD_szGain, sizer);

    // Binning
    wxArrayString opts;
    bool includeSwBinning = m_pCamera->GetOfferSwBinning();
    m_pCamera->GetBinningOpts(&opts, false); // Default initialization, will be overridden in LoadValues()
    int width = StringArrayWidth(opts);
    wxStaticText *pLabel = new wxStaticText(GetParentWindow(AD_szBinning), wxID_ANY, _("Binning:"));
    m_binning = new wxChoice(GetParentWindow(AD_szBinning), wxID_ANY, wxDefaultPosition, wxSize(width + 35, -1), opts);
    m_binning->SetToolTip("Camera binning, used to optimize guider image scale or improve SNR for CCD cameras");
    m_allowSwBinning = new wxCheckBox(GetParentWindow(AD_szBinning), wxID_ANY, _("Enable software binning"));
    m_allowSwBinning->SetToolTip(_("Can be used to increase binning beyond camera hardware/driver limits. "
                                   "Try to keep the guider image scale > 0.5 arc-sec/px."));
    wxSizer *szB = new wxBoxSizer(wxHORIZONTAL);
    szB->Add(pLabel, wxSizerFlags(0).Align(wxALIGN_CENTER_VERTICAL));
    szB->Add(m_binning, wxSizerFlags(0).Border(wxLEFT, 2));
    szB->Add(m_allowSwBinning, wxSizerFlags(0).Align(wxALIGN_CENTER_VERTICAL).Border(wxLEFT, 4));
    m_allowSwBinning->SetValue(false); // May be overridden in LoadValues()
    m_allowSwBinning->Enable(includeSwBinning);

    m_allowSwBinning->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &CameraConfigDialogCtrlSet::OnSwBinningChecked, this);
    AddGroup(CtrlMap, AD_szBinning, szB);

    // Cooler
    wxSizer *sz = new wxBoxSizer(wxHORIZONTAL);
    m_coolerOn = new wxCheckBox(GetParentWindow(AD_szCooler), wxID_ANY, _("Cooler On"));
    m_coolerOn->SetToolTip(_("Turn camera cooler on or off"));
    sz->Add(m_coolerOn, wxSizerFlags().Align(wxALIGN_CENTER_VERTICAL).Border(wxRIGHT));
    m_coolerSetpt = NewSpinnerInt(GetParentWindow(AD_szCooler), textWidth, 5, -99, 99, 1);
    wxSizer *szt = MakeLabeledControl(AD_szCooler, _("Set Temperature"), m_coolerSetpt, _("Cooler setpoint temperature"));
    sz->Add(szt, wxSizerFlags().Align(wxALIGN_CENTER_VERTICAL));
    AddGroup(CtrlMap, AD_szCooler, sz);

    // Max ADU and related saturation choices in a single group
    width = StringWidth(_T("65535"));
    parent = GetParentWindow(AD_szSaturationOptions);
    m_camSaturationADU = new wxTextCtrl(parent, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(1.5 * width, -1));
    m_camSaturationADU->SetToolTip(
        _("ADU level to determine saturation - 65535 for most 16-bit cameras, or 255 for 8-bit cameras."));
    m_SaturationByADU = new wxRadioButton(parent, wxID_ANY, _("Saturation by Max-ADU value:"));
    m_SaturationByADU->SetToolTip(_("Identify star saturation based on camera maximum-ADU value (recommended)"));
    m_SaturationByADU->Bind(wxEVT_COMMAND_RADIOBUTTON_SELECTED, &CameraConfigDialogCtrlSet::OnSaturationChoiceChanged, this);
    wxStaticBoxSizer *szADUGroup = new wxStaticBoxSizer(wxHORIZONTAL, parent, wxEmptyString);
    szADUGroup->Add(m_SaturationByADU, wxSizerFlags().Border(wxTOP, 2));
    szADUGroup->Add(m_camSaturationADU, wxSizerFlags().Border(wxLEFT, 6));

    m_SaturationByProfile = new wxRadioButton(parent, wxID_ANY, _("Saturation via star-profile"));
    m_SaturationByProfile->SetToolTip(_("Identify star saturation based on flat-topped profile, regardless of brightness"));
    m_SaturationByProfile->Bind(wxEVT_COMMAND_RADIOBUTTON_SELECTED, &CameraConfigDialogCtrlSet::OnSaturationChoiceChanged,
                                this);
    wxFlexGridSizer *szSaturationGroup = new wxFlexGridSizer(1, 2, 5, 15);

    szSaturationGroup->Add(szADUGroup, wxSizerFlags().Border(wxALL, 3).Align(wxALIGN_CENTER_VERTICAL));
    szSaturationGroup->Add(m_SaturationByProfile, wxSizerFlags(0).Border(wxLEFT, 70).Expand().Align(wxALIGN_CENTER_VERTICAL));
    AddGroup(CtrlMap, AD_szSaturationOptions, szSaturationGroup);

    // Watchdog timeout
    m_timeoutVal = NewSpinnerInt(GetParentWindow(AD_szCameraTimeout), textWidth, 5, 5, 9999, 1);
    AddLabeledCtrl(CtrlMap, AD_szCameraTimeout, _("Disconnect nonresponsive   \n camera after (seconds)"), m_timeoutVal,
                   wxString::Format(_("The camera will be disconnected if it fails to respond for this long. "
                                      "The default value, %d seconds, should be appropriate for most cameras."),
                                    DefaultGuideCameraTimeoutMs / 1000));
}

void CameraConfigDialogCtrlSet::OnSaturationChoiceChanged(wxCommandEvent& event)
{
    m_camSaturationADU->Enable(m_SaturationByADU->GetValue());
}

void CameraConfigDialogCtrlSet::OnSwBinningChecked(wxCommandEvent& event)
{
    wxArrayString opts;
    int currBinning = GetIntChoice(m_binning, 1);
    m_pCamera->GetBinningOpts(&opts, event.IsChecked());
    FillChoiceItems(m_binning, opts);
    if (event.IsChecked())
        SetIntChoice(m_binning, currBinning);
    else
        SetIntChoice(m_binning, wxMin(currBinning, m_pCamera->MaxHwBinning));
}

static unsigned short SaturationValFromBPP(GuideCamera *cam)
{
    return (unsigned short) ((1U << cam->BitsPerPixel()) - 1);
}

void CameraConfigDialogCtrlSet::LoadValues()
{
    assert(m_pCamera);

    if (m_pCamera->HasSubframes)
    {
        m_pUseSubframes->SetValue(m_pCamera->UseSubframes);
    }
    else
    {
        m_pUseSubframes->Enable(false);
    }

    if (m_pCamera->HasGainControl)
    {
        m_pCameraGain->SetValue(m_pCamera->GetCameraGain());
        m_resetGain->Enable(m_pCamera->Connected);
    }
    else
    {
        m_pCameraGain->Enable(false);
        m_resetGain->Enable(false);
    }

    wxArrayString opts;
    int binning = m_pCamera->GetBinning();
    bool includeSwBinning = m_pCamera->GetOfferSwBinning();
    m_allowSwBinning->Enable(includeSwBinning);
    // Automatically show s/w binning options if they're likely to be needed
    if (includeSwBinning && (pFrame->GetCameraPixelScale() < 1.0 || binning > m_pCamera->MaxHwBinning))
    {
        m_allowSwBinning->SetValue(true);
        m_pCamera->GetBinningOpts(&opts, true);
    }
    else
    {
        m_allowSwBinning->SetValue(false);
        m_pCamera->GetBinningOpts(&opts, false);
    }
    FillChoiceItems(m_binning, opts);
    SetIntChoice(m_binning, binning);
    m_prevBinning = binning;
    // don't allow binning change when calibrating or guiding
    m_binning->Enable(!pFrame->pGuider || !pFrame->pGuider->IsCalibratingOrGuiding());

    m_timeoutVal->SetValue(m_pCamera->GetTimeoutMs() / 1000);

    bool saturationByADU = m_pCamera->IsSaturationByADU();
    m_SaturationByADU->SetValue(saturationByADU);
    m_SaturationByProfile->SetValue(!saturationByADU);

    if (pConfig->Profile.HasEntry("/camera/SaturationADU"))
    {
        unsigned int maxADU = wxMin(pConfig->Profile.GetInt("/camera/SaturationADU", 0), 65535);
        m_camSaturationADU->SetValue(wxString::Format("%u", maxADU));
    }
    else
    {
        // first time initialization
        int val = SaturationValFromBPP(m_pCamera);
        Debug.Write(wxString::Format("initializing cam saturation ADU val to %d\n", val));
        m_camSaturationADU->SetValue(wxString::Format("%d", val));
    }
    wxCommandEvent dummy;
    OnSaturationChoiceChanged(dummy);

    // do not allow saturation detection changes unless the camera is connected.
    // The Max ADU value needs to know the camera's BPP which may not be available
    // unless the camera is connected
    if (!m_pCamera->Connected)
    {
        m_SaturationByADU->Enable(false);
        m_SaturationByProfile->Enable(false);
        m_camSaturationADU->Enable(false);
    }

    double pxSize;
    if (m_pCamera->GetDevicePixelSize(&pxSize)) // true=>error
    {
        pxSize = m_pCamera->GetCameraPixelSize();
        m_pPixelSize->Enable(!pFrame->CaptureActive);
    }
    else
        m_pPixelSize->Enable(false); // Got a device-level pixel size, disable the control

    m_pPixelSize->SetValue(pxSize);

    if (m_pCamera->HasCooler)
    {
        bool ok = false;
        bool on;
        double setpt;

        if (m_pCamera->Connected)
        {
            double power, temp;
            bool err = m_pCamera->GetCoolerStatus(&on, &setpt, &power, &temp);
            if (!err)
                ok = true;
        }

        if (ok)
        {
            m_coolerOn->SetValue(on);
            if (!on)
            {
                setpt = pConfig->Profile.GetDouble("/camera/CoolerSetpt", 10.0);
            }
            m_coolerSetpt->SetValue((int) floor(setpt));
        }

        m_coolerOn->Enable(ok);
        m_coolerSetpt->Enable(ok);
    }
    else
    {
        m_coolerOn->Enable(false);
        m_coolerSetpt->Enable(false);
    }
}

void CameraConfigDialogCtrlSet::UnloadValues()
{
    assert(m_pCamera);

    if (m_pCamera->HasSubframes)
    {
        bool oldVal = m_pCamera->UseSubframes;
        bool newVal = m_pUseSubframes->GetValue();
        m_pCamera->UseSubframes = newVal;
        pConfig->Profile.SetBoolean("/camera/UseSubframes", newVal);
        // MultiStar can't track secondary star locations during periods when subframes are used
        if (oldVal && !newVal)
            if (pFrame->pGuider->GetMultiStarMode())
                pFrame->pGuider->SetMultiStarMode(true); // Will force a refresh of secondary stars
    }

    if (m_pCamera->HasGainControl)
    {
        m_pCamera->SetCameraGain(m_pCameraGain->GetValue());
    }

    int oldBin = m_pCamera->GetBinning();
    int newBin = GetIntChoice(m_binning, 1);
    if (newBin != oldBin)
        pFrame->pAdvancedDialog->FlagImageScaleChange();
    m_pCamera->SetBinning(newBin);

    m_pCamera->SetTimeoutMs(m_timeoutVal->GetValue() * 1000);

    double oldPxSz = m_pCamera->GetCameraPixelSize();
    double newPxSz = m_pPixelSize->GetValue();
    if (oldPxSz != newPxSz &&
        pFrame->pAdvancedDialog->PercentChange(oldPxSz, newPxSz) >
            5.0) // Avoid rounding problems with floating point equality test; don't clear
                 // calibration for inconsequential changes
        pFrame->pAdvancedDialog->FlagImageScaleChange();
    m_pCamera->SetCameraPixelSize(m_pPixelSize->GetValue());

    bool saturationByADU = m_SaturationByADU->GetValue();
    unsigned short saturationVal = 0;

    if (saturationByADU)
    {
        long val = 0;
        m_camSaturationADU->GetValue().ToLong(&val);
        if (val > 0)
        {
            saturationVal = wxMin(val, SaturationValFromBPP(m_pCamera));
        }
        else
        {
            // user-entered zero treated as 'set to default'
            saturationVal = SaturationValFromBPP(m_pCamera);
        }
    }

    m_pCamera->SetSaturationByADU(saturationByADU, saturationVal);

    if (m_pCamera->HasCooler)
    {
        bool on = m_coolerOn->GetValue();
        m_pCamera->SetCoolerOn(on);
        double setpt = (double) m_coolerSetpt->GetValue();
        m_pCamera->SetCoolerSetpoint(setpt);
        pConfig->Profile.SetDouble("/camera/CoolerSetpt", setpt);
    }

    pFrame->pStatsWin->UpdateCooler();
}

double CameraConfigDialogCtrlSet::GetPixelSize()
{
    return m_pPixelSize->GetValue();
}

void CameraConfigDialogCtrlSet::SetPixelSize(double val)
{
    m_pPixelSize->SetValue(val);
}

int CameraConfigDialogCtrlSet::GetBinning()
{
    return GetIntChoice(m_binning, 1);
}

void CameraConfigDialogCtrlSet::SetBinning(int binning)
{
    SetIntChoice(m_binning, binning);
}

void GuideCamera::GetBinningOpts(wxArrayString *opts, int maxHwBinning, bool includeSwBinning)
{
    for (auto choice : GetBinningChoices(maxHwBinning))
    {
        if (includeSwBinning || choice.second.second == 1)
            opts->Add(wxString::Format("%d", choice.first));
    }
}

// Get all the available binning choices for both hardware and software binning.
// Hardware binning takes precedence over software binning.
//
// The max combined binning level is the camera's max binning or the software max
// binning (4), whichever is greater.
//
// Example: maxHwBin = 2
//    combined     hw     sw
//         1        1      1
//         2        2      1
//         3        1      3
//         4        2      2
BinningChoices GuideCamera::GetBinningChoices(int maxHwBin)
{
    int maxSwBin = GuideCamera::MAX_SOFTWARE_BINNING;
    int maxCombinedBin = wxMax(maxHwBin, maxSwBin);
    BinningChoices choices;
    for (int hwBin = 1; hwBin <= maxHwBin; hwBin++)
        for (int swBin = 1; swBin <= maxSwBin; swBin++)
        {
            auto combined = hwBin * swBin;
            if (combined > maxCombinedBin)
                continue;
            auto it = choices.find(combined);
            if (it == choices.end() || hwBin > it->second.first)
                choices[combined] = std::make_pair(hwBin, swBin);
        }
    return choices;
}

// Get the hardware and software binning levels for a given combined binning level and
// maximum hardware binning level.
//
// Uses GuideCamera::GetBinningChoices() to generate the list of valid choices, then
// selects the closest match having a combined binning value less than or equal to the
// requested value.
std::pair<int, int> GuideCamera::GetHwAndSwBinning(int maxHwBinning, int combinedBinning)
{
    auto prev = std::make_pair(1, 1);
    for (auto choice : GetBinningChoices(maxHwBinning))
    {
        if (choice.first == combinedBinning)
            return choice.second;
        if (choice.first > combinedBinning)
            return prev;
        prev = choice.second;
    }
    return prev;
}

wxString GuideCamera::GetSettingsSummary()
{
    int darkDur;

    { // lock scope
        wxCriticalSectionLocker lck(DarkFrameLock);
        darkDur = CurrentDarkFrame ? CurrentDarkFrame->ImgExpDur : 0;
    } // lock scope

    // return a loggable summary of current camera settings
    wxString pixelSizeStr;
    if (m_pixelSize == UnknownPixelSize)
        pixelSizeStr = _("unspecified");
    else
        pixelSizeStr = wxString::Format(_("%0.1f um"), m_pixelSize);

    wxString s = wxString::Format(
        "Camera = %s%s, full size = %d x %d, %s, %s, pixel size = %s\n", Name,
        HasGainControl ? wxString::Format(", gain = %d", GuideCameraGain) : "",
        FrameSize.GetWidth(), FrameSize.GetHeight(),
        darkDur ? wxString::Format("have dark, dark dur = %d", darkDur) : "no dark",
        CurrentDefectMap ? "defect map in use" : "no defect map", pixelSizeStr);
    if (!m_charNote.IsEmpty())
        s += "  Characterisation: " + m_charNote + "\n";
    return s;
}

void GuideCamera::AddDark(usImage *dark)
{
    int const expdur = dark->ImgExpDur;

    { // lock scope
        wxCriticalSectionLocker lck(DarkFrameLock);

        // free the prior dark with this exposure duration
        ExposureImgMap::iterator pos = Darks.find(expdur);
        if (pos != Darks.end())
        {
            usImage *prior = pos->second;
            if (prior == CurrentDarkFrame)
                CurrentDarkFrame = dark;
            delete prior;
        }

    } // lock scope

    Darks[expdur] = dark;
}

void GuideCamera::SelectDark(int exposureDuration)
{
    // select the dark frame with the smallest exposure >= the requested exposure.
    // if there are no darks with exposures > the select exposure, select the dark with the greatest exposure

    wxCriticalSectionLocker lck(DarkFrameLock);

    CurrentDarkFrame = 0;
    for (ExposureImgMap::const_iterator it = Darks.begin(); it != Darks.end(); ++it)
    {
        CurrentDarkFrame = it->second;
        if (it->first >= exposureDuration)
            break;
    }
}

void GuideCamera::GetDarkLibraryProperties(int *pNumDarks, double *pMinExp, double *pMaxExp)
{
    double minExp = 9999.0;
    double maxExp = -9999.0;
    int ct = 0;

    { // lock scope
        wxCriticalSectionLocker lck(DarkFrameLock);

        for (auto it = Darks.begin(); it != Darks.end(); ++it)
        {
            if (it->first < minExp)
                minExp = it->first;
            if (it->first > maxExp)
                maxExp = it->first;
            ++ct;
        }
    } // lock scope

    *pNumDarks = ct;
    *pMinExp = minExp;
    *pMaxExp = maxExp;
}

void GuideCamera::ClearDefectMap()
{
    wxCriticalSectionLocker lck(DarkFrameLock);

    if (CurrentDefectMap)
    {
        Debug.AddLine("Clearing defect map...");
        delete CurrentDefectMap;
        CurrentDefectMap = nullptr;
    }
}

void GuideCamera::SetDefectMap(DefectMap *defectMap)
{
    wxCriticalSectionLocker lck(DarkFrameLock);
    delete CurrentDefectMap;
    CurrentDefectMap = defectMap;
}

void GuideCamera::ClearDarks()
{
    wxCriticalSectionLocker lck(DarkFrameLock);
    while (!Darks.empty())
    {
        ExposureImgMap::iterator it = Darks.begin();
        delete it->second;
        Darks.erase(it);
    }
    CurrentDarkFrame = nullptr;
}

void GuideCamera::SubtractDark(usImage& img)
{
    // dark subtraction is done in the camera worker thread, so we need to acquire the
    // DarkFrameLock to protect against the dark frame disappearing when the main
    // thread does "Load Darks" or "Clear Darks"

    wxCriticalSectionLocker lck(DarkFrameLock);

    if (CurrentDefectMap)
    {
        RemoveDefects(img, *CurrentDefectMap);
    }
    else if (CurrentDarkFrame)
    {
        Subtract(img, *CurrentDarkFrame);
    }
}

static void InitiateReconnect()
{
    WorkerThread *thr = WorkerThread::This();
    if (thr)
    {
        // Defer sending the completion of exposure message until after
        // the camera re-connecttion attempt
        thr->SetSkipExposeComplete();
    }
    pFrame->TryReconnect();
}

void GuideCamera::DisconnectWithAlert(CaptureFailType type)
{
    switch (type)
    {
    case CAPT_FAIL_MEMORY:
        DisconnectWithAlert(_("Memory allocation error during capture"), NO_RECONNECT);
        break;

    case CAPT_FAIL_TIMEOUT:
    {
        wxString msg;
        // Dark library exposure times won't match the selected exposure time in the pull-down menu of the main window
        if (!ShutterClosed)
            msg = (wxString::Format(
                _("After %.1f sec the camera has not completed a %.1f sec exposure, so "
                  "it has been disconnected to prevent other problems. Refer to Trouble-shooting section of Help."),
                (pFrame->RequestedExposureDuration() + m_timeoutMs) / 1000., pFrame->RequestedExposureDuration() / 1000.));
        else
            msg = _("The camera has not completed an exposure in at least 15 seconds, so "
                    "it has been disconnected to prevent other problems. Refer to Trouble-shooting section of Help.");

        DisconnectWithAlert(msg, RECONNECT);
    }
    break;
    }
}

void GuideCamera::DisconnectWithAlert(const wxString& msg, ReconnectType reconnect)
{
    Disconnect();

    // CAUTION: this function can be called from the worker thread, so
    // care must be taken not to make any direct UI updates

    pFrame->UpdateStatusBarStateLabels();
    pFrame->NotifyUpdateButtonsStatus(); // in case camera dialog button depends on connected state

    if (reconnect == RECONNECT)
    {
        pFrame->Alert(msg + "\n" + _("PHD will make several attempts to re-connect the camera."));
        InitiateReconnect();
    }
    else
    {
        pFrame->Alert(msg + "\n" +
                      _("The camera has been disconnected. Please resolve the problem and re-connect the camera."));
    }
}

void GuideCamera::InitCapture() { }

// convert a rectangle from binned coordinates to un-binned coordinates
inline static wxRect unbinned_rect(const wxRect& binnedRect, int binning)
{
    auto x = binnedRect.GetLeft() * binning;
    auto y = binnedRect.GetTop() * binning;
    auto width = binnedRect.GetWidth() * binning;
    auto heigth = binnedRect.GetHeight() * binning;
    return wxRect(x, y, width, heigth);
}

// convert a rectangle from un-binned coordinates to binned coordinates
inline static wxRect binned_rect(const wxRect& unbinnedRect, int binning)
{
    auto x = unbinnedRect.GetLeft() / binning;
    auto y = unbinnedRect.GetTop() / binning;
    auto width = unbinnedRect.GetWidth() / binning;
    auto heigth = unbinnedRect.GetHeight() / binning;
    return wxRect(x, y, width, heigth);
}

inline static wxSize binned_size(const wxSize& unbinnedSize, int binning)
{
    return wxSize(unbinnedSize.x / binning, unbinnedSize.y / binning);
}

bool GuideCamera::Capture(GuideCamera *camera, usImage& img, const CaptureParams& captureParams)
{
    // The subframe and LimitFrame are in software-binned coordinates, but the camera
    // subclass Capture methods work with hardware coordinates.
    int swBinning = captureParams.swBinning;
    CaptureParams cameraParams(captureParams);
    if (swBinning > 1)
    {
        cameraParams.limitFrame = unbinned_rect(captureParams.limitFrame, swBinning);
        cameraParams.subframe = unbinned_rect(captureParams.subframe, swBinning);
    }

    img.InitImgStartTime();
    img.LimitFrame = captureParams.limitFrame;
    img.Binning = captureParams.hwBinning;
    img.BitsPerPixel = captureParams.bpp;
    img.Gain = captureParams.gain;
    img.ImgExpDur = captureParams.duration;

    bool err = camera->Capture(img, cameraParams);
    if (err)
        return err;

    // perform software binning if needed
    if (swBinning > 1)
    {
        usImage binnedImage;
        if (binnedImage.Init(binned_size(img.Size, swBinning)))
        {
            camera->DisconnectWithAlert(CAPT_FAIL_MEMORY);
            return true;
        }
        BinPixels(binnedImage.ImageData, img.ImageData, img.Size, swBinning);
        img.SwapImageData(binnedImage);
        img.Size = binnedImage.Size;
        img.NPixels = binnedImage.NPixels;
        img.Binning *= swBinning;
        // scale the subframe from camera coords to binned coords
        img.Subframe = binned_rect(img.Subframe, swBinning);
    }

    return err;
}

bool GuideCamera::ST4HasGuideOutput()
{
    return m_hasGuideOutput;
}

bool GuideCamera::ST4HostConnected()
{
    return Connected;
}

bool GuideCamera::ST4HasNonGuiMove()
{
    // should never be called

    assert(false);
    return true;
}

bool GuideCamera::ST4PulseGuideScope(int direction, int duration)
{
    // should never be called

    assert(false);
    return true;
}
