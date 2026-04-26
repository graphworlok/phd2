/*
 *  noise_config_panel.cpp
 *  PHD Guiding
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#include "phd.h"
#include "noise_config_panel.h"
#include "noise_pipeline.h"
#include "noise_stage_hot_pixel.h"
#include "noise_stage_rolling_bg.h"
#include "noise_stage_dead_zone.h"
#include "dead_zone_map.h"
#include "camera.h"

#include <wx/checkbox.h>
#include <wx/spinctrl.h>
#include <wx/button.h>
#include <wx/stattext.h>
#include <wx/statbox.h>
#include <wx/sizer.h>
#include <wx/msgdlg.h>
#include <wx/progdlg.h>
#include <wx/dialog.h>
#include <wx/scrolwin.h>
#include <wx/dcclient.h>
#include <wx/dcmemory.h>
#include <wx/image.h>
#include <wx/bitmap.h>
#include <wx/statbmp.h>
#include <wx/choice.h>
#include <wx/filedlg.h>
#include <wx/display.h>
#include <wx/zipstrm.h>
#include <wx/wfstream.h>
#include <wx/mstream.h>
#include <wx/dir.h>
#include <wx/filename.h>
#include <wx/datetime.h>

static RollingBackgroundStage *FindRolling()
{
    if (!pNoise)
        return nullptr;
    return dynamic_cast<RollingBackgroundStage *>(pNoise->FindStage(wxT("RollingBackground")));
}

static HotPixelStage *FindHotPixel()
{
    if (!pNoise)
        return nullptr;
    return dynamic_cast<HotPixelStage *>(pNoise->FindStage(wxT("HotPixel")));
}

static NoiseStageDeadZone *FindDeadZone()
{
    if (!pNoise)
        return nullptr;
    return dynamic_cast<NoiseStageDeadZone *>(pNoise->FindStage(wxT("DeadZone")));
}

NoiseConfigPanel::NoiseConfigPanel(wxWindow *parent)
    : wxPanel(parent, wxID_ANY)
{
    wxStaticBoxSizer *rbgBox =
        new wxStaticBoxSizer(wxVERTICAL, this, _("Rolling background subtraction"));

    m_enable = new wxCheckBox(rbgBox->GetStaticBox(), wxID_ANY,
                              _("Enable rolling background model"));
    m_enable->SetToolTip(_("Maintains a per-pixel exponential-moving-average "
                           "background from observed frames and subtracts it "
                           "from each new frame. Captures vignetting, amp glow, "
                           "fixed-pattern noise, hot pixels and slow dark "
                           "drift in one pass. Adapts continuously to sensor "
                           "temperature. Off by default."));
    rbgBox->Add(m_enable, 0, wxALL, 6);

    wxFlexGridSizer *grid = new wxFlexGridSizer(2, 6, 12);
    grid->AddGrowableCol(1, 1);

    auto addRow = [&](const wxString& label, wxWindow *ctrl, const wxString& tip)
    {
        wxStaticText *t = new wxStaticText(rbgBox->GetStaticBox(), wxID_ANY, label);
        grid->Add(t, 0, wxALIGN_CENTER_VERTICAL);
        grid->Add(ctrl, 0, wxEXPAND);
        if (!tip.empty())
        {
            t->SetToolTip(tip);
            ctrl->SetToolTip(tip);
        }
    };

    m_alpha = new wxSpinCtrlDouble(rbgBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                                   wxDefaultPosition, wxDefaultSize,
                                   wxSP_ARROW_KEYS, 0.002, 0.5, 0.04, 0.002);
    m_alpha->SetDigits(3);
    addRow(_("EMA alpha:"), m_alpha,
           _("Weight given to each new frame in the running background "
             "estimate. Smaller = smoother / slower to adapt. 0.04 ~ 25-frame "
             "time constant."));

    m_outlierSigma = new wxSpinCtrlDouble(rbgBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                                          wxDefaultPosition, wxDefaultSize,
                                          wxSP_ARROW_KEYS, 2.0, 10.0, 4.0, 0.25);
    m_outlierSigma->SetDigits(2);
    addRow(_("Outlier reject (sigma):"), m_outlierSigma,
           _("Samples further than N sigma from the running mean are not fed "
             "into the background model. Rejects cosmic rays, satellite trails "
             "and stars that slipped the mask."));

    m_maskRadius = new wxSpinCtrl(rbgBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                                  wxDefaultPosition, wxDefaultSize,
                                  wxSP_ARROW_KEYS, 1, 40, 5);
    addRow(_("Catalog star mask radius (px):"), m_maskRadius,
           _("When a plate solution is available, pixels inside a disk of this "
             "radius around each matched catalog star are excluded from the "
             "background model."));

    m_minSamples = new wxSpinCtrl(rbgBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                                  wxDefaultPosition, wxDefaultSize,
                                  wxSP_ARROW_KEYS, 1, 200, 8);
    addRow(_("Min frames before subtracting:"), m_minSamples,
           _("Number of frames that must be observed before the pipeline "
             "starts subtracting the model. Set to 0 to subtract from the "
             "first frame (not recommended)."));

    m_skipInitial = new wxSpinCtrl(rbgBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                                   wxDefaultPosition, wxDefaultSize,
                                   wxSP_ARROW_KEYS, 0, 30, 2);
    addRow(_("Skip initial frames:"), m_skipInitial,
           _("Discard this many frames at startup before feeding the "
             "background model. Lets the camera AGC settle so the model "
             "is not poisoned by an unrepresentative first frame."));

    m_warmup = new wxSpinCtrl(rbgBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                              wxDefaultPosition, wxDefaultSize,
                              wxSP_ARROW_KEYS, 1, 200, 16);
    addRow(_("Warmup frames:"), m_warmup,
           _("Number of initial frames during which the effective alpha "
             "decays from 1.0 down to the steady-state value, so the model "
             "converges quickly at startup."));

    m_starQuantile = new wxSpinCtrlDouble(rbgBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                                          wxDefaultPosition, wxDefaultSize,
                                          wxSP_ARROW_KEYS, 0.80, 0.9999, 0.995, 0.001);
    m_starQuantile->SetDigits(4);
    addRow(_("Bright-pixel reject quantile:"), m_starQuantile,
           _("Fallback gate used when no plate solution is available. Pixels "
             "brighter than this frame-level quantile are assumed to be stars "
             "and excluded from the background update."));

    rbgBox->Add(grid, 0, wxALL | wxEXPAND, 6);

    m_floorZero = new wxCheckBox(rbgBox->GetStaticBox(), wxID_ANY,
                                 _("Clamp result at zero (else apply small pedestal)"));
    m_floorZero->SetToolTip(_("When unchecked, the per-frame mean of the model is "
                              "added back after subtraction so global brightness "
                              "is preserved (true flat-field semantics)."));
    rbgBox->Add(m_floorZero, 0, wxALL, 6);

    m_persistModel = new wxCheckBox(rbgBox->GetStaticBox(), wxID_ANY,
                                    _("Persist model per camera (auto-save and reload)"));
    m_persistModel->SetToolTip(_("When enabled, the converged background model is "
                                 "saved under the camera's hardware id and reloaded "
                                 "automatically on the next session, skipping warmup. "
                                 "Stored as a multi-extension FITS file in the dark "
                                 "library directory. Requires a backend that exposes "
                                 "a hardware id (e.g. V4L2/UVC). Reset Model also "
                                 "deletes the saved file."));
    rbgBox->Add(m_persistModel, 0, wxALL, 6);

    m_diagnostics = new wxCheckBox(rbgBox->GetStaticBox(), wxID_ANY,
                                   _("Emit per-frame diagnostic lines to debug log"));
    m_diagnostics->SetToolTip(_("Writes a CSV-style 'NoiseDiag,frame=...' line per "
                                "frame to the PHD2 debug log. Useful for inspecting "
                                "before/after statistics, model coverage and mask "
                                "counts. Off by default - adds a couple of histogram "
                                "passes per frame when on."));
    rbgBox->Add(m_diagnostics, 0, wxALL, 6);

    wxBoxSizer *btnRow = new wxBoxSizer(wxHORIZONTAL);
    m_resetBtn = new wxButton(rbgBox->GetStaticBox(), wxID_ANY, _("Reset model"));
    m_resetBtn->SetToolTip(_("Discard the accumulated background model. The "
                             "next frame starts a fresh warmup."));
    m_resetBtn->Bind(wxEVT_BUTTON, &NoiseConfigPanel::OnReset, this);
    btnRow->Add(m_resetBtn, 0, wxRIGHT, 8);
    m_status = new wxStaticText(rbgBox->GetStaticBox(), wxID_ANY, wxEmptyString);
    btnRow->Add(m_status, 1, wxALIGN_CENTER_VERTICAL);
    rbgBox->Add(btnRow, 0, wxALL | wxEXPAND, 6);

    wxBoxSizer *top = new wxBoxSizer(wxVERTICAL);
    top->Add(rbgBox, 0, wxALL | wxEXPAND, 8);

    // ----- Hot-pixel rejection -----
    wxStaticBoxSizer *hpBox =
        new wxStaticBoxSizer(wxVERTICAL, this, _("Hot-pixel / cosmic-ray rejection"));

    m_hpEnable = new wxCheckBox(hpBox->GetStaticBox(), wxID_ANY,
                                _("Enable spatial 3x3 median hot-pixel filter"));
    m_hpEnable->SetToolTip(_("Replaces isolated bright pixels (cosmic rays, "
                             "transient hot pixels, satellite hits) with the "
                             "median of their 3x3 neighbourhood. Stateless and "
                             "active from frame one, so it complements the "
                             "rolling background model which needs warmup."));
    hpBox->Add(m_hpEnable, 0, wxALL, 6);

    wxFlexGridSizer *hpGrid = new wxFlexGridSizer(2, 6, 12);
    hpGrid->AddGrowableCol(1, 1);

    auto addHpRow = [&](const wxString& label, wxWindow *ctrl, const wxString& tip)
    {
        wxStaticText *t = new wxStaticText(hpBox->GetStaticBox(), wxID_ANY, label);
        hpGrid->Add(t, 0, wxALIGN_CENTER_VERTICAL);
        hpGrid->Add(ctrl, 0, wxEXPAND);
        if (!tip.empty())
        {
            t->SetToolTip(tip);
            ctrl->SetToolTip(tip);
        }
    };

    m_hpRatio = new wxSpinCtrlDouble(hpBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                                     wxDefaultPosition, wxDefaultSize,
                                     wxSP_ARROW_KEYS, 1.5, 20.0, 3.0, 0.25);
    m_hpRatio->SetDigits(2);
    addHpRow(_("Median ratio threshold:"), m_hpRatio,
             _("A pixel is replaced when it exceeds the local 3x3 median by "
               "this multiplicative factor. 3.0 = pixel is at least 3x brighter "
               "than its neighbours."));

    m_hpAbsOffset = new wxSpinCtrl(hpBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                                   wxDefaultPosition, wxDefaultSize,
                                   wxSP_ARROW_KEYS, 0, 16383, 200);
    addHpRow(_("Absolute offset (ADU):"), m_hpAbsOffset,
             _("In addition to the ratio test, the centre must exceed the "
               "median by at least this many ADU (16-bit scale). Prevents "
               "over-firing in dark regions where the ratio is unstable."));

    m_hpMaxRepl = new wxSpinCtrl(hpBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                                 wxDefaultPosition, wxDefaultSize,
                                 wxSP_ARROW_KEYS, 0, 200000, 5000);
    addHpRow(_("Max replacements / frame:"), m_hpMaxRepl,
             _("If more pixels than this would be flagged, the filter aborts "
               "for that frame and leaves it untouched. Guards against "
               "wholesale corruption from cloud edges, dew, or focuser drift. "
               "Set to 0 to disable the cap."));

    hpBox->Add(hpGrid, 0, wxALL | wxEXPAND, 6);

    m_hpDiagnostics = new wxCheckBox(hpBox->GetStaticBox(), wxID_ANY,
                                     _("Emit per-frame diagnostic lines to debug log"));
    m_hpDiagnostics->SetToolTip(_("Writes a line per frame indicating how many "
                                  "pixels were replaced or whether the cap was hit."));
    hpBox->Add(m_hpDiagnostics, 0, wxALL, 6);

    top->Add(hpBox, 0, wxALL | wxEXPAND, 8);

    // ----- Dead-zone mask -----
    wxStaticBoxSizer *dzBox =
        new wxStaticBoxSizer(wxVERTICAL, this, _("Dead zones (extended dark calibration)"));

    m_dzEnable = new wxCheckBox(dzBox->GetStaticBox(), wxID_ANY,
                                _("Enable dead-zone masking"));
    m_dzEnable->SetToolTip(_("Suppresses output from regions that an extended "
                             "dark-frame calibration has identified as too "
                             "noisy to use (RTS pixels, amp glow, edge "
                             "banding). Star detection cannot pick spurious "
                             "peaks in masked regions. The mask is bound to "
                             "the camera's hardware id and capture mode."));
    dzBox->Add(m_dzEnable, 0, wxALL, 6);

    m_dzFillMedian = new wxCheckBox(dzBox->GetStaticBox(), wxID_ANY,
                                    _("Fill masked pixels with frame median (else zero)"));
    m_dzFillMedian->SetToolTip(_("When unchecked, masked pixels are set to 0 - "
                                 "the cleanest signal for star detection. "
                                 "When checked, the per-frame median of "
                                 "unmasked pixels is used so the visual "
                                 "appearance is less jarring."));
    dzBox->Add(m_dzFillMedian, 0, wxALL, 6);

    m_dzDiagnostics = new wxCheckBox(dzBox->GetStaticBox(), wxID_ANY,
                                     _("Emit per-frame diagnostic lines to debug log"));
    dzBox->Add(m_dzDiagnostics, 0, wxALL, 6);

    wxFlexGridSizer *dzGrid = new wxFlexGridSizer(2, 6, 12);
    dzGrid->AddGrowableCol(1, 1);

    auto addDzRow = [&](const wxString& label, wxWindow *ctrl, const wxString& tip)
    {
        wxStaticText *t = new wxStaticText(dzBox->GetStaticBox(), wxID_ANY, label);
        dzGrid->Add(t, 0, wxALIGN_CENTER_VERTICAL);
        dzGrid->Add(ctrl, 0, wxEXPAND);
        if (!tip.empty())
        {
            t->SetToolTip(tip);
            ctrl->SetToolTip(tip);
        }
    };

    m_dzNumFrames = new wxSpinCtrl(dzBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                                   wxDefaultPosition, wxDefaultSize,
                                   wxSP_ARROW_KEYS, 8, 500, 64);
    addDzRow(_("Calibration frames:"), m_dzNumFrames,
             _("Number of dark frames captured during the build process. "
               "More frames give a tighter per-pixel variance estimate; "
               "anything under 16 is barely better than the standard BPM "
               "calibration."));

    m_dzSigma = new wxSpinCtrlDouble(dzBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                                     wxDefaultPosition, wxDefaultSize,
                                     wxSP_ARROW_KEYS, 1.5, 20.0, 4.0, 0.25);
    m_dzSigma->SetDigits(2);
    addDzRow(_("Noise sigma multiplier:"), m_dzSigma,
             _("A pixel is flagged when its temporal stdev exceeds the "
               "median pixel stdev by this multiple. Lower = more pixels "
               "flagged."));

    m_dzMinRegion = new wxSpinCtrl(dzBox->GetStaticBox(), wxID_ANY, wxEmptyString,
                                   wxDefaultPosition, wxDefaultSize,
                                   wxSP_ARROW_KEYS, 1, 100, 4);
    addDzRow(_("Min region size (px):"), m_dzMinRegion,
             _("Connected groups of flagged pixels smaller than this are "
               "discarded - they belong in the bad-pixel map, not the "
               "dead-zone map."));

    dzBox->Add(dzGrid, 0, wxALL | wxEXPAND, 6);

    wxBoxSizer *dzBtns = new wxBoxSizer(wxHORIZONTAL);
    m_dzBuildBtn = new wxButton(dzBox->GetStaticBox(), wxID_ANY,
                                _("Build dead-zone map..."));
    m_dzBuildBtn->SetToolTip(_("Captures the configured number of dark "
                               "frames synchronously, computes per-pixel "
                               "temporal variance, and saves a mask bound "
                               "to this camera and mode. Cover the lens "
                               "or close the shutter before starting."));
    m_dzBuildBtn->Bind(wxEVT_BUTTON, &NoiseConfigPanel::OnDzBuild, this);
    dzBtns->Add(m_dzBuildBtn, 0, wxRIGHT, 8);

    m_dzClearBtn = new wxButton(dzBox->GetStaticBox(), wxID_ANY,
                                _("Clear saved map"));
    m_dzClearBtn->SetToolTip(_("Deletes the saved mask file for this "
                               "camera/mode."));
    m_dzClearBtn->Bind(wxEVT_BUTTON, &NoiseConfigPanel::OnDzClear, this);
    dzBtns->Add(m_dzClearBtn, 0, wxRIGHT, 8);

    m_dzViewBtn = new wxButton(dzBox->GetStaticBox(), wxID_ANY,
                               _("View mask..."));
    m_dzViewBtn->SetToolTip(_("Opens a viewer that shows the loaded "
                              "dead-zone mask for the current camera/mode "
                              "as a red overlay (red = masked, dark = ok). "
                              "The viewer can also export the mask as a "
                              "PNG."));
    m_dzViewBtn->Bind(wxEVT_BUTTON, &NoiseConfigPanel::OnDzView, this);
    dzBtns->Add(m_dzViewBtn, 0, wxRIGHT, 8);

    m_dzStatus = new wxStaticText(dzBox->GetStaticBox(), wxID_ANY, wxEmptyString);
    dzBtns->Add(m_dzStatus, 1, wxALIGN_CENTER_VERTICAL);

    dzBox->Add(dzBtns, 0, wxALL | wxEXPAND, 6);

    top->Add(dzBox, 0, wxALL | wxEXPAND, 8);

    // ----- Diagnostics bundle export -----
    wxBoxSizer *diagRow = new wxBoxSizer(wxHORIZONTAL);
    wxButton *diagBtn = new wxButton(this, wxID_ANY,
                                     _("Export diagnostics bundle..."));
    diagBtn->SetToolTip(_("Collects the rolling-background model, "
                          "dead-zone mask, current pipeline configuration "
                          "and a tail of the debug log into a single zip "
                          "file suitable for sharing with a developer or "
                          "feeding to an LLM for analysis."));
    diagBtn->Bind(wxEVT_BUTTON, &NoiseConfigPanel::OnExportDiagBundle, this);
    diagRow->Add(diagBtn, 0, wxRIGHT, 8);
    diagRow->AddStretchSpacer();
    top->Add(diagRow, 0, wxALL | wxEXPAND, 8);

    // Info blurb for future stages / auto-tune hook
    wxStaticText *info = new wxStaticText(this, wxID_ANY,
        _("Additional noise-reduction stages and camera auto-tuning will "
          "appear on this page as they are implemented."));
    info->Wrap(520);
    top->Add(info, 0, wxALL, 8);

    SetSizer(top);
}

void NoiseConfigPanel::LoadValues()
{
    RollingBackgroundStage *s = FindRolling();
    if (!s)
    {
        if (m_enable) m_enable->Enable(false);
        return;
    }

    m_enable->SetValue(s->IsEnabled());
    m_alpha->SetValue(s->GetAlpha());
    m_outlierSigma->SetValue(s->GetOutlierSigma());
    m_maskRadius->SetValue(s->GetMaskRadius());
    m_minSamples->SetValue(s->GetMinSamples());
    m_skipInitial->SetValue(s->GetSkipInitialFrames());
    m_warmup->SetValue(s->GetWarmupSamples());
    m_starQuantile->SetValue(s->GetStarQuantile());
    m_floorZero->SetValue(s->GetFloorAtZero());
    m_persistModel->SetValue(s->GetPersistModel());
    m_diagnostics->SetValue(s->GetDiagnostics());

    if (HotPixelStage *hp = FindHotPixel())
    {
        m_hpEnable->SetValue(hp->IsEnabled());
        m_hpRatio->SetValue(hp->GetRatio());
        m_hpAbsOffset->SetValue(hp->GetAbsOffset());
        m_hpMaxRepl->SetValue(hp->GetMaxReplacements());
        m_hpDiagnostics->SetValue(hp->GetDiagnostics());
    }
    else if (m_hpEnable)
    {
        m_hpEnable->Enable(false);
    }

    if (NoiseStageDeadZone *dz = FindDeadZone())
    {
        m_dzEnable->SetValue(dz->IsEnabled());
        m_dzFillMedian->SetValue(dz->GetFillWithMedian());
        m_dzDiagnostics->SetValue(dz->GetDiagnostics());
    }
    else if (m_dzEnable)
    {
        m_dzEnable->Enable(false);
        m_dzBuildBtn->Enable(false);
        m_dzClearBtn->Enable(false);
        if (m_dzViewBtn) m_dzViewBtn->Enable(false);
    }

    UpdateStatus();
    UpdateDzStatus();
}

void NoiseConfigPanel::UnloadValues()
{
    RollingBackgroundStage *s = FindRolling();
    if (!s)
        return;

    s->SetEnabled(m_enable->GetValue());
    s->SetAlpha((float) m_alpha->GetValue());
    s->SetOutlierSigma((float) m_outlierSigma->GetValue());
    s->SetMaskRadius(m_maskRadius->GetValue());
    s->SetMinSamples(m_minSamples->GetValue());
    s->SetSkipInitialFrames(m_skipInitial->GetValue());
    s->SetPersistModel(m_persistModel->GetValue());
    s->SetDiagnostics(m_diagnostics->GetValue());
    s->SetWarmupSamples(m_warmup->GetValue());
    s->SetStarQuantile((float) m_starQuantile->GetValue());
    s->SetFloorAtZero(m_floorZero->GetValue());

    // Persist via the stage so per-camera overrides are honoured.
    s->SaveConfig();

    if (HotPixelStage *hp = FindHotPixel())
    {
        hp->SetEnabled(m_hpEnable->GetValue());
        hp->SetRatio((float) m_hpRatio->GetValue());
        hp->SetAbsOffset(m_hpAbsOffset->GetValue());
        hp->SetMaxReplacements(m_hpMaxRepl->GetValue());
        hp->SetDiagnostics(m_hpDiagnostics->GetValue());
        hp->SaveConfig();
    }

    if (NoiseStageDeadZone *dz = FindDeadZone())
    {
        dz->SetEnabled(m_dzEnable->GetValue());
        dz->SetFillWithMedian(m_dzFillMedian->GetValue());
        dz->SetDiagnostics(m_dzDiagnostics->GetValue());
        dz->SaveConfig();
    }
}

void NoiseConfigPanel::OnReset(wxCommandEvent& /*evt*/)
{
    RollingBackgroundStage *s = FindRolling();
    if (s)
        s->Reset();
    UpdateStatus();
}

void NoiseConfigPanel::UpdateDzStatus()
{
    if (!m_dzStatus)
        return;
    NoiseStageDeadZone *dz = FindDeadZone();
    if (!dz)
    {
        m_dzStatus->SetLabel(_("Dead-zone stage not available"));
        return;
    }
    const wxString tag = NoiseStageDeadZone::CurrentCameraTag();
    if (tag.empty())
    {
        m_dzStatus->SetLabel(_("Camera not connected - cannot bind a mask"));
        return;
    }
    const DeadZoneStats s = dz->CurrentStats();
    if (s.maskedPixels == 0)
    {
        m_dzStatus->SetLabel(_("No mask loaded for this camera/mode"));
        return;
    }
    m_dzStatus->SetLabel(wxString::Format(
        _("Mask: %zu px in %zu regions (%dx%d)"),
        s.maskedPixels, s.regionCount, s.width, s.height));
}

// ---------------------------------------------------------------------------
// Dead-zone mask viewer dialog
// ---------------------------------------------------------------------------
//
// Lightweight modal that shows the current mask as a coloured image
// inside a scrolled window. Three zoom presets so the user can sanity-
// check both global region distribution (fit-to-window) and individual
// pixel-cluster shapes (1:1).
//
namespace
{

class DeadZoneViewDlg : public wxDialog
{
    wxImage          m_image;       // RGB rendering of the mask, 1:1 scale
    wxScrolledWindow *m_scroller;
    wxStaticBitmap   *m_bitmap;
    int              m_zoomIdx;     // 0 = fit, 1 = 100%, 2 = 200%

    void Rerender()
    {
        if (!m_image.IsOk())
            return;

        wxSize target;
        if (m_zoomIdx == 0)
        {
            // Fit longest side into 80% of the screen.
            wxSize screen = wxGetDisplaySize();
            const int maxW = (int) (screen.x * 0.80);
            const int maxH = (int) (screen.y * 0.70);
            const double sx = (double) maxW / m_image.GetWidth();
            const double sy = (double) maxH / m_image.GetHeight();
            const double s  = std::min(1.0, std::min(sx, sy));
            target = wxSize((int) (m_image.GetWidth()  * s),
                            (int) (m_image.GetHeight() * s));
        }
        else
        {
            const int factor = (m_zoomIdx == 2) ? 2 : 1;
            target = wxSize(m_image.GetWidth()  * factor,
                            m_image.GetHeight() * factor);
        }

        wxImage scaled = (target == m_image.GetSize())
                       ? m_image
                       : m_image.Scale(target.x, target.y, wxIMAGE_QUALITY_NEAREST);
        m_bitmap->SetBitmap(wxBitmap(scaled));
        m_scroller->SetScrollbars(20, 20, target.x / 20 + 1, target.y / 20 + 1);
        m_scroller->FitInside();
        Layout();
    }

public:
    DeadZoneViewDlg(wxWindow *parent, wxImage img, const wxString& subtitle)
        : wxDialog(parent, wxID_ANY, _("Dead-zone mask"),
                   wxDefaultPosition, wxDefaultSize,
                   wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER),
          m_image(std::move(img)),
          m_scroller(nullptr),
          m_bitmap(nullptr),
          m_zoomIdx(0)
    {
        wxBoxSizer *top = new wxBoxSizer(wxVERTICAL);

        if (!subtitle.empty())
            top->Add(new wxStaticText(this, wxID_ANY, subtitle),
                     0, wxALL, 8);

        m_scroller = new wxScrolledWindow(this, wxID_ANY,
                                          wxDefaultPosition, wxSize(640, 480),
                                          wxHSCROLL | wxVSCROLL);
        m_bitmap = new wxStaticBitmap(m_scroller, wxID_ANY, wxNullBitmap);
        wxBoxSizer *sc = new wxBoxSizer(wxVERTICAL);
        sc->Add(m_bitmap, 0);
        m_scroller->SetSizer(sc);
        top->Add(m_scroller, 1, wxEXPAND | wxALL, 8);

        wxBoxSizer *btnRow = new wxBoxSizer(wxHORIZONTAL);

        wxArrayString zooms;
        zooms.Add(_("Fit window"));
        zooms.Add(_("100% (1:1)"));
        zooms.Add(_("200%"));
        wxChoice *zoomChoice = new wxChoice(this, wxID_ANY,
                                            wxDefaultPosition, wxDefaultSize,
                                            zooms);
        zoomChoice->SetSelection(0);
        zoomChoice->Bind(wxEVT_CHOICE,
            [this, zoomChoice](wxCommandEvent&) {
                m_zoomIdx = zoomChoice->GetSelection();
                Rerender();
            });
        btnRow->Add(new wxStaticText(this, wxID_ANY, _("Zoom:")),
                    0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
        btnRow->Add(zoomChoice, 0, wxRIGHT, 16);

        wxButton *saveBtn = new wxButton(this, wxID_ANY, _("Save PNG..."));
        saveBtn->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
            wxFileDialog fd(this, _("Save dead-zone mask as PNG"),
                            wxEmptyString, wxT("dead_zone_mask.png"),
                            _("PNG image (*.png)|*.png"),
                            wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
            if (fd.ShowModal() == wxID_OK)
                m_image.SaveFile(fd.GetPath(), wxBITMAP_TYPE_PNG);
        });
        btnRow->Add(saveBtn, 0, wxRIGHT, 8);

        wxButton *closeBtn = new wxButton(this, wxID_OK, _("Close"));
        btnRow->AddStretchSpacer();
        btnRow->Add(closeBtn, 0);

        top->Add(btnRow, 0, wxEXPAND | wxALL, 8);

        SetSizerAndFit(top);
        // Don't Fit() against the full image - scrolling handles oversize.
        SetClientSize(wxSize(720, 560));
        Centre(wxBOTH);

        Rerender();
    }
};

} // namespace

void NoiseConfigPanel::OnDzView(wxCommandEvent& /*evt*/)
{
    // Always render from the on-disk file: that's the canonical source
    // the stage itself loads from, and it works whether or not the
    // pipeline has run yet this session.
    const wxString tag = NoiseStageDeadZone::CurrentCameraTag();
    if (tag.empty())
    {
        wxMessageBox(_("Connect the camera whose mask you want to view."),
                     _("No camera"), wxICON_INFORMATION, this);
        return;
    }

    DeadZoneMap m;
    if (!m.LoadFromDisk(tag))
    {
        wxMessageBox(_("No saved dead-zone mask for this camera/mode. "
                       "Build one first."),
                     _("Nothing to view"),
                     wxICON_INFORMATION, this);
        return;
    }

    wxImage img = m.RenderRGB();
    if (!img.IsOk())
    {
        wxMessageBox(_("Unable to render the mask."),
                     _("Render error"), wxICON_ERROR, this);
        return;
    }

    const DeadZoneStats st = m.Stats();
    const wxString subtitle = wxString::Format(
        _("Camera tag: %s   |   %zu masked px in %zu regions   |   %d \u00D7 %d"),
        tag, st.maskedPixels, st.regionCount, st.width, st.height);

    DeadZoneViewDlg dlg(this, std::move(img), subtitle);
    dlg.ShowModal();
}

void NoiseConfigPanel::OnDzClear(wxCommandEvent& /*evt*/)
{
    NoiseStageDeadZone *dz = FindDeadZone();
    if (!dz)
        return;
    if (wxMessageBox(_("Delete the saved dead-zone mask for the current "
                       "camera and mode? The next session will start with "
                       "no mask until you build a new one."),
                     _("Clear dead-zone map"),
                     wxYES_NO | wxICON_QUESTION, this) != wxYES)
        return;
    dz->ClearCurrent();
    UpdateDzStatus();
}

void NoiseConfigPanel::OnDzBuild(wxCommandEvent& /*evt*/)
{
    NoiseStageDeadZone *dz = FindDeadZone();
    if (!dz)
        return;

    if (!pCamera || !pCamera->Connected)
    {
        wxMessageBox(_("Connect a camera before building a dead-zone map."),
                     _("Camera not connected"), wxICON_ERROR, this);
        return;
    }

    const wxString tag = NoiseStageDeadZone::CurrentCameraTag();
    if (tag.empty())
    {
        wxMessageBox(_("This camera has no stable hardware id, so a "
                       "dead-zone mask cannot be saved against it."),
                     _("No hardware id"), wxICON_ERROR, this);
        return;
    }

    const int numFrames = m_dzNumFrames->GetValue();
    const int expMs = (pFrame ? pFrame->RequestedExposureDuration() : 1000);
    if (expMs <= 0)
    {
        wxMessageBox(_("Set a positive exposure on the main window before "
                       "building a dead-zone map."),
                     _("Invalid exposure"), wxICON_ERROR, this);
        return;
    }

    if (wxMessageBox(wxString::Format(
                _("This will capture %d dark frames at %.2f s. "
                  "Cover the lens or close the shutter, then click OK."),
                numFrames, expMs / 1000.0),
            _("Build dead-zone map"),
            wxOK | wxCANCEL | wxICON_INFORMATION, this) != wxOK)
        return;

    DeadZoneBuilder builder;
    builder.Begin(pCamera->FrameSize.GetWidth(), pCamera->FrameSize.GetHeight());

    wxProgressDialog progress(_("Building dead-zone map"),
                              _("Capturing calibration frames..."),
                              numFrames, this,
                              wxPD_CAN_ABORT | wxPD_AUTO_HIDE | wxPD_APP_MODAL);

    pCamera->InitCapture();
    bool aborted = false;
    int captured = 0;
    for (int i = 1; i <= numFrames; ++i)
    {
        if (!progress.Update(i - 1,
                             wxString::Format(_("Capturing frame %d / %d"),
                                              i, numFrames)))
        {
            aborted = true;
            break;
        }

        usImage frame;
        CaptureParams cp;
        cp.duration = expMs;
        cp.hwBinning = pCamera->HwBinning;
        cp.swBinning = 1;
        cp.bpp = pCamera->BitsPerPixel();
        cp.gain = pCamera->GuideCameraGain;
        cp.captureOptions = CAPTURE_DARK;

        if (GuideCamera::Capture(pCamera, frame, cp))
        {
            wxMessageBox(_("Frame capture failed - aborting build."),
                         _("Capture error"), wxICON_ERROR, this);
            aborted = true;
            break;
        }

        builder.AddFrame(frame);
        ++captured;
        wxYield();
    }
    progress.Update(numFrames);

    if (aborted || captured < 2)
    {
        wxMessageBox(_("Build aborted; no mask was saved."),
                     _("Aborted"), wxICON_INFORMATION, this);
        return;
    }

    DeadZoneBuilder::Params params;
    params.sigmaMultiplier = (float) m_dzSigma->GetValue();
    params.minRegionPx     = m_dzMinRegion->GetValue();
    // Other params stay at defaults; advanced users can edit the FITS
    // by hand or rebuild with adjusted code.

    DeadZoneMap map;
    if (!builder.Finalize(params, &map))
    {
        wxMessageBox(_("Failed to finalize dead-zone mask."),
                     _("Build error"), wxICON_ERROR, this);
        return;
    }
    if (!dz->AdoptAndSave(std::move(map), true))
    {
        wxMessageBox(_("Mask was built but could not be saved to disk."),
                     _("Save error"), wxICON_WARNING, this);
    }
    UpdateDzStatus();

    const DeadZoneStats s = dz->CurrentStats();
    wxMessageBox(wxString::Format(
                     _("Dead-zone build complete.\n\n"
                       "Captured frames: %d\n"
                       "Masked pixels: %zu\n"
                       "Connected regions: %zu\n\n"
                       "Saved to: %s"),
                     captured, s.maskedPixels, s.regionCount,
                     NoiseStageDeadZone::CurrentMapPath()),
                 _("Dead-zone build"), wxICON_INFORMATION, this);
}

void NoiseConfigPanel::UpdateStatus()
{
    RollingBackgroundStage *s = FindRolling();
    if (!s || !m_status)
        return;
    const wxSize sz = s->ModelSize();
    const int fc = s->FrameCount();
    wxString txt;
    if (sz.GetWidth() == 0)
        txt = _("Model: not yet initialized");
    else
        txt = wxString::Format(_("Model: %d x %d, %d frames observed"),
                               sz.GetWidth(), sz.GetHeight(), fc);
    m_status->SetLabel(txt);
}

// ---------------------------------------------------------------------------
// Diagnostics bundle export
// ---------------------------------------------------------------------------
//
// Builds a single zip archive containing everything needed to reason
// about the noise pipeline's current behaviour:
//
//   noise_diag.txt           - human-readable summary of camera, sensor
//                              and per-stage configuration / stats.
//   rolling_bg_model.fit     - copy of the persisted EMA model (if any).
//   dead_zone_mask.fit       - copy of the persisted mask (if any).
//   dead_zone_mask.png       - rendered RGB visualisation of the mask.
//   phd2_debug_tail.log      - last 200 KB of the current debug log.
//
// Designed for "share this with a developer / feed to an LLM" workflows.
//
namespace
{

// Append a buffer as a zip entry.
static bool ZipAddBuffer(wxZipOutputStream& zip, const wxString& name,
                         const void *data, size_t len, const wxDateTime& dt)
{
    if (!zip.PutNextEntry(name, dt))
        return false;
    if (len > 0)
        zip.Write(data, len);
    return zip.LastWrite() == len;
}

// Append an existing on-disk file as a zip entry. Silently skipped when
// the file is missing (the bundle is best-effort).
static bool ZipAddFile(wxZipOutputStream& zip, const wxString& diskPath,
                       const wxString& entryName)
{
    if (diskPath.empty() || !wxFileExists(diskPath))
        return false;
    wxFFileInputStream in(diskPath);
    if (!in.IsOk())
        return false;
    wxDateTime mtime;
    {
        wxFileName fn(diskPath);
        fn.GetTimes(nullptr, &mtime, nullptr);
    }
    if (!zip.PutNextEntry(entryName, mtime.IsValid() ? mtime : wxDateTime::Now()))
        return false;
    zip.Write(in);
    return zip.IsOk();
}

// Append a tail (last `tailBytes` bytes) of a file as a zip entry.
static bool ZipAddFileTail(wxZipOutputStream& zip, const wxString& diskPath,
                           const wxString& entryName, wxFileOffset tailBytes)
{
    if (diskPath.empty() || !wxFileExists(diskPath))
        return false;
    wxFFileInputStream in(diskPath);
    if (!in.IsOk())
        return false;
    const wxFileOffset total = in.GetLength();
    if (total > tailBytes)
        in.SeekI(total - tailBytes);
    wxDateTime mtime;
    {
        wxFileName fn(diskPath);
        fn.GetTimes(nullptr, &mtime, nullptr);
    }
    if (!zip.PutNextEntry(entryName, mtime.IsValid() ? mtime : wxDateTime::Now()))
        return false;
    zip.Write(in);
    return zip.IsOk();
}

// Locate the most recently modified PHD2 debug log in the log directory.
// Returns empty when none found.
static wxString FindLatestDebugLog()
{
    const wxString dir = Debug.GetLogDir();
    if (dir.empty() || !wxDirExists(dir))
        return wxString();

    wxArrayString files;
    wxDir::GetAllFiles(dir, &files, wxT("PHD2_DebugLog*.txt"), wxDIR_FILES);

    wxString best;
    wxDateTime bestMtime;
    for (size_t i = 0; i < files.size(); ++i)
    {
        wxFileName fn(files[i]);
        wxDateTime mt;
        if (!fn.GetTimes(nullptr, &mt, nullptr))
            continue;
        if (best.empty() || mt.IsLaterThan(bestMtime))
        {
            best = files[i];
            bestMtime = mt;
        }
    }
    return best;
}

static wxString FormatYesNo(bool v)
{
    return v ? wxT("yes") : wxT("no");
}

static wxString BuildDiagText(const wxString& camTag)
{
    wxString out;
    out.reserve(8192);

    out += wxT("PHD2 noise pipeline diagnostics bundle\n");
    out += wxT("=======================================\n\n");
    out += wxString::Format(wxT("Generated: %s\n"),
                            wxDateTime::Now().FormatISOCombined(' '));
    out += wxString::Format(wxT("PHD2 version: %s\n"), wxString(FULLVER));
    out += wxT("\n");

    // Camera
    out += wxT("[Camera]\n");
    if (pCamera)
    {
        out += wxString::Format(wxT("Name:        %s\n"), pCamera->Name);
        out += wxString::Format(wxT("Connected:   %s\n"),
                                FormatYesNo(pCamera->Connected));
        out += wxString::Format(wxT("HardwareId:  %s\n"),
                                pCamera->GetHardwareId());
        out += wxString::Format(wxT("CameraTag:   %s\n"), camTag);
        out += wxString::Format(wxT("FrameSize:   %d x %d\n"),
                                pCamera->FrameSize.GetWidth(),
                                pCamera->FrameSize.GetHeight());
        out += wxString::Format(wxT("HwBinning:   %d\n"), (int) pCamera->HwBinning);
        out += wxString::Format(wxT("Gain:        %d\n"), pCamera->GuideCameraGain);
        out += wxString::Format(wxT("Sensor:      %s\n"),
                                pCamera->GetSelectedSensorName());
    }
    else
    {
        out += wxT("(no camera object)\n");
    }
    out += wxT("\n");

    // Pipeline
    out += wxT("[Pipeline]\n");
    if (!pNoise)
    {
        out += wxT("(noise pipeline not initialised)\n\n");
        return out;
    }
    out += wxString::Format(wxT("Stage count: %zu\n\n"), pNoise->StageCount());

    for (size_t i = 0; i < pNoise->StageCount(); ++i)
    {
        NoiseReductionStage *st = pNoise->GetStage(i);
        if (!st)
            continue;
        out += wxString::Format(wxT("Stage %zu: %s (enabled=%s)\n"),
                                i, st->Name(),
                                FormatYesNo(st->IsEnabled()));

        if (RollingBackgroundStage *r = dynamic_cast<RollingBackgroundStage *>(st))
        {
            const wxSize sz = r->ModelSize();
            out += wxString::Format(wxT("  alpha=%.4f outlierSigma=%.2f maskRadius=%d\n"),
                                    r->GetAlpha(), r->GetOutlierSigma(),
                                    r->GetMaskRadius());
            out += wxString::Format(wxT("  minSamples=%d skipInitial=%d warmup=%d\n"),
                                    r->GetMinSamples(),
                                    r->GetSkipInitialFrames(),
                                    r->GetWarmupSamples());
            out += wxString::Format(wxT("  starQuantile=%.3f floorAtZero=%s persistModel=%s\n"),
                                    r->GetStarQuantile(),
                                    FormatYesNo(r->GetFloorAtZero()),
                                    FormatYesNo(r->GetPersistModel()));
            out += wxString::Format(wxT("  diagnostics=%s\n"),
                                    FormatYesNo(r->GetDiagnostics()));
            out += wxString::Format(wxT("  modelSize=%d x %d frameCount=%d\n"),
                                    sz.GetWidth(), sz.GetHeight(),
                                    r->FrameCount());
        }
        else if (HotPixelStage *h = dynamic_cast<HotPixelStage *>(st))
        {
            out += wxString::Format(wxT("  ratio=%.2f absOffset=%d maxReplacements=%d\n"),
                                    h->GetRatio(), h->GetAbsOffset(),
                                    h->GetMaxReplacements());
            out += wxString::Format(wxT("  diagnostics=%s\n"),
                                    FormatYesNo(h->GetDiagnostics()));
        }
        else if (NoiseStageDeadZone *d = dynamic_cast<NoiseStageDeadZone *>(st))
        {
            const DeadZoneStats s = d->CurrentStats();
            out += wxString::Format(wxT("  fillWithMedian=%s diagnostics=%s\n"),
                                    FormatYesNo(d->GetFillWithMedian()),
                                    FormatYesNo(d->GetDiagnostics()));
            out += wxString::Format(wxT("  mask: %zu px in %zu regions (%d x %d)\n"),
                                    s.maskedPixels, s.regionCount,
                                    s.width, s.height);
            out += wxString::Format(wxT("  mapPath: %s\n"),
                                    NoiseStageDeadZone::CurrentMapPath());
        }
        out += wxT("\n");
    }

    return out;
}

} // namespace

void NoiseConfigPanel::OnExportDiagBundle(wxCommandEvent& /*evt*/)
{
    const wxString camTag = pCamera ? RollingBackgroundStage::CurrentCameraTag()
                                    : wxString();
    const wxString stamp =
        wxDateTime::Now().Format(wxT("%Y%m%dT%H%M%S"));
    wxString defaultName = wxT("phd2_noise_diag_");
    if (!camTag.empty())
        defaultName += camTag + wxT("_");
    defaultName += stamp + wxT(".zip");

    wxFileDialog fd(this, _("Save noise diagnostics bundle"),
                    wxEmptyString, defaultName,
                    _("Zip archive (*.zip)|*.zip"),
                    wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (fd.ShowModal() != wxID_OK)
        return;

    const wxString outPath = fd.GetPath();

    wxFFileOutputStream out(outPath);
    if (!out.IsOk())
    {
        wxMessageBox(wxString::Format(_("Could not open %s for writing."),
                                      outPath),
                     _("Export failed"), wxICON_ERROR, this);
        return;
    }
    wxZipOutputStream zip(out);

    const wxDateTime now = wxDateTime::Now();

    // 1. noise_diag.txt
    {
        const wxString text = BuildDiagText(camTag);
        const wxScopedCharBuffer utf8 = text.utf8_str();
        ZipAddBuffer(zip, wxT("noise_diag.txt"),
                     utf8.data(), utf8.length(), now);
    }

    // 2. Rolling background model (if persisted)
    if (!camTag.empty())
    {
        const wxString rbgPath =
            RollingBackgroundStage::ModelFilePath(camTag);
        ZipAddFile(zip, rbgPath, wxT("rolling_bg_model.fit"));
    }

    // 3. Dead-zone mask FITS + rendered PNG (if persisted)
    if (!camTag.empty())
    {
        const wxString dzPath = DeadZoneMap::MapFilePath(camTag);
        ZipAddFile(zip, dzPath, wxT("dead_zone_mask.fit"));

        DeadZoneMap m;
        if (m.LoadFromDisk(camTag))
        {
            wxImage img = m.RenderRGB();
            if (img.IsOk())
            {
                wxMemoryOutputStream pngBuf;
                if (img.SaveFile(pngBuf, wxBITMAP_TYPE_PNG))
                {
                    const size_t n = pngBuf.GetSize();
                    std::vector<unsigned char> raw(n);
                    pngBuf.CopyTo(raw.data(), n);
                    ZipAddBuffer(zip, wxT("dead_zone_mask.png"),
                                 raw.data(), n, now);
                }
            }
        }
    }

    // 4. Tail of debug log (last ~200 KB)
    const wxString debugLog = FindLatestDebugLog();
    if (!debugLog.empty())
    {
        ZipAddFileTail(zip, debugLog, wxT("phd2_debug_tail.log"),
                       200 * 1024);
    }

    if (!zip.Close() || !out.Close())
    {
        wxMessageBox(_("Failed to finalize the zip archive."),
                     _("Export failed"), wxICON_ERROR, this);
        return;
    }

    wxMessageBox(wxString::Format(
                     _("Diagnostics bundle written to:\n\n%s"),
                     outPath),
                 _("Export complete"), wxICON_INFORMATION, this);
}
