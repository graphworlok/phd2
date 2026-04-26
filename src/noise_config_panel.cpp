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

#include <wx/checkbox.h>
#include <wx/spinctrl.h>
#include <wx/button.h>
#include <wx/stattext.h>
#include <wx/statbox.h>
#include <wx/sizer.h>

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

    UpdateStatus();
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
}

void NoiseConfigPanel::OnReset(wxCommandEvent& /*evt*/)
{
    RollingBackgroundStage *s = FindRolling();
    if (s)
        s->Reset();
    UpdateStatus();
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
