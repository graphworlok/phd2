/*
 *  noise_config_panel.h
 *  PHD Guiding
 *
 *  Advanced-dialog ("Brain") page for the noise reduction pipeline.
 *  Presently exposes the settings of RollingBackgroundStage. Built as
 *  a self-contained panel so additional pipeline stages can be appended
 *  without entangling the BrainCtrlIdMap machinery.
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#ifndef NOISE_CONFIG_PANEL_H_INCLUDED
#define NOISE_CONFIG_PANEL_H_INCLUDED

#include <wx/panel.h>

class wxCheckBox;
class wxSpinCtrl;
class wxSpinCtrlDouble;
class wxButton;
class wxStaticText;

class NoiseConfigPanel : public wxPanel
{
    // Rolling background controls
    wxCheckBox       *m_enable        = nullptr;
    wxSpinCtrlDouble *m_alpha         = nullptr;
    wxSpinCtrlDouble *m_outlierSigma  = nullptr;
    wxSpinCtrl       *m_maskRadius    = nullptr;
    wxSpinCtrl       *m_minSamples    = nullptr;
    wxSpinCtrl       *m_skipInitial   = nullptr;
    wxSpinCtrl       *m_warmup        = nullptr;
    wxSpinCtrlDouble *m_starQuantile  = nullptr;
    wxCheckBox       *m_floorZero     = nullptr;
    wxCheckBox       *m_persistModel  = nullptr;
    wxCheckBox       *m_diagnostics   = nullptr;
    wxButton         *m_resetBtn      = nullptr;
    wxStaticText     *m_status        = nullptr;

    // Hot-pixel stage controls
    wxCheckBox       *m_hpEnable      = nullptr;
    wxSpinCtrlDouble *m_hpRatio       = nullptr;
    wxSpinCtrl       *m_hpAbsOffset   = nullptr;
    wxSpinCtrl       *m_hpMaxRepl     = nullptr;
    wxCheckBox       *m_hpDiagnostics = nullptr;

    void OnReset(wxCommandEvent& evt);
    void UpdateStatus();

public:
    NoiseConfigPanel(wxWindow *parent);

    // Called by AdvancedDialog on open / apply.
    void LoadValues();
    void UnloadValues();
};

#endif // NOISE_CONFIG_PANEL_H_INCLUDED
