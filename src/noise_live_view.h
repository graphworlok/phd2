/*
 *  noise_live_view.h
 *  PHD Guiding
 *
 *  Modeless dialog that shows the most recent (raw, denoised) frame
 *  pair pulled from the noise pipeline tap, plus an optional
 *  difference-amplified panel. Designed to stay open while guiding so
 *  the user can immediately see the effect of changing a stage's
 *  parameters.
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#ifndef NOISE_LIVE_VIEW_H_INCLUDED
#define NOISE_LIVE_VIEW_H_INCLUDED

#include <wx/dialog.h>
#include <wx/timer.h>

class wxStaticBitmap;
class wxStaticText;
class wxChoice;
class wxCheckBox;

class NoiseLiveViewDlg : public wxDialog
{
public:
    NoiseLiveViewDlg(wxWindow *parent);
    ~NoiseLiveViewDlg() override;

    // Override Show to gate tap activation and the refresh timer on
    // visibility, so the dialog can be hidden and re-shown without
    // leaving the camera worker paying for an inactive viewer.
    bool Show(bool show = true) override;

private:
    enum StretchMode
    {
        STRETCH_AUTO_FRAME = 0,    // each frame stretched to its own min..max
        STRETCH_AUTO_99    = 1,    // 1..99 percentile of the raw frame
        STRETCH_FIXED_16B  = 2,    // full 16-bit dynamic range
    };

    void OnTimer(wxTimerEvent& evt);
    void OnClose(wxCloseEvent& evt);
    void OnStretchChange(wxCommandEvent& evt);
    void OnDiffToggle(wxCommandEvent& evt);

    wxStaticBitmap *m_rawBmp;
    wxStaticBitmap *m_denoisedBmp;
    wxStaticBitmap *m_diffBmp;
    wxStaticText   *m_rawStats;
    wxStaticText   *m_denoisedStats;
    wxStaticText   *m_diffStats;
    wxStaticText   *m_overall;
    wxChoice       *m_stretch;
    wxCheckBox     *m_showDiff;
    wxTimer         m_timer;

    unsigned    m_lastSeen      = 0;
    StretchMode m_stretchMode   = STRETCH_AUTO_FRAME;
    bool        m_diffEnabled   = true;
    int         m_displayMaxDim = 480;   // longest side, px
    bool        m_active        = false; // tap currently activated for this dlg

    wxDECLARE_EVENT_TABLE();
};

#endif // NOISE_LIVE_VIEW_H_INCLUDED
