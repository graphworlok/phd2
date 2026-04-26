/*
 *  noise_live_view.cpp
 *  PHD Guiding
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#include "phd.h"
#include "noise_live_view.h"
#include "noise_pipeline_tap.h"
#include "usImage.h"

#include <wx/sizer.h>
#include <wx/button.h>
#include <wx/statbmp.h>
#include <wx/stattext.h>
#include <wx/choice.h>
#include <wx/checkbox.h>
#include <wx/image.h>
#include <wx/bitmap.h>
#include <wx/dcclient.h>
#include <wx/dcmemory.h>
#include <wx/statline.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{

enum
{
    ID_LIVE_VIEW_TIMER = 13901,
    ID_LIVE_VIEW_STRETCH,
    ID_LIVE_VIEW_DIFF_TOGGLE,
};

// Compute black/white levels for a stretch mode.
struct LevelPair
{
    int blevel;
    int wlevel;
};

LevelPair ComputeLevels(const usImage& img, int mode)
{
    LevelPair r{ 0, 65535 };
    if (img.NPixels == 0 || !img.ImageData)
        return r;

    if (mode == 2 /* STRETCH_FIXED_16B */)
        return { 0, 65535 };

    if (mode == 0 /* STRETCH_AUTO_FRAME */)
    {
        unsigned short mn = img.ImageData[0];
        unsigned short mx = mn;
        for (unsigned int i = 1; i < img.NPixels; ++i)
        {
            unsigned short v = img.ImageData[i];
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        if (mx <= mn) mx = mn + 1;
        return { mn, mx };
    }

    // STRETCH_AUTO_99: histogram, take 1st and 99th percentile.
    std::vector<unsigned int> hist(65536, 0);
    for (unsigned int i = 0; i < img.NPixels; ++i)
        ++hist[img.ImageData[i]];

    const unsigned int total = img.NPixels;
    const unsigned int loCut = total / 100;
    const unsigned int hiCut = total - (total / 100);
    unsigned int acc = 0;
    int lo = 0, hi = 65535;
    for (int v = 0; v < 65536; ++v)
    {
        acc += hist[v];
        if (acc >= loCut) { lo = v; break; }
    }
    acc = 0;
    for (int v = 65535; v >= 0; --v)
    {
        acc += hist[v];
        if (acc >= total - hiCut) { hi = v; break; }
    }
    if (hi <= lo) hi = lo + 1;
    return { lo, hi };
}

// Render an image into an RGB wxImage at the requested longest-side
// size, using the supplied black/white levels.
wxImage RenderToFit(const usImage& img, const LevelPair& lev, int maxDim)
{
    if (img.NPixels == 0 || !img.ImageData)
        return wxImage();

    const int w = img.Size.GetWidth();
    const int h = img.Size.GetHeight();
    if (w <= 0 || h <= 0)
        return wxImage();

    // Build a 256-entry LUT from the supplied levels.
    unsigned char lut[65536];
    const double scale = 255.0 / std::max(1, lev.wlevel - lev.blevel);
    for (int v = 0; v < 65536; ++v)
    {
        int x = v - lev.blevel;
        if (x < 0) x = 0;
        double d = x * scale;
        if (d > 255.0) d = 255.0;
        lut[v] = (unsigned char) d;
    }

    wxImage out(w, h, false);
    unsigned char *dst = out.GetData();
    const unsigned short *src = img.ImageData;
    for (unsigned int i = 0; i < img.NPixels; ++i)
    {
        unsigned char d = lut[src[i]];
        *dst++ = d; *dst++ = d; *dst++ = d;
    }

    // Rescale so the longest side fits maxDim, preserving aspect.
    if (w > maxDim || h > maxDim)
    {
        const double s = (double) maxDim / std::max(w, h);
        out = out.Scale((int) (w * s), (int) (h * s),
                        wxIMAGE_QUALITY_NORMAL);
    }
    return out;
}

// Compute |a - b| into out. out is sized to match a; both inputs must
// match size, else returns false. Ranges of the result are returned in
// outMax for the caller to use as the white level for display.
bool ComputeAbsDiff(const usImage& a, const usImage& b, usImage& out,
                    unsigned short *outMax, double *outMean)
{
    if (a.Size != b.Size || a.NPixels != b.NPixels)
        return false;
    if (out.Init(a.Size))
        return false;

    unsigned short mx = 0;
    double sum = 0;
    for (unsigned int i = 0; i < a.NPixels; ++i)
    {
        int d = (int) a.ImageData[i] - (int) b.ImageData[i];
        if (d < 0) d = -d;
        unsigned short v = (unsigned short) d;
        out.ImageData[i] = v;
        if (v > mx) mx = v;
        sum += v;
    }
    if (outMax)  *outMax  = mx;
    if (outMean) *outMean = sum / a.NPixels;
    return true;
}

// Compute simple stats for the status line.
struct ImgStats
{
    int    minV;
    int    maxV;
    double mean;
};
ImgStats ComputeStats(const usImage& img)
{
    ImgStats r{ 0, 0, 0.0 };
    if (img.NPixels == 0 || !img.ImageData)
        return r;
    r.minV = img.ImageData[0];
    r.maxV = r.minV;
    double sum = 0;
    for (unsigned int i = 0; i < img.NPixels; ++i)
    {
        int v = img.ImageData[i];
        if (v < r.minV) r.minV = v;
        if (v > r.maxV) r.maxV = v;
        sum += v;
    }
    r.mean = sum / img.NPixels;
    return r;
}

} // namespace

wxBEGIN_EVENT_TABLE(NoiseLiveViewDlg, wxDialog)
    EVT_TIMER(ID_LIVE_VIEW_TIMER, NoiseLiveViewDlg::OnTimer)
    EVT_CLOSE(NoiseLiveViewDlg::OnClose)
    EVT_CHOICE(ID_LIVE_VIEW_STRETCH, NoiseLiveViewDlg::OnStretchChange)
    EVT_CHECKBOX(ID_LIVE_VIEW_DIFF_TOGGLE, NoiseLiveViewDlg::OnDiffToggle)
wxEND_EVENT_TABLE()

NoiseLiveViewDlg::NoiseLiveViewDlg(wxWindow *parent)
    : wxDialog(parent, wxID_ANY, _("Noise pipeline live view"),
               wxDefaultPosition, wxDefaultSize,
               wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER),
      m_rawBmp(nullptr), m_denoisedBmp(nullptr), m_diffBmp(nullptr),
      m_rawStats(nullptr), m_denoisedStats(nullptr), m_diffStats(nullptr),
      m_overall(nullptr), m_stretch(nullptr), m_showDiff(nullptr),
      m_timer(this, ID_LIVE_VIEW_TIMER)
{
    // Tap and timer are activated lazily in Show() so a hidden dialog
    // doesn't burden the camera worker.

    wxBoxSizer *top = new wxBoxSizer(wxVERTICAL);

    // Header: stretch and diff toggle.
    wxBoxSizer *hdr = new wxBoxSizer(wxHORIZONTAL);
    hdr->Add(new wxStaticText(this, wxID_ANY, _("Stretch:")),
             0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 6);
    wxArrayString stretches;
    stretches.Add(_("Per-frame min..max"));
    stretches.Add(_("1-99 percentile"));
    stretches.Add(_("Full 16-bit"));
    m_stretch = new wxChoice(this, ID_LIVE_VIEW_STRETCH,
                             wxDefaultPosition, wxDefaultSize, stretches);
    m_stretch->SetSelection(0);
    m_stretch->SetToolTip(_("Selects how the 16-bit frame data is mapped "
                            "to 8-bit display intensity. Per-frame "
                            "min..max maximises contrast but can amplify "
                            "noise; 1-99 percentile is more stable; full "
                            "16-bit shows the raw scaling guiding sees."));
    hdr->Add(m_stretch, 0, wxRIGHT, 12);

    m_showDiff = new wxCheckBox(this, ID_LIVE_VIEW_DIFF_TOGGLE,
                                _("Show |raw - denoised| difference"));
    m_showDiff->SetValue(m_diffEnabled);
    m_showDiff->SetToolTip(_("Adds a third panel showing the absolute "
                             "per-pixel difference, auto-stretched. Bright "
                             "spots indicate where the pipeline is "
                             "subtracting the most signal."));
    hdr->Add(m_showDiff, 0, wxALIGN_CENTER_VERTICAL);

    top->Add(hdr, 0, wxALL | wxEXPAND, 8);

    // Image panels.
    wxBoxSizer *imgRow = new wxBoxSizer(wxHORIZONTAL);

    auto buildPanel = [this](const wxString& title, wxStaticBitmap **bmpOut,
                             wxStaticText **statsOut)
    {
        wxBoxSizer *col = new wxBoxSizer(wxVERTICAL);
        col->Add(new wxStaticText(this, wxID_ANY, title),
                 0, wxALIGN_CENTER_HORIZONTAL | wxBOTTOM, 4);
        // Placeholder: a black image at the target display size so
        // the dialog is laid out correctly before the first frame.
        wxImage placeholder(m_displayMaxDim, m_displayMaxDim);
        placeholder.Clear();
        wxStaticBitmap *bmp = new wxStaticBitmap(
            this, wxID_ANY, wxBitmap(placeholder));
        col->Add(bmp, 0, wxALIGN_CENTER_HORIZONTAL);
        wxStaticText *stats = new wxStaticText(
            this, wxID_ANY, _("(no frame yet)"),
            wxDefaultPosition, wxDefaultSize, wxST_NO_AUTORESIZE);
        col->Add(stats, 0, wxTOP, 4);
        *bmpOut   = bmp;
        *statsOut = stats;
        return col;
    };

    imgRow->Add(buildPanel(_("Raw (post-dark, pre-pipeline)"),
                           &m_rawBmp, &m_rawStats),
                1, wxALL | wxEXPAND, 6);
    imgRow->Add(buildPanel(_("Denoised (post-pipeline)"),
                           &m_denoisedBmp, &m_denoisedStats),
                1, wxALL | wxEXPAND, 6);
    imgRow->Add(buildPanel(_("Difference"),
                           &m_diffBmp, &m_diffStats),
                1, wxALL | wxEXPAND, 6);

    top->Add(imgRow, 1, wxEXPAND);

    top->Add(new wxStaticLine(this), 0, wxEXPAND | wxALL, 6);

    m_overall = new wxStaticText(this, wxID_ANY,
        _("Waiting for frames... start a guide loop or single exposure."));
    top->Add(m_overall, 0, wxALL, 8);

    wxBoxSizer *btns = new wxBoxSizer(wxHORIZONTAL);
    btns->AddStretchSpacer();
    btns->Add(new wxButton(this, wxID_CLOSE, _("Close")), 0);
    top->Add(btns, 0, wxALL | wxEXPAND, 8);

    Bind(wxEVT_BUTTON,
         [this](wxCommandEvent&) { Close(); }, wxID_CLOSE);

    SetEscapeId(wxID_CLOSE);
    SetSizerAndFit(top);

    // Reasonable default size.
    SetClientSize(wxSize(1100, 620));
    Centre(wxBOTH);

    // Show diff panel by default (matches m_diffEnabled).
    if (!m_diffEnabled)
        m_diffBmp->Show(false);
}

NoiseLiveViewDlg::~NoiseLiveViewDlg()
{
    if (m_active)
    {
        if (m_timer.IsRunning())
            m_timer.Stop();
        NoisePipelineTap::Deactivate();
        m_active = false;
    }
}

bool NoiseLiveViewDlg::Show(bool show)
{
    const bool result = wxDialog::Show(show);
    if (show && !m_active)
    {
        NoisePipelineTap::Activate();
        // Poll ~5 Hz; the worker only updates the tap on each frame
        // so a faster timer would burn CPU on no-ops.
        m_timer.Start(200);
        m_active = true;
        m_lastSeen = 0;
    }
    else if (!show && m_active)
    {
        if (m_timer.IsRunning())
            m_timer.Stop();
        NoisePipelineTap::Deactivate();
        m_active = false;
    }
    return result;
}

void NoiseLiveViewDlg::OnClose(wxCloseEvent& /*evt*/)
{
    // Hide rather than destroy: the menu handler keeps a single
    // instance and just re-shows it. The instance is destroyed once
    // by MyFrame at app shutdown.
    Show(false);
}

void NoiseLiveViewDlg::OnStretchChange(wxCommandEvent& /*evt*/)
{
    m_stretchMode = (StretchMode) m_stretch->GetSelection();
    // Force the next timer tick to re-render even if no new frame
    // arrived: we want the new stretch reflected immediately.
    m_lastSeen = 0;
}

void NoiseLiveViewDlg::OnDiffToggle(wxCommandEvent& /*evt*/)
{
    m_diffEnabled = m_showDiff->GetValue();
    m_diffBmp->Show(m_diffEnabled);
    m_diffStats->Show(m_diffEnabled);
    Layout();
    m_lastSeen = 0;
}

void NoiseLiveViewDlg::OnTimer(wxTimerEvent& /*evt*/)
{
    usImage raw, denoised;
    if (!NoisePipelineTap::TryFetch(raw, denoised, m_lastSeen))
        return;

    const LevelPair lev = ComputeLevels(raw, m_stretchMode);

    wxImage rawImg = RenderToFit(raw, lev, m_displayMaxDim);
    wxImage denImg = RenderToFit(denoised, lev, m_displayMaxDim);

    if (rawImg.IsOk())
        m_rawBmp->SetBitmap(wxBitmap(rawImg));
    if (denImg.IsOk())
        m_denoisedBmp->SetBitmap(wxBitmap(denImg));

    const ImgStats rs = ComputeStats(raw);
    const ImgStats ds = ComputeStats(denoised);

    m_rawStats->SetLabel(wxString::Format(
        _("min=%d max=%d mean=%.1f"),
        rs.minV, rs.maxV, rs.mean));
    m_denoisedStats->SetLabel(wxString::Format(
        _("min=%d max=%d mean=%.1f"),
        ds.minV, ds.maxV, ds.mean));

    if (m_diffEnabled)
    {
        usImage diff;
        unsigned short diffMax = 0;
        double diffMean = 0;
        if (ComputeAbsDiff(raw, denoised, diff, &diffMax, &diffMean))
        {
            // Auto-stretch the diff against its own dynamic range. Add
            // a small floor so a fully-zero diff doesn't divide-by-one.
            LevelPair dlev{ 0, std::max<int>(1, diffMax) };
            wxImage diffImg = RenderToFit(diff, dlev, m_displayMaxDim);
            if (diffImg.IsOk())
                m_diffBmp->SetBitmap(wxBitmap(diffImg));
            m_diffStats->SetLabel(wxString::Format(
                _("max|d|=%u mean|d|=%.2f"),
                (unsigned) diffMax, diffMean));
        }
    }

    m_overall->SetLabel(wxString::Format(
        _("Frame %u  |  %d x %d  |  stretch %d-%d ADU"),
        m_lastSeen, raw.Size.GetWidth(), raw.Size.GetHeight(),
        lev.blevel, lev.wlevel));
}
