/*
 *  noise_pipeline.cpp
 *  PHD Guiding
 *
 *  Copyright (c) 2026 PHD2 Contributors - BSD licence.
 */

#include "phd.h"
#include "noise_pipeline.h"
#include "noise_stage_hot_pixel.h"
#include "noise_stage_rolling_bg.h"
#include "noise_stage_dead_zone.h"

NoisePipeline *pNoise = nullptr;

NoisePipeline::NoisePipeline() = default;
NoisePipeline::~NoisePipeline() = default;

void NoisePipeline::AddStage(std::unique_ptr<NoiseReductionStage> stage)
{
    m_stages.push_back(std::move(stage));
}

void NoisePipeline::Process(usImage& img, const PlateSolveResult *solve)
{
    for (auto& stage : m_stages)
    {
        if (!stage->IsEnabled())
            continue;
        // Observe must see the RAW frame; Apply mutates img in place.
        // Running Apply first made Observe update the model from the already-
        // subtracted residual, which collapses the model toward zero and
        // effectively turns subtraction into a no-op after a few frames.
        stage->Observe(img, solve);
        stage->Apply(img, solve);
    }
}

NoiseReductionStage *NoisePipeline::GetStage(size_t i) const
{
    return i < m_stages.size() ? m_stages[i].get() : nullptr;
}

NoiseReductionStage *NoisePipeline::FindStage(const wxString& name) const
{
    for (auto& s : m_stages)
        if (s->Name() == name)
            return s.get();
    return nullptr;
}

void NoisePipeline::LoadConfig()
{
    for (auto& s : m_stages)
        s->LoadConfig();
}

void NoisePipeline::SaveConfig()
{
    for (auto& s : m_stages)
        s->SaveConfig();
}

void NoisePipeline::ResetAll()
{
    for (auto& s : m_stages)
        s->Reset();
}

void InitNoisePipeline()
{
    if (pNoise)
        return;
    pNoise = new NoisePipeline();
    // HotPixel runs first: it's a stateless spatial filter that strips
    // cosmic rays and isolated hot pixels before RollingBackground gets
    // to observe the frame. Doing it the other way round would let those
    // spikes pollute the EMA's outlier detector during the warmup phase.
    pNoise->AddStage(std::unique_ptr<NoiseReductionStage>(new HotPixelStage()));
    pNoise->AddStage(std::unique_ptr<NoiseReductionStage>(new RollingBackgroundStage()));
    // DeadZone runs last: it overwrites flagged regions in the
    // already-background-subtracted frame so the star detector cannot
    // find spurious peaks there. Running it earlier would force
    // RollingBackground to observe synthetic fill values, which would
    // bias the model in those areas.
    pNoise->AddStage(std::unique_ptr<NoiseReductionStage>(new NoiseStageDeadZone()));
    pNoise->LoadConfig();
}

void ShutdownNoisePipeline()
{
    if (!pNoise)
        return;
    pNoise->SaveConfig();
    delete pNoise;
    pNoise = nullptr;
}
