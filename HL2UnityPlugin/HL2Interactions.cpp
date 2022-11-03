#include "pch.h"
#include "HL2Interactions.h"
#include "HL2Interactions.g.cpp"

namespace winrt::HL2UnityPlugin::implementation
{
    void HL2Interactions::SetReferenceCoordinateSystem(Windows::Perception::Spatial::SpatialCoordinateSystem const& refCoord)
    {
        throw hresult_not_implemented();
    }
    void HL2Interactions::Update(Windows::Perception::PerceptionTimestamp const& timestamp)
    {
        throw hresult_not_implemented();
    }
    com_array<float> HL2Interactions::GetHeadTransform()
    {
        throw hresult_not_implemented();
    }
    bool HL2Interactions::IsHandTracked(HL2UnityPlugin::HandIndex const& handIndex)
    {
        throw hresult_not_implemented();
    }
    com_array<float> HL2Interactions::GetOrientedJoint(HL2UnityPlugin::HandIndex const& handIndex)
    {
        throw hresult_not_implemented();
    }
    void HL2Interactions::EnableEyeTracking()
    {
        throw hresult_not_implemented();
    }
    bool HL2Interactions::IsEyeTrackingActive()
    {
        throw hresult_not_implemented();
    }
    com_array<float> HL2Interactions::GetEyeGazeOrigin()
    {
        throw hresult_not_implemented();
    }
    com_array<float> HL2Interactions::GetEyeGazeDirection()
    {
        throw hresult_not_implemented();
    }
}
