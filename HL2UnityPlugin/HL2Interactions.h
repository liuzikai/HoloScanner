#pragma once
#include "HL2Interactions.g.h"

namespace winrt::HL2UnityPlugin::implementation
{
    struct HL2Interactions : HL2InteractionsT<HL2Interactions>
    {
        HL2Interactions() = default;

        void SetReferenceCoordinateSystem(Windows::Perception::Spatial::SpatialCoordinateSystem const& refCoord);
        void Update(Windows::Perception::PerceptionTimestamp const& timestamp);
        com_array<float> GetHeadTransform();
        bool IsHandTracked(HL2UnityPlugin::HandIndex const& handIndex);
        com_array<float> GetOrientedJoint(HL2UnityPlugin::HandIndex const& handIndex);
        void EnableEyeTracking();
        bool IsEyeTrackingActive();
        com_array<float> GetEyeGazeOrigin();
        com_array<float> GetEyeGazeDirection();
    };
}
namespace winrt::HL2UnityPlugin::factory_implementation
{
    struct HL2Interactions : HL2InteractionsT<HL2Interactions, implementation::HL2Interactions>
    {
    };
}
