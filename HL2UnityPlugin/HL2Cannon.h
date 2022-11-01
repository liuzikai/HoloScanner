#pragma once
#include "HL2Cannon.g.h"

namespace winrt::HL2UnityPlugin::implementation
{
    struct HL2Cannon : HL2CannonT<HL2Cannon>
    {
        HL2Cannon() = default;

        void StartUpdateLoop();
        void StopUpdateLoop();
        HL2UnityPlugin::Matrix44 GetHeadTransform();
        bool IsHandTracked(HL2UnityPlugin::HandIndex const& handIndex);
        HL2UnityPlugin::Vector4 GetJoint(HL2UnityPlugin::HandIndex const& handIndex, HL2UnityPlugin::HandJointIndex const& jointIndex);
        HL2UnityPlugin::Vector4 GetJointOrientation(HL2UnityPlugin::HandIndex const& handIndex, HL2UnityPlugin::HandJointIndex const& jointIndex);
        HL2UnityPlugin::Matrix44 GetOrientedJoint(HL2UnityPlugin::HandIndex const& handIndex, HL2UnityPlugin::HandJointIndex const& jointIndex);
        bool IsEyeTrackingActive();
        HL2UnityPlugin::Vector4 GetEyeGazeOrigin();
        HL2UnityPlugin::Vector4 GetEyeGazeDirection();
    };
}
namespace winrt::HL2UnityPlugin::factory_implementation
{
    struct HL2Cannon : HL2CannonT<HL2Cannon, implementation::HL2Cannon>
    {
    };
}
