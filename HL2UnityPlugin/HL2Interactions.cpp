#include "pch.h"
#include "HL2Interactions.h"
#include "HL2Interactions.g.cpp"

using namespace DirectX;

namespace winrt::HL2UnityPlugin::implementation
{

    HL2Interactions::HL2Interactions() 
    {
        m_headPosition = XMVectorZero();
        m_headForwardDirection = XMVectorSet(0.0f, 0.0f, -1.0f, 0.0f);
        m_headUpDirection = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
        m_headTransform = XMMatrixIdentity();

	    m_isArticulatedHandTrackingAPIAvailable = false;
        memset(m_handTracked, 0, sizeof(m_handTracked));

        m_isEyeTrackingAvailable = false;
        m_isEyeTrackingRequested = false;
        m_isEyeTrackingEnabled = false;
        m_isEyeTrackingActive = false;
        m_eyeGazeOrigin = XMVectorZero();
	    m_eyeGazeDirection = XMVectorSet(0.0f, 0.0f, -1.0f, 0.0f);

        if (winrt::Windows::Foundation::Metadata::ApiInformation::IsMethodPresent(L"Windows.UI.Input.Spatial.SpatialInteractionSourceState", L"TryGetHandPose"))
            m_isArticulatedHandTrackingAPIAvailable = true;

        if (winrt::Windows::Foundation::Metadata::ApiInformation::IsMethodPresent(L"Windows.Perception.People.EyesPose", L"IsSupported"))
            m_isEyeTrackingAvailable = winrt::Windows::Perception::People::EyesPose::IsSupported();

    }

    // Set the reference coordinate system. Need to be set before the sensor loop starts; otherwise, default coordinate will be used.
    void HL2Interactions::SetReferenceCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem refCoord)
    {
        m_refFrame = refCoord;
    }

    void HL2Interactions::Update(Windows::Perception::PerceptionTimestamp timestamp)
    {
        // Update head and eye
        winrt::Windows::UI::Input::Spatial::SpatialPointerPose pointerPose = winrt::Windows::UI::Input::Spatial::SpatialPointerPose::TryGetAtTimestamp(m_refFrame, timestamp);
        if (pointerPose)
        {
            m_headPosition = XMVectorSetW(XMLoadFloat3(&pointerPose.Head().Position()), 1.0f);
            m_headForwardDirection = XMLoadFloat3(&pointerPose.Head().ForwardDirection());
            m_headUpDirection = XMLoadFloat3(&pointerPose.Head().UpDirection());
            XMMATRIX worldTransform = XMMatrixLookToRH(XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f), m_headForwardDirection, m_headUpDirection);
	        m_headTransform = XMMatrixMultiply(XMMatrixTranspose(worldTransform), XMMatrixTranslationFromVector(m_headPosition));

            if (m_isEyeTrackingEnabled)
            {
                if (pointerPose.Eyes() && pointerPose.Eyes().IsCalibrationValid())
                {
                    m_isEyeTrackingActive = true;

                    if (pointerPose.Eyes().Gaze())
                    {
                        auto spatialRay = pointerPose.Eyes().Gaze().Value();
                        m_eyeGazeOrigin = XMVectorSetW(XMLoadFloat3(&spatialRay.Origin), 1.0f);
                        m_eyeGazeDirection = XMLoadFloat3(&spatialRay.Direction);
                    }
                }
                else
                {
                    m_isEyeTrackingActive = false;
                }
            }
        }

        // Request eye tracking access if needed
        if (m_isEyeTrackingAvailable && m_isEyeTrackingRequested && !m_isEyeTrackingEnabled)
        {
            m_isEyeTrackingRequested = false;

            std::thread requestAccessThread([this]()
                {
                    auto status = winrt::Windows::Perception::People::EyesPose::RequestAccessAsync().get();

                    if (status == winrt::Windows::UI::Input::GazeInputAccessStatus::Allowed)
                        m_isEyeTrackingEnabled = true;
                    else
                        m_isEyeTrackingEnabled = false;
                });

            requestAccessThread.detach();
        }

        // Update hands
        memset(m_handTracked, 0, sizeof(m_handTracked));
        auto sourceStates = m_spatialInteractionManager.GetDetectedSourcesAtTimestamp(timestamp);
        for (auto currentState : sourceStates)
        {
            if (currentState.Source().Kind() != winrt::Windows::UI::Input::Spatial::SpatialInteractionSourceKind::Hand) continue;
            
            // Check handedness
            int handIndex;
            switch (currentState.Source().Handedness())
            {
                case winrt::Windows::UI::Input::Spatial::SpatialInteractionSourceHandedness::Left:
                    handIndex = HL2UnityPlugin::HandIndex::Left;
                    break;
                case winrt::Windows::UI::Input::Spatial::SpatialInteractionSourceHandedness::Right:
                    handIndex = HL2UnityPlugin::HandIndex::Right;
                    break;
                default:
                    continue;
            }

            auto &hand = m_hand[handIndex];
            m_handTracked[handIndex] = true;

            // Get joint data
            winrt::Windows::Perception::People::HandPose handPose{ nullptr };
            if (m_isArticulatedHandTrackingAPIAvailable)
            {
                handPose = currentState.TryGetHandPose();
            }

            if (handPose)
            {
                static vector<winrt::Windows::Perception::People::HandJointKind> requestedJointIndices;
                if (requestedJointIndices.empty())
                {
                    requestedJointIndices.resize((size_t) HL2UnityPlugin::HandJointIndex::Count);
                    for (int jointIndex = 0; jointIndex < (int) HL2UnityPlugin::HandJointIndex::Count; ++jointIndex)
                    {
                        requestedJointIndices[jointIndex] = (winrt::Windows::Perception::People::HandJointKind) jointIndex;
                    }
                }

                static vector<winrt::Windows::Perception::People::JointPose> jointPoses(requestedJointIndices.size());

                if (handPose.TryGetJoints(m_refFrame, requestedJointIndices, jointPoses))
                {
                    auto jointCount = requestedJointIndices.size();
                    hand.handJoints.resize(jointCount);

                    for (unsigned jointIndex = 0; jointIndex < jointCount; ++jointIndex)
                    {
                        HandJoint &joint = hand.handJoints[jointIndex];
                        joint.position = XMVectorSetW(XMLoadFloat3(&jointPoses[jointIndex].Position), 1.0f);
                        joint.orientation = XMLoadFloat4((XMFLOAT4*) &jointPoses[jointIndex].Orientation);
                        joint.radius = jointPoses[jointIndex].Radius;
                        joint.tracked = (jointPoses[jointIndex].Accuracy == winrt::Windows::Perception::People::JointPoseAccuracy::High);

                        joint.transformation = XMMatrixAffineTransformation(XMVectorSet(1.0f, 1.0f, 1.0f, 1.0f), XMVectorZero(), joint.position, joint.orientation);
                    }
                }
            }

            // Get location of hand input source itself
            auto location = currentState.Properties().TryGetLocation(m_refFrame);
            if (location)
            {
                auto position = location.Position();
                if(position) hand.position = XMLoadFloat3(&position.Value());
                
                auto orientation = location.Orientation();
                if (orientation) hand.orientation = XMLoadFloat4((XMFLOAT4*) &orientation.Value());
            }
        }
    }

    Windows::Foundation::Numerics::float4x4 HL2Interactions::GetHeadTransform()
    {
        return XMMATRIXToFloat4x4(m_headTransform);
    }

    bool HL2Interactions::IsHandTracked(HL2UnityPlugin::HandIndex handIndex)
    {
        return m_handTracked[handIndex];
    }

    bool HL2Interactions::IsJointTracked(HL2UnityPlugin::HandIndex handIndex, HL2UnityPlugin::HandJointIndex jointIndex)
    {
        return m_handTracked[handIndex] && m_hand[handIndex].handJoints[jointIndex].tracked;
    }

    Windows::Foundation::Numerics::float4x4 HL2Interactions::GetOrientedJoint(HL2UnityPlugin::HandIndex handIndex, HL2UnityPlugin::HandJointIndex jointIndex)
    {
        throw XMMATRIXToFloat4x4(m_hand[handIndex].handJoints[jointIndex].transformation);
    }

    void HL2Interactions::EnableEyeTracking()
    {
        m_isEyeTrackingRequested = true;
    }

    bool HL2Interactions::IsEyeTrackingEnabled()
    {
        return m_isEyeTrackingEnabled;
    }

    bool HL2Interactions::IsEyeTrackingActive()
    {
        return m_isEyeTrackingActive;
    }

    Windows::Foundation::Numerics::float4 HL2Interactions::GetEyeGazeOrigin()
    {
        return XMVECTORToFloat4(m_eyeGazeOrigin);
    }

    Windows::Foundation::Numerics::float4 HL2Interactions::GetEyeGazeDirection()
    {
        return XMVECTORToFloat4(m_eyeGazeDirection);
    }

    Windows::Foundation::Numerics::float4 HL2Interactions::XMVECTORToFloat4(const XMVECTOR &vector) 
    {
        Windows::Foundation::Numerics::float4 ret;
        XMStoreFloat4(&ret, vector);
        return ret;
    }

    Windows::Foundation::Numerics::float4 HL2Interactions::XMMATRIXToFloat4x4(const XMMATRIX &matrix) 
    {
        Windows::Foundation::Numerics::float4x4 ret;
        XMStoreFloat4x4(&ret, matrix);
        return ret;
    }
}
