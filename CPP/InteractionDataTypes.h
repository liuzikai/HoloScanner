//
// Created by Zikai Liu on 11/14/22.
//

#pragma once

#include "Timestamp.h"
#include <DirectXMath.h>

struct HandJoint {
    bool tracked;
    DirectX::XMMATRIX transformation;
};

enum HandJointIndex {
    Palm,
    Wrist,
    ThumbMetacarpal,
    ThumbProximal,
    ThumbDistal,
    ThumbTip,
    IndexMetacarpal,
    IndexProximal,
    IndexIntermediate,
    IndexDistal,
    IndexTip,
    MiddleMetacarpal,
    MiddleProximal,
    MiddleIntermediate,
    MiddleDistal,
    MiddleTip,
    RingMetacarpal,
    RingProximal,
    RingIntermediate,
    RingDistal,
    RingTip,
    PinkyMetacarpal,
    PinkyProximal,
    PinkyIntermediate,
    PinkyDistal,
    PinkyTip,
    HandJointIndexCount,
};

struct Hand {
    bool tracked;
    HandJoint joints[HandJointIndexCount];
};

enum HandIndex {
    Left,
    Right,
    HandIndexCount,
};

struct InteractionFrame {
    DirectX::XMMATRIX headTransformation;

    Hand hands[HandIndexCount];

    bool eyeTracked;
    DirectX::XMVECTOR eyeOrigin;
    DirectX::XMVECTOR eyeDirection;
};

class InteractionSource {
public:
    virtual bool getNextInteractionFrame(timestamp_t &timestamp, InteractionFrame &frame) = 0;
};
