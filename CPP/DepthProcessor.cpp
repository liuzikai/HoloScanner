//
// Created by Zikai Liu on 11/14/22.
//

#include <cmath>
#include <iostream>
#include "DirectXHelpers.h"
#include "DepthProcessor.h"

using namespace DirectX;

static inline float pow2(float a) { return a * a; }

DepthProcessor::DepthProcessor(const DirectX::XMMATRIX &extrinsics, const float *lutBuf)
        : extrinsics(extrinsics), cam2rig(XMMatrixInverse(nullptr, extrinsics)) {

    // Store LUT as XMVector
    lut.reserve(CLIPPED_DEPTH_FRAME_WIDTH * CLIPPED_DEPTH_FRAME_HEIGHT);
    for (size_t i = 0; i < AHAT_HEIGHT; i++) {
        for (size_t j = 0; j < AHAT_WIDTH; j++) {
            if (i >= ROI_ROW_LOWER && i < ROI_ROW_UPPER && j >= ROI_COL_LOWER && j < ROI_COL_UPPER) {
                lut.emplace_back(XMLoadFloat3((const XMFLOAT3 *) lutBuf));
            }
            lutBuf += 3;
        }
    }

    // initialize std deviation back buffer
    for (auto i = 0; i < STD_LOG_SIZE; ++i) {
        clippedDepthFrames.emplace_back(CLIPPED_DEPTH_FRAME_WIDTH * CLIPPED_DEPTH_FRAME_HEIGHT, 0.0f);
    }

    clippedDepthMovingSum.resize(CLIPPED_DEPTH_FRAME_WIDTH * CLIPPED_DEPTH_FRAME_HEIGHT, 0.0f);
    stdVal.resize(CLIPPED_DEPTH_FRAME_WIDTH * CLIPPED_DEPTH_FRAME_HEIGHT, 0.0f);

//    handBones

    //handBonesRight = {
    //    /* Thumb  */ {27, 28}, /**/ {28, 29}, /**/{ 29, 30 }, /**/{ 30, 31 },
    //    /* Index  */ {27, 32}, /**/ {32, 33}, /**/ {33, 34}, /**/ {34, 35}, /**/ {35, 36},
    //    /* Middle */ {27, 37}, /**/ {37, 38}, /**/ {38, 39}, /**/ {39, 40}, /**/ {40, 41},
    //    /* Ring   */ {27, 42}, /**/ {42, 43}, /**/ {43, 44}, /**/ {44, 45}, /**/ {45, 46},
    //    /* Pinky  */ {27, 47}, /**/ {47, 48}, /**/ {48, 49}, /**/ {49, 50}, /**/ {50, 51}
    //};

    //d_finger = self.hands__finger_radius = 0.025
    //d_hand = self.hands__wrist_radius = 0.05
    //dist = [d_hand, d_hand] + (hands.HandJointIndex.Count.value - 2) * [d_finger]

    // everything in the same one to fit the bone indexes
    // left hand
    /*fingerSizes = { WRIST_RADIUS, WRIST_RADIUS };
    for (auto i = 0; i < HandJointIndexCount - 2; ++i) {
        fingerSize.push_back(FINGER_RADIUS);
    }*/
    // right hand
    //fingerSizes.push_back(WRIST_RADIUS);
    //fingerSizes.push_back(WRIST_RADIUS);
    //for (auto i = 0; i < HandJointIndexCount - 2; ++i) {
    //    fingerSize.push_back(FINGER_RADIUS);
    //}
}

bool DepthProcessor::getNextPCDRaw(timestamp_t &timestamp, PCDRaw &pcdRaw) {
    std::lock_guard<std::mutex> lock(pcdMutex);
    if (pcdRawFrames.empty()) return false;
    timestamp = pcdRawFrames.front().first;
    pcdRaw = std::move(pcdRawFrames.front().second);
    pcdRawFrames.pop();
    return true;
}

int DepthProcessor::countTrackedJoints(const Hand &hand) {
    int result = 0;
    for (const auto &joint: hand.joints) {
        if (joint.tracked) result++;
    }
    return result;
}

/*bool createHandMesh(const Hand & hand, std::vector<DirectX::XMVECTOR> &mesh, std::vector<float> &distances) {
    bool tracked = hand.tracked && countTrackedJoints(hand);
    //bool rhTracked = hand.hands[Right].tracked && countTrackedJoints(interactionFrame.hands[Right]);
    //if (!leftTracked || !rightTracked) return; //lostTracking = true;

    if (!tracked) return false;

    mesh.clear();
    distances.clear();
    //auto lh = hand.hands->joints[HandIndex::Left];
    std::vector<DirectX::XMVECTOR> joints;
    for (int j = 0; j < HandJointIndexCount; j++) {
        joints.push_back(XMTransformToTranslate(hand[j].transformation));
    }
    for (auto b : handBones) {
        int j1 = b[0];
        int j2 = b[1];
        float d1 = fingerSizes[j1];
        float d2 = fingerSizes[j2];
        DirectX::XMVECTOR origin = hand[j1];
        DirectX::XMVECTOR boneDir = joints[j2] - origin;
        float boneLength = std::sqrt(
            std::powf(boneDir[0], 2.0f) + 
            std::powf(boneDir[1], 2.0f) + 
            std::powf(boneDir[2], 2.0f));

        if (boneLength < std::numeric_limits<float>::epsilon()) {
            continue;
        }

        boneDir = boneDir / boneLength;
        float d = std::fminf(d1, d2);
        float segmentCount = std::ceil(boneLength / d);

        if (segmentCount < std::numeric_limits<float>::epsilon()) {
            continue;
        }

        float delta = boneLength / segmentCount;
        int sCount = static_cast<int>(segmentCount);
        for (int s = 0; s < sCount; ++s) {
            mesh.push_back(origin + i * delta * boneDir);
            float percent = 1.0f / boneLength * i * delta;
            distances.push_back(d1 * (1 - percent) + d2 * percent);
        }
    }

    return true;
}*/

bool DepthProcessor::update(const RawDataFrame &input) {
    std::pair<timestamp_t, PCDRaw> frame;
    frame.first = input.timestamp;
    frame.second.reserve((ROI_ROW_UPPER - ROI_ROW_LOWER) * (ROI_COL_UPPER - ROI_COL_LOWER));

    bool leftTracked = input.hands[Left].tracked && countTrackedJoints(input.hands[Left]);
    bool rightTracked = input.hands[Right].tracked && countTrackedJoints(input.hands[Right]);
    if (!leftTracked || !rightTracked) return false;  // tell the user of dropping frame

    auto lwTranf = input.hands[HandIndex::Left].joints[HandJointIndex::Wrist].translationInRig;
    auto rwTransf = input.hands[HandIndex::Right].joints[HandJointIndex::Wrist].translationInRig;
    auto midpoint = (lwTranf + rwTransf) * 0.5f;

    auto midpZ = XMVectorGetZ(midpoint) * 1000.0f;
    auto depthNearClip = static_cast<uint16_t>(midpZ - 200.0f);  // 200mm
    auto depthFarClip = static_cast<uint16_t>(midpZ + 200.0f);
#if 0
    std::cout << "depthNearClip = " << depthNearClip << ", depthFarClip = " << depthFarClip << std::endl;
#endif

    size_t bufferInd = stdLogIndex;
    size_t nextInd = (stdLogIndex + 1) % STD_LOG_SIZE;
    for (auto i = ROI_ROW_LOWER; i < ROI_ROW_UPPER; i++) {
        for (auto j = ROI_COL_LOWER; j < ROI_COL_UPPER; j++) {
            uint16_t depth = input.depth[(i * AHAT_WIDTH + j)];
            depth = (depth > 4090) ? 0 : depth;

            size_t ind = (i - ROI_ROW_LOWER) * CLIPPED_DEPTH_FRAME_WIDTH + (j - ROI_COL_LOWER);
            clippedDepthFrames[bufferInd][ind] = depth;
            clippedDepthMovingSum[ind] -= (float) clippedDepthFrames[nextInd][ind];
            clippedDepthMovingSum[ind] += (float) depth;
        }
    }
    stdLogIndex = nextInd;

    float curMaxStdVal = 0;
    for (auto r = 0; r < CLIPPED_DEPTH_FRAME_HEIGHT; ++r) {
        for (auto c = 0; c < CLIPPED_DEPTH_FRAME_WIDTH; ++c) {
            size_t ind = r * CLIPPED_DEPTH_FRAME_WIDTH + c;

            float dev = 0;
            for (auto b = 0; b < STD_LOG_SIZE; ++b) {
                dev += pow2((float) clippedDepthFrames[b][ind] - (clippedDepthMovingSum[ind] / STD_LOG_SIZE));
            }
            // dev = std::sqrtf(dev / STD_LOG_SIZE);
            stdVal[ind] = dev;

            if (dev > curMaxStdVal) {
                curMaxStdVal = dev;
            }
        }
    }

    for (auto r = 0; r < CLIPPED_DEPTH_FRAME_HEIGHT; ++r) {
        for (auto c = 0; c < CLIPPED_DEPTH_FRAME_WIDTH; ++c) {
            size_t ind = r * CLIPPED_DEPTH_FRAME_WIDTH + c;

            uint16_t depth = clippedDepthFrames[bufferInd][ind];
            float cDev = stdVal[ind];

#if 0
            if (220 * CLIPPED_DEPTH_FRAME_WIDTH + 256 == ind) {
                std::cout << "depth = " << depth << ", midpZ = " << midpZ << ", cDev = " << cDev << ", max = " << curMaxStdVal << std::endl;
            }
#endif

            if (cDev < MAX_STD_VAL && depth > depthNearClip && depth < depthFarClip) {
                auto pointInCam = ((float) depth / 1000.0f) * lut[ind];
                auto pointInRig = XMVector3Transform(pointInCam, cam2rig);

                XMFLOAT4 f;
                XMStoreFloat4(&f, pointInRig);
                frame.second.push_back(f.x / f.w);
                frame.second.push_back(f.y / f.w);
                frame.second.push_back(-f.z / f.w);
            }
        }
    }

    // Save the data
    {
        std::lock_guard<std::mutex> lock(pcdMutex);
        pcdRawFrames.emplace(std::move(frame));
    }

    return true;
}
