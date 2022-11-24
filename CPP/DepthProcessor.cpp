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

    // Initialize std deviation logging buffers
    for (auto i = 0; i < STD_LOG_SIZE; ++i) {
        clippedDepthFrames.emplace_back(CLIPPED_DEPTH_FRAME_WIDTH * CLIPPED_DEPTH_FRAME_HEIGHT, 0.0f);
    }
    clippedDepthMovingSum.resize(CLIPPED_DEPTH_FRAME_WIDTH * CLIPPED_DEPTH_FRAME_HEIGHT, 0.0f);
}

bool DepthProcessor::getNextPCDRaw(timestamp_t &timestamp, PCDRaw &pcdRaw) {
    std::lock_guard<std::mutex> lock(pcdMutex);
    if (pcdRawFrames.empty()) return false;
    timestamp = pcdRawFrames.front().first;
    pcdRaw = std::move(pcdRawFrames.front().second);
    pcdRawFrames.pop();
    return true;
}

bool DepthProcessor::updateHandMesh(const Hand &hand, std::vector<DirectX::XMVECTOR> &mesh,
                                    std::vector<float> &filterDistances) {
    if (!hand.strictlyTracked) return false;

    mesh.clear();
    filterDistances.clear();

    for (const auto &b: HAND_BONES) {
        const float d1 = FINGER_SIZES[b[0]];
        const float d2 = FINGER_SIZES[b[1]];
        DirectX::XMVECTOR origin = hand.joints[b[0]].translationInRig;
        DirectX::XMVECTOR boneDirection = hand.joints[b[1]].translationInRig - origin;
        float boneLength = XMVectorGetX(XMVector3Length(boneDirection));

        if (boneLength < std::numeric_limits<float>::epsilon()) {
            continue;
        }

        boneDirection = boneDirection / boneLength;
        float d = std::fminf(d1, d2);
        float segmentCount = std::ceil(boneLength / d);

        if (segmentCount < std::numeric_limits<float>::epsilon()) {
            continue;
        }

        float delta = boneLength / segmentCount;
        int sCount = static_cast<int>(segmentCount);
        for (int s = 0; s < sCount; ++s) {
            float x = (float) s * delta;
            mesh.push_back(origin + x * boneDirection);
            float percent = 1.0f / boneLength * x;
            filterDistances.push_back(d1 * (1.0f - percent) + d2 * percent);
        }
    }

    return true;
}

bool DepthProcessor::update(const RawDataFrame &input) {
    if (!input.hands[Left].strictlyTracked || !input.hands[Right].strictlyTracked) {
        return false;  // tell the user of dropping frame
    }

    updateHandMesh(input.hands[Left], handMesh[Left], handFilterDistances[Left]);
    updateHandMesh(input.hands[Right], handMesh[Right], handFilterDistances[Right]);

    std::pair<timestamp_t, PCDRaw> frame;
    frame.first = input.timestamp;
    frame.second.reserve(CLIPPED_DEPTH_FRAME_WIDTH * CLIPPED_DEPTH_FRAME_HEIGHT);
    
    auto lwTranf = input.hands[HandIndex::Left].joints[HandJointIndex::Wrist].translationInRig;
    auto rwTransf = input.hands[HandIndex::Right].joints[HandJointIndex::Wrist].translationInRig;
    auto midpoint = (lwTranf + rwTransf) * 0.5f;

    auto midpZ = static_cast<uint16_t>(XMVectorGetZ(midpoint) * 1000.0f);
    auto depthNearClip = midpZ - DEPTH_FILTER_OFFSET;
    auto depthFarClip = midpZ + DEPTH_FILTER_OFFSET;
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
    
    for (auto r = 0; r < CLIPPED_DEPTH_FRAME_HEIGHT; ++r) {
        for (auto c = 0; c < CLIPPED_DEPTH_FRAME_WIDTH; ++c) {
            size_t ind = r * CLIPPED_DEPTH_FRAME_WIDTH + c;

            uint16_t depth = clippedDepthFrames[bufferInd][ind];

            // Calculate standard deviation
            float dev = 0;
            for (auto b = 0; b < STD_LOG_SIZE; ++b) {
                dev += pow2((float) clippedDepthFrames[b][ind] - (clippedDepthMovingSum[ind] / STD_LOG_SIZE));
            }
            // dev = std::sqrtf(dev / STD_LOG_SIZE);

#if 0
            if (220 * CLIPPED_DEPTH_FRAME_WIDTH + 256 == ind) {
                std::cout << "depth = " << depth << ", midpZ = " << midpZ << ", cDev = " << cDev << ", max = " << curMaxStdVal << std::endl;
            }
#endif

            if (dev < MAX_STD_VAL && depth > depthNearClip && depth < depthFarClip) {
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
