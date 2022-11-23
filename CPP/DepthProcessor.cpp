//
// Created by Zikai Liu on 11/14/22.
//

#include "DepthProcessor.h"

using namespace DirectX;

DepthProcessor::DepthProcessor(const DirectX::XMMATRIX &extrinsics, const float *lutBuf)
        : extrinsics(extrinsics), depthCameraPoseInvMatrix(XMMatrixInverse(nullptr, extrinsics)) {

    // Store LUT as XMVector
    //lut.reserve(AHAT_WIDTH * AHAT_HEIGHT);
    lut.reserve(clippedDepthFrameHeight * clippedDepthFrameWidth);
    //for (size_t i = 0; i < AHAT_HEIGHT; i++) {
    for (size_t i = roiRowLower; i < roiRowUpper; i++) {
        //for (size_t j = 0; j < AHAT_WIDTH; j++) {
        for (size_t j = roiColLower; j < roiColUpper; j++) {
            lut.emplace_back(XMLoadFloat3((const XMFLOAT3 *) lutBuf));
            lutBuf += 3;
        }
    }

    // initialize std deviation back buffer
    for (auto i = 0; i < stdLogSize; ++i) {
        stdDepthVec.push_back(std::vector<float>(clippedDepthFrameWidth * clippedDepthFrameHeight), 0.0f);
    }

    depthMovingAverage.resize(clippedDepthFrameWidth * clippedDepthFrameHeight, 0.0f);
    stdVal.resize(clippedDepthFrameWidth * clippedDepthFrameHeight, 0.0f);

}

bool DepthProcessor::getNextPCDRaw(timestamp_t &timestamp, PCDRaw &pcdRaw) {
    std::lock_guard<std::mutex> lock(pcdMutex);
    if (pcdRawFrames.empty()) return false;
    timestamp = pcdRawFrames.front().first;
    pcdRaw = std::move(pcdRawFrames.front().second);
    pcdRawFrames.pop();
    return true;
}

int DepthProcessor::countTrackedJoints(const Hand& hand) {
    int result = 0;
    for (const auto& joint : hand.joints) {
        if (joint.tracked) result++;
    }
    return result;
}

void DepthProcessor::updateAHAT(
    const timestamp_t &timestamp,
    const uint16_t *depthFrame,
    const DirectX::XMMATRIX &rig2world,
    const InteractionFrame& hand
    ) {
    std::pair<timestamp_t, PCDRaw> frame;
    frame.first = timestamp;
    frame.second.reserve((ROI_ROW_UPPER - ROI_ROW_LOWER) * (ROI_COL_UPPER - ROI_COL_LOWER));

    XMMATRIX depthToWorld = depthCameraPoseInvMatrix * rig2world;

    bool leftTracked = interactionFrame.hands[Left].tracked && countTrackedJoints(interactionFrame.hands[Left]);
    bool rightTracked = interactionFrame.hands[Right].tracked && countTrackedJoints(interactionFrame.hands[Right]);
    if (!leftTracked || !rightTracked) return; //lostTracking = true;

    DirectX::XMMATRIX leftWrist = hand.hands->joints[HandIndex::Left][HandJointIndex::Wrist].transformation;
    DirectX::XMMATRIX rightWrist = hand.hands->joints[HandIndex::Right][HandJointIndex::Wrist].transformation;

    auto lwTranf = XMTransformToTranslate(leftWrist);
    auto rwTransf = XMTransformToTranslate(rightWrist);
    auto midpoint = XMVectorMidpoint(leftWrist, rightWrist);

    uint16_t midpZ = static_cast<uint16_t>(midpoint[2] * 1000);
    uint16_t depthNearClip = static_cast<uint16_t>(midpZ - 200.0f); // 200 == 200cm
    uint16_t depthFarClip = static_cast<uint16_t>(midpZ + 200.0f);

    //self.window__cp_x_from = 256 - 168
    //self.window__cp_x_to = 256 + 80
    //self.window__cp_y_from = 256 - 128
    //self.window__cp_y_to = 256 + 128


    size_t bufferInd = stdLogIndex % stdLogSize;
    size_t nextInd = (stdLogIndex + 1) % stdLogSize;
    for (auto i = roiRowLower; i < rowRowUpper; i++) {
        for (auto j = roiColLower; j < roiColUpper; j++) {
            //auto idx = AHAT_WIDTH * i + j;
            uint16_t depth = depthFrame[idx];
            depth = (depth > 4090) ? 0 : depth;

            size_t ind = i * clippedDepthFrameHeight + j;
            stdDepthVec[bufferInd][ind] = static_cast<float>(depth);
            depthMovingAverage[ind] -= stdDepthVec[nextInd][ind];
            depthMovingAverage[ind] += depth;
        }
    }

    float s = static_cast<float>(stdLogSize);
    float curMaxStdVal = std::numeric_limits<float>::min();
    for (auto r = 0; r < clippedDepthFrameHeight; ++r) {
        for (auto c = 0; c < clippedDepthFrameWidth; ++c) {
            size_t ind = r * clippedDepthFrameHeight + c;
            float dev = 0;
            for (auto b = 0; b < stdLogSize; ++d) {
                dev += std::powf((stdDepthVec[b][ind] - depthMovingAverage[ind] / s), 2.0f);
            }

            stdVal[ind] = dev;
            if (dev > curMaxStdVal) {
                curMaxStdVal = dev;
            }
        }
    }

    for (auto r = 0; r < clippedDepthFrameHeight; ++r) {
        for (auto c = 0; c < clippedDepthFrameWidth; ++c) {
            size_t ind = r * clippedDepthFrameHeight + c;

            float depth = stdDepthVec[bufferInd][ind];
            float cDev = stdVal[ind] / curMaxStdVal;
            if (cDev < maxStdVal && depth > depthNearClip && depth < depthFarClip) {
                auto tempPoint = depth / 1000 * lut[ind];

                // apply transformation
                auto pointInWorld = XMVector3Transform(tempPoint, depthToWorld);

                XMFLOAT3 f;
                XMStoreFloat3(&f, pointInWorld);
                frame.second.push_back(f.x);
                frame.second.push_back(f.y);
                frame.second.push_back(-f.z);
            }
        }
    }

    // Save the data
    {
        std::lock_guard<std::mutex> lock(pcdMutex);
        pcdRawFrames.emplace(std::move(frame));
    }
}
