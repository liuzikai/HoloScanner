//
// Created by Zikai Liu on 11/14/22.
//

#include "DepthProcessor.h"

using namespace DirectX;

DepthProcessor::DepthProcessor(const DirectX::XMMATRIX &extrinsics, const float *lutBuf)
        : extrinsics(extrinsics), depthCameraPoseInvMatrix(XMMatrixInverse(nullptr, extrinsics)) {

    // Store LUT as XMVector
    lut.reserve(AHAT_WIDTH * AHAT_HEIGHT);
    for (size_t i = 0; i < AHAT_HEIGHT; i++) {
        for (size_t j = 0; j < AHAT_WIDTH; j++) {
            lut.emplace_back(XMLoadFloat3((const XMFLOAT3 *) lutBuf));
            lutBuf += 3;
        }
    }
}

bool DepthProcessor::getNextPCDRaw(timestamp_t &timestamp, PCDRaw &pcdRaw) {
    std::lock_guard<std::mutex> lock(pcdMutex);
    if (pcdRawFrames.empty()) return false;
    timestamp = pcdRawFrames.front().first;
    pcdRaw = std::move(pcdRawFrames.front().second);
    pcdRawFrames.pop();
    return true;
}

void DepthProcessor::updateAHAT(const timestamp_t &timestamp, const uint16_t *depthFrame,
                                const DirectX::XMMATRIX &rig2world) {
    std::pair<timestamp_t, PCDRaw> frame;
    frame.first = timestamp;
    frame.second.reserve((ROI_ROW_UPPER - ROI_ROW_LOWER) * (ROI_COL_UPPER - ROI_COL_LOWER));

    XMMATRIX depthToWorld = depthCameraPoseInvMatrix * rig2world;

    for (auto i = ROI_ROW_LOWER; i < ROI_ROW_UPPER; i++) {
        for (auto j = ROI_COL_LOWER; j < ROI_COL_UPPER; j++) {
            auto idx = AHAT_WIDTH * i + j;
            uint16_t depth = depthFrame[idx];
            depth = (depth > 4090) ? 0 : depth;

            if (depth > DEPTH_NEAR_CLIP && depth < DEPTH_FAR_CLIP) {

                // optimize with LUT
                auto tempPoint = (float) depth / 1000 * lut[idx];

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
