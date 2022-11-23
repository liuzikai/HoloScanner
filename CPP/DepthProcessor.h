//
// Created by Zikai Liu on 11/14/22.
//

#ifndef HOLOSCANNER_DEPTHPROCESSOR_H
#define HOLOSCANNER_DEPTHPROCESSOR_H

#include <queue>
#include <mutex>

#include "DepthDataTypes.h"
#include "InteractionDataTypes.h"

class DepthProcessor {
public:

    DepthProcessor(const DirectX::XMMATRIX &extrinsics, const float *lut);

    void updateAHAT(const timestamp_t &timestamp, const uint16_t *depth, const DirectX::XMMATRIX &rig2world, const InteractionFrame &hand);

    bool getNextPCDRaw(timestamp_t &timestamp, PCDRaw &pcdRaw);

private:
    int countTrackedJoints(const Hand& hand);
    static constexpr size_t ROI_ROW_LOWER = (size_t) (0.2 * AHAT_HEIGHT);
    static constexpr size_t ROI_ROW_UPPER = (size_t) (0.55 * AHAT_HEIGHT);
    static constexpr size_t ROI_COL_LOWER = (size_t) (0.3 * AHAT_WIDTH);
    static constexpr size_t ROI_COL_UPPER = (size_t) (0.7 * AHAT_WIDTH);
    static constexpr uint16_t DEPTH_NEAR_CLIP = 200; // Unit: mm
    static constexpr uint16_t DEPTH_FAR_CLIP = 800;

    constexpr size_t roiColLower = 256 - 128;
    constexpr size_t roiColUpper = 256 + 128;
    constexpr size_t roiRowLower = 256 - 168;
    constexpr size_t roiRowUpper = 256 + 80;
    constexpr size_t clippedDepthFrameWidth = roiColUpper - roiColLower;
    constexpr size_t clippedDepthFrameHeight = roiRowUpper - roiRowLower;

    size_t stdLogIndex = 0;
    constexpr size_t stdLogSize = 10;
    constexpr float maxStdVal = 0.01*0.01; // squared value on purpose
    std::vector<float> stdVal;
    std::vector<float> depthMovingAverage;
    std::vector<std::vector<float>> stdDepthVec;
    std::queue<std::pair<timestamp_t, PCDRaw>> pcdRawFrames;
    std::mutex pcdMutex;

    DirectX::XMMATRIX extrinsics;
    DirectX::XMMATRIX depthCameraPoseInvMatrix;

    std::vector<DirectX::XMVECTOR> lut;
};


#endif //HOLOSCANNER_DEPTHPROCESSOR_H
