//
// Created by Zikai Liu on 11/14/22.
//

#ifndef HOLOSCANNER_DEPTHPROCESSOR_H
#define HOLOSCANNER_DEPTHPROCESSOR_H

#include "DepthDataTypes.h"
#include <queue>
#include <mutex>

class DepthProcessor {
public:

    DepthProcessor(const DirectX::XMMATRIX &extrinsics, const float *lut);

    void updateAHAT(const timestamp_t &timestamp, const uint16_t *depth, const DirectX::XMMATRIX &rig2world);

    bool getNextPCDRaw(timestamp_t &timestamp, PCDRaw &pcdRaw);

private:

    struct DepthCamRoi {
        float kRowLower = 0.2;
        float kRowUpper = 0.55;
        float kColLower = 0.3;
        float kColUpper = 0.7;
        uint16_t depthNearClip = 200; // Unit: mm
        uint16_t depthFarClip = 800;
    } depthCamROI;

    std::queue<std::pair<timestamp_t, PCDRaw>> pcdRawFrames;
    std::mutex pcdMutex;

    DirectX::XMMATRIX extrinsics;
    DirectX::XMMATRIX depthCameraPoseInvMatrix;

    std::vector<DirectX::XMVECTOR> lut;
};


#endif //HOLOSCANNER_DEPTHPROCESSOR_H
