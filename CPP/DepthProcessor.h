//
// Created by Zikai Liu on 11/14/22.
//

#ifndef HOLOSCANNER_DEPTHPROCESSOR_H
#define HOLOSCANNER_DEPTHPROCESSOR_H

#include <queue>
#include <mutex>

#include "RawDataTypes.h"

class DepthProcessor {
public:

    DepthProcessor(const DirectX::XMMATRIX &extrinsics, const float *lut);

    bool update(const RawDataFrame &rawDataFrame);

    bool getNextPCDRaw(timestamp_t &timestamp, PCDRaw &pcdRaw);

public:

    static constexpr size_t ROI_ROW_LOWER = 256 - 168;
    static constexpr size_t ROI_ROW_UPPER = 256 + 80;
    static constexpr size_t ROI_COL_LOWER = 256 - 128;
    static constexpr size_t ROI_COL_UPPER = 256 + 128;
    static constexpr uint16_t DEPTH_NEAR_CLIP = 200;  // Unit: mm
    static constexpr uint16_t DEPTH_FAR_CLIP = 800;
    static constexpr uint16_t DEPTH_FILTER_OFFSET = 200;

    static constexpr size_t CLIPPED_DEPTH_FRAME_WIDTH = ROI_COL_UPPER - ROI_COL_LOWER;
    static constexpr size_t CLIPPED_DEPTH_FRAME_HEIGHT = ROI_ROW_UPPER - ROI_ROW_LOWER;

    std::vector<AHATDepth> clippedDepthFrames;
    std::vector<float> clippedDepthMovingSum;
    size_t stdLogIndex = 0;
    static constexpr size_t STD_LOG_SIZE = 10;

    static constexpr float MAX_STD_VAL = 15000.0f; // squared value on purpose


    std::queue<std::pair<timestamp_t, PCDRaw>> pcdRawFrames;
    std::mutex pcdMutex;

    DirectX::XMMATRIX extrinsics;
    DirectX::XMMATRIX cam2rig;

    std::vector<DirectX::XMVECTOR> lut;

    std::vector<DirectX::XMVECTOR> handMesh[HandIndexCount];
    std::vector<float> handFilterDistanceSq[HandIndexCount];  // squared distances

    // @formatter:off
    static constexpr int HAND_BONES[][2] = {
        /* Thumb  */ {1, 2 }, /**/ {2, 3  }, /**/ {3, 4  }, /**/ {4, 5  },
        /* Index  */ {1, 6 }, /**/ {6, 7  }, /**/ {7, 8  }, /**/ {8, 9  }, /**/ {9, 10 },
        /* Middle */ {1, 11}, /**/ {11, 12}, /**/ {12, 13}, /**/ {13, 14}, /**/ {14, 15},
        /* Ring   */ {1, 16}, /**/ {16, 17}, /**/ {17, 18}, /**/ {18, 19}, /**/ {19, 20},
        /* Pinky  */ {1, 21}, /**/ {21, 22}, /**/ {22, 23}, /**/ {23, 24}, /**/ {24, 25}
    };
    // @formatter:on

    static constexpr float WRIST_RADIUS = 0.05f;
    static constexpr float FINGER_RADIUS = 0.025f;
    static constexpr float FINGER_SIZES[HandJointIndexCount] = {
            WRIST_RADIUS, WRIST_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS,
            FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS,
            FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS,
            FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS, FINGER_RADIUS,
    };
    static_assert(sizeof(DepthProcessor::FINGER_SIZES) == HandJointIndexCount * sizeof(float), "FINGER_SIZES");


    bool updateHandMesh(const Hand &hand, std::vector<DirectX::XMVECTOR> &mesh, std::vector<float> &filterDistanceSq);

    bool inHandMesh(const DirectX::XMVECTOR &point);
};


#endif //HOLOSCANNER_DEPTHPROCESSOR_H
