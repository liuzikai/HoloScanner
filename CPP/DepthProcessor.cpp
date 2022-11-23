//
// Created by Zikai Liu on 11/14/22.
//

#include "DepthProcessor.h"
//#include "math.h"
#include <cmath>

using namespace DirectX;

DepthProcessor::DepthProcessor(const DirectX::XMMATRIX& extrinsics, const float* lutBuf)
    : extrinsics(extrinsics), depthCameraPoseInvMatrix(XMMatrixInverse(nullptr, extrinsics)) {

    // Store LUT as XMVector
    //lut.reserve(AHAT_WIDTH * AHAT_HEIGHT);
    lut.reserve(clippedDepthFrameHeight * clippedDepthFrameWidth);
    //for (size_t i = 0; i < AHAT_HEIGHT; i++) {
    for (size_t i = roiRowLower; i < roiRowUpper; i++) {
        //for (size_t j = 0; j < AHAT_WIDTH; j++) {
        for (size_t j = roiColLower; j < roiColUpper; j++) {
            lut.emplace_back(XMLoadFloat3((const XMFLOAT3*)lutBuf));
            lutBuf += 3;
        }
    }

    // initialize std deviation back buffer
    for (auto i = 0; i < stdLogSize; ++i) {
        stdDepthVec.push_back(std::vector<float>(clippedDepthFrameWidth * clippedDepthFrameHeight), 0.0f);
    }

    depthMovingAverage.resize(clippedDepthFrameWidth * clippedDepthFrameHeight, 0.0f);
    stdVal.resize(clippedDepthFrameWidth * clippedDepthFrameHeight, 0.0f);

    handBones = {
        /* Thumb  */ {1, 2 }, /**/ {2, 3  }, /**/ {3, 4  }, /**/ {4, 5  },
        /* Index  */ {1, 6 }, /**/ {6, 7  }, /**/ {7, 8  }, /**/ {8, 9  }, /**/ {9, 10 },
        /* Middle */ {1, 11}, /**/ {11, 12}, /**/ {12, 13}, /**/ {13, 14}, /**/ {14, 15},
        /* Ring   */ {1, 16}, /**/ {16, 17}, /**/ {17, 18}, /**/ {18, 19}, /**/ {19, 20},
        /* Pinky  */ {1, 21}, /**/ {21, 22}, /**/ {22, 23}, /**/ {23, 24}, /**/ {24, 25}
    };

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
    fingerSizes = { WRIST_RADIUS, WRIST_RADIUS };
    for (auto i = 0; i < HandJointIndexCount - 2; ++i) {
        fingerSize.push_back(FINGER_RADIUS);
    }
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

int DepthProcessor::countTrackedJoints(const Hand& hand) {
    int result = 0;
    for (const auto& joint : hand.joints) {
        if (joint.tracked) result++;
    }
    return result;
}

//def create_hand_mesh(self, joints, bones, distances) :
//    mesh = []
//    new_distances = []
//    for b in bones :
    //    j1 = b[0]
    //    j2 = b[1]
    //    d1 = distances[j1]
    //    d2 = distances[j2]
    //    origin = joints[j1]
    //    bone_dir = joints[j2] - origin
    //    bone_length = np.linalg.norm(bone_dir)

    //    if bone_length < np.finfo(dtype = np.float32).eps:
    //    continue

    //    bone_dir /= bone_length
    //    d = min(d1, d2)
    //    seg_count = math.ceil(bone_length / d)

    //    if seg_count == 0:
    //    continue

    //    delta = bone_length / seg_count
    //    for i in range(1, seg_count) :
    //        mesh.append(origin + i * delta * bone_dir)
    //        percent = 1.0 / bone_length * i * delta
    //        new_distances.append(d1 * (1 - percent) + d2 * percent)

//   for j, d in zip(joints, distances) :
//       mesh.append(j)
//       new_distances.append(d)

//       return np.array(mesh, dtype = np.float32), new_distances

//void createHandMesh(const InteractionFrame& hand, bool &lhTracked, bool &rhTracked) {
bool createHandMesh(const Hand & hand, std::vector<DirectX::XMVECTOR> &mesh, std::vector<float> &distances);
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

    //bool leftTracked = interactionFrame.hands[Left].tracked && countTrackedJoints(interactionFrame.hands[Left]);
    //bool rightTracked = interactionFrame.hands[Right].tracked && countTrackedJoints(interactionFrame.hands[Right]);
    //if (!leftTracked || !rightTracked) return; //lostTracking = true;
    
    bool lhTracked = createHandMesh(hand.hands->joints[HandIndex::Left], lhMesh, lhDistances);
    bool rhTracked = createHandMesh(hand.hands->joints[HandIndex::Right], rhMesh, rhDistances);

    if (!lhTracked || !rhTracked) return;

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
            if (cDev < MAX_STD_VAL && depth > depthNearClip && depth < depthFarClip) {
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
