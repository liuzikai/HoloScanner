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

bool DepthProcessor::getNextHandDebugFrame(timestamp_t &timestamp, HandDebugFrame &hdFrame) {
    std::lock_guard<std::mutex> lock(pcdMutex);
    if (handMeshDebugQueue.empty()) return false;

    hdFrame = std::move(handMeshDebugQueue.front());
    timestamp = hdFrame.timestamp;
    handMeshDebugQueue.pop();
    return true;
}

void DepthProcessor::wristNormals(
        const DirectX::XMVECTOR &wrist,
        const DirectX::XMVECTOR &palm,
        const DirectX::XMVECTOR &indexMetacarpal,
        const DirectX::XMVECTOR &pinkyMetacarpal,
        float dist,
        bool isRightHand,
        DirectX::XMVECTOR &nDir,
        DirectX::XMVECTOR &nTDir,
        std::vector<DirectX::XMVECTOR> &vertices,
        std::vector<std::vector<int>> &indices,
        std::vector<float> &filterDistanceSq
) {
    DirectX::XMVECTOR temp = wrist - palm;
    DirectX::XMVECTOR backSide_1 = wrist + 1.0f * temp;
    DirectX::XMVECTOR backSide_2 = wrist + 2.0f * temp;
    DirectX::XMVECTOR backSide_3 = wrist + 3.0f * temp;

    DirectX::XMVECTOR dirIndexMetacarpal = indexMetacarpal - wrist;
    DirectX::XMVECTOR dirPinkyMetacarpal = pinkyMetacarpal - wrist;

    if (isRightHand) {
        nDir = DirectX::XMVector3Cross(dirPinkyMetacarpal, dirIndexMetacarpal);
    } else {
        nDir = DirectX::XMVector3Cross(dirIndexMetacarpal, dirPinkyMetacarpal);
    }
    nDir = DirectX::XMVector3Normalize(nDir);
    nTDir = DirectX::XMVector3Normalize(DirectX::XMVector3Cross(nDir, temp));

    size_t startS = vertices.size();
    vertices.push_back(wrist);
    vertices.push_back(palm);
    vertices.push_back(backSide_1);
    vertices.push_back(backSide_2);
    vertices.push_back(backSide_3);
    vertices.push_back(wrist + dist * nDir);
    vertices.push_back(wrist - dist * nDir);
    vertices.push_back(wrist + dist * nTDir);
    vertices.push_back(wrist - dist * nTDir);
    vertices.push_back(backSide_1 + dist * nDir);
    vertices.push_back(backSide_1 - dist * nDir);
    vertices.push_back(backSide_1 + dist * nTDir);
    vertices.push_back(backSide_1 - dist * nTDir);
    vertices.push_back(backSide_2 + dist * nDir);
    vertices.push_back(backSide_2 - dist * nDir);
    vertices.push_back(backSide_2 + dist * nTDir);
    vertices.push_back(backSide_2 - dist * nTDir);
    vertices.push_back(backSide_3 + dist * nDir);
    vertices.push_back(backSide_3 - dist * nDir);
    vertices.push_back(backSide_3 + dist * nTDir);
    vertices.push_back(backSide_3 - dist * nTDir);
    vertices.push_back(palm + dist * nDir);
    size_t endS = vertices.size();

    // add the squared distance for each new vertex
    float sqDistance = dist * dist;
    for (auto i = startS; i < endS; ++i) {
        filterDistanceSq.push_back(sqDistance);
    }

    int offset = static_cast<int>(startS);
    indices.push_back({0 + offset, 2 + offset});
    indices.push_back({2 + offset, 3 + offset});
    indices.push_back({3 + offset, 4 + offset});
    indices.push_back({0 + offset, 5 + offset});
    indices.push_back({0 + offset, 6 + offset});
    indices.push_back({0 + offset, 7 + offset});
    indices.push_back({0 + offset, 8 + offset});
    indices.push_back({2 + offset, 9 + offset});
    indices.push_back({2 + offset, 10 + offset});
    indices.push_back({2 + offset, 11 + offset});
    indices.push_back({2 + offset, 12 + offset});
    indices.push_back({3 + offset, 13 + offset});
    indices.push_back({3 + offset, 14 + offset});
    indices.push_back({3 + offset, 15 + offset});
    indices.push_back({3 + offset, 16 + offset});
    indices.push_back({4 + offset, 17 + offset});
    indices.push_back({4 + offset, 18 + offset});
    indices.push_back({4 + offset, 19 + offset});
    indices.push_back({4 + offset, 20 + offset});
    indices.push_back({1 + offset, 21 + offset});
}

void DepthProcessor::fingerNormals(
        const DirectX::XMVECTOR &tip,
        const DirectX::XMVECTOR &distal,
        const DirectX::XMVECTOR &intermediate,
        const DirectX::XMVECTOR &proximal,
        const DirectX::XMVECTOR &nDir,
        const DirectX::XMVECTOR &nTDir,
        float dist,
        bool flip,
        bool indexOrPinky,
        std::vector<DirectX::XMVECTOR> &vertices,
        std::vector<std::vector<int>> &indices,
        std::vector<float> &filterDistanceSq
) {
    DirectX::XMVECTOR dirTip = tip - distal;
    dirTip = DirectX::XMVector3Normalize(dirTip);
    DirectX::XMVECTOR dirDistal = distal - intermediate;
    dirDistal = DirectX::XMVector3Normalize(dirDistal);
    DirectX::XMVECTOR dirIntermediate = intermediate - proximal;
    dirIntermediate = DirectX::XMVector3Normalize(dirIntermediate);

    DirectX::XMVECTOR nTip = DirectX::XMVector3Cross(nTDir, dirTip);
    DirectX::XMVECTOR nDistal = DirectX::XMVector3Cross(nTDir, dirDistal);
    DirectX::XMVECTOR nIntermediate = DirectX::XMVector3Cross(nTDir, dirIntermediate);

    size_t startS = vertices.size();
    vertices.push_back(distal);
    vertices.push_back(intermediate);
    vertices.push_back(proximal);
    vertices.push_back(distal + dist * nDistal);
    vertices.push_back(intermediate + dist * nIntermediate);
    vertices.push_back(proximal + dist * nDir);

    int offset = static_cast<int>(startS);
    indices.push_back({0 + offset, 3 + offset});
    indices.push_back({1 + offset, 4 + offset});
    indices.push_back({2 + offset, 5 + offset});

    if (indexOrPinky) {
        if (flip) {
            dist = -dist;
        }

        vertices.push_back(distal + dist * nTDir);
        vertices.push_back(intermediate + dist * nTDir);
        vertices.push_back(proximal + dist * nTDir);

        indices.push_back({0 + offset, 6 + offset});
        indices.push_back({1 + offset, 7 + offset});
        indices.push_back({2 + offset, 8 + offset});
    }

    size_t endS = vertices.size();

    // add the squared distance for each new vertex
    float sqDistance = dist * dist;
    for (auto i = startS; i < endS; ++i) {
        filterDistanceSq.push_back(sqDistance);
    }
}

void DepthProcessor::thumbNormals(
        const DirectX::XMVECTOR &tip,
        const DirectX::XMVECTOR &distal,
        const DirectX::XMVECTOR &proximal,
        const DirectX::XMVECTOR &nTDir,
        float dist,
        bool isRightHand,
        std::vector<DirectX::XMVECTOR> &vertices,
        std::vector<std::vector<int>> &indices,
        std::vector<float> &filterDistanceSq
) {
    if (isRightHand) {
        dist = -dist;
    }

    size_t startS = vertices.size();
    vertices.push_back(distal);
    vertices.push_back(proximal);
    vertices.push_back(distal + dist * nTDir);
    vertices.push_back(proximal + dist * nTDir);

    int offset = static_cast<int>(startS);
    indices.push_back({0 + offset, 2 + offset});
    indices.push_back({1 + offset, 3 + offset});

    size_t endS = vertices.size();

    // add the squared distance for each new vertex
    float sqDistance = dist * dist;
    for (auto i = startS; i < endS; ++i) {
        filterDistanceSq.push_back(sqDistance);
    }
}

void DepthProcessor::updateHandMesh(
        const Hand &hand,
        std::vector<DirectX::XMVECTOR> &vertices,
        std::vector<std::vector<int>> &indices,
        std::vector<float> &filterDistanceSq
) {
    if (!hand.strictlyTracked) return;

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
        for (int s = 0; s <= sCount; ++s) {
            float x = (float) s * delta;
            vertices.push_back(origin + x * boneDirection);
            if (s != 0) {
                indices.push_back({static_cast<int>(vertices.size()) - 2, static_cast<int>(vertices.size()) - 1});
            }
            float percent = 1.0f / boneLength * x;
            filterDistanceSq.push_back(pow2(d1 * (1.0f - percent) + d2 * percent));
        }
    }
}

bool DepthProcessor::inHandMesh(const XMVECTOR &point) {
    for (int h = 0; h < HandIndexCount; h++) {
        size_t size = handMesh[h].size();
        for (int i = 0; i < size; i++) {
            float distSq = XMVectorGetX(XMVector3LengthSq(point - handMesh[h][i]));
            if (distSq < handFilterDistanceSq[h][i]) return true;
        }
    }
    return false;
}

bool DepthProcessor::update(const RawDataFrame &input) {
    if (!input.hands[Left].strictlyTracked || !input.hands[Right].strictlyTracked) {
        return false;  // tell the user of dropping frame
    }

    if (pcdRawFrames.size() > MAX_PENDING_FRAMES) return true;

    handMesh[Left].clear();
    handMesh[Right].clear();
    handMeshIndices[Left].clear();
    handMeshIndices[Right].clear();
    handFilterDistanceSq[Left].clear();
    handFilterDistanceSq[Right].clear();

    updateHandMesh(input.hands[Left], handMesh[Left], handMeshIndices[Left], handFilterDistanceSq[Left]);
    updateHandMesh(input.hands[Right], handMesh[Right], handMeshIndices[Right], handFilterDistanceSq[Right]);

    // normal vectors on left and right wrists
    DirectX::XMVECTOR lnDir;
    DirectX::XMVECTOR lnTDir;
    DirectX::XMVECTOR rnDir;
    DirectX::XMVECTOR rnTDir;
    const HandJoint *lhJoints = input.hands[HandIndex::Left].joints;
    const HandJoint *rhJoints = input.hands[HandIndex::Right].joints;

    if (input.hands[Left].strictlyTracked) {
        // left wrist
        wristNormals(
                lhJoints[HandJointIndex::Wrist].translationInRig,
                lhJoints[HandJointIndex::Palm].translationInRig,
                lhJoints[HandJointIndex::IndexMetacarpal].translationInRig,
                lhJoints[HandJointIndex::PinkyMetacarpal].translationInRig,
                WRIST_RADIUS, false, lnDir, lnTDir, handMesh[Left], handMeshIndices[Left],
                handFilterDistanceSq[Left]
        );

        //// left index finger
        //fingerNormals(
        //        lhJoints[HandJointIndex::IndexTip].translationInRig,
        //        lhJoints[HandJointIndex::IndexDistal].translationInRig,
        //        lhJoints[HandJointIndex::IndexIntermediate].translationInRig,
        //        lhJoints[HandJointIndex::IndexProximal].translationInRig,
        //        lnDir, lnTDir, INDEX_RADIUS, false, true,
        //        handMesh[Left], handMeshIndices[Left],
        //        handFilterDistanceSq[Left]
        //);

        // left middle finger
        fingerNormals(
                lhJoints[HandJointIndex::MiddleTip].translationInRig,
                lhJoints[HandJointIndex::MiddleDistal].translationInRig,
                lhJoints[HandJointIndex::MiddleIntermediate].translationInRig,
                lhJoints[HandJointIndex::MiddleProximal].translationInRig,
                lnDir, lnTDir, FINGER_RADIUS, false, false,
                handMesh[Left], handMeshIndices[Left],
                handFilterDistanceSq[Left]
        );

        // left ring finger
        fingerNormals(
                lhJoints[HandJointIndex::RingTip].translationInRig,
                lhJoints[HandJointIndex::RingDistal].translationInRig,
                lhJoints[HandJointIndex::RingIntermediate].translationInRig,
                lhJoints[HandJointIndex::RingProximal].translationInRig,
                lnDir, lnTDir, FINGER_RADIUS, false, false,
                handMesh[Left], handMeshIndices[Left],
                handFilterDistanceSq[Left]
        );

        // left pinky finger
        fingerNormals(
                lhJoints[HandJointIndex::PinkyTip].translationInRig,
                lhJoints[HandJointIndex::PinkyDistal].translationInRig,
                lhJoints[HandJointIndex::PinkyIntermediate].translationInRig,
                lhJoints[HandJointIndex::PinkyProximal].translationInRig,
                lnDir, lnTDir, FINGER_RADIUS, true, true,
                handMesh[Left], handMeshIndices[Left],
                handFilterDistanceSq[Left]
        );

        // left thumb
        thumbNormals(
                lhJoints[HandJointIndex::ThumbTip].translationInRig,
                lhJoints[HandJointIndex::ThumbDistal].translationInRig,
                lhJoints[HandJointIndex::ThumbProximal].translationInRig,
                lnTDir, THUMB_RADIUS, false,
                handMesh[Left], handMeshIndices[Left],
                handFilterDistanceSq[Left]
        );
    }

    if (input.hands[Right].strictlyTracked) {
        // right wrist
        wristNormals(
                rhJoints[HandJointIndex::Wrist].translationInRig,
                rhJoints[HandJointIndex::Palm].translationInRig,
                rhJoints[HandJointIndex::IndexMetacarpal].translationInRig,
                rhJoints[HandJointIndex::PinkyMetacarpal].translationInRig,
                WRIST_RADIUS, true, rnDir, rnTDir, handMesh[Right], handMeshIndices[Right],
                handFilterDistanceSq[Right]
        );

        //// right index finger
        //fingerNormals(
        //        rhJoints[HandJointIndex::IndexTip].translationInRig,
        //        rhJoints[HandJointIndex::IndexDistal].translationInRig,
        //        rhJoints[HandJointIndex::IndexIntermediate].translationInRig,
        //        rhJoints[HandJointIndex::IndexProximal].translationInRig,
        //        rnDir, rnTDir, INDEX_RADIUS, true, true,
        //        handMesh[Right], handMeshIndices[Right],
        //        handFilterDistanceSq[Right]
        //);

        // right middle finger
        fingerNormals(
                rhJoints[HandJointIndex::MiddleTip].translationInRig,
                rhJoints[HandJointIndex::MiddleDistal].translationInRig,
                rhJoints[HandJointIndex::MiddleIntermediate].translationInRig,
                rhJoints[HandJointIndex::MiddleProximal].translationInRig,
                rnDir, rnTDir, FINGER_RADIUS, true, false,
                handMesh[Right], handMeshIndices[Right],
                handFilterDistanceSq[Right]
        );

        // right ring finger
        fingerNormals(
                rhJoints[HandJointIndex::RingTip].translationInRig,
                rhJoints[HandJointIndex::RingDistal].translationInRig,
                rhJoints[HandJointIndex::RingIntermediate].translationInRig,
                rhJoints[HandJointIndex::RingProximal].translationInRig,
                rnDir, rnTDir, FINGER_RADIUS, true, false,
                handMesh[Right], handMeshIndices[Right],
                handFilterDistanceSq[Right]
        );

        // right pinky finger
        fingerNormals(
                rhJoints[HandJointIndex::PinkyTip].translationInRig,
                rhJoints[HandJointIndex::PinkyDistal].translationInRig,
                rhJoints[HandJointIndex::PinkyIntermediate].translationInRig,
                rhJoints[HandJointIndex::PinkyProximal].translationInRig,
                rnDir, rnTDir, FINGER_RADIUS, false, true,
                handMesh[Right], handMeshIndices[Right],
                handFilterDistanceSq[Right]
        );

        // right thumb
        thumbNormals(
                rhJoints[HandJointIndex::ThumbTip].translationInRig,
                rhJoints[HandJointIndex::ThumbDistal].translationInRig,
                rhJoints[HandJointIndex::ThumbProximal].translationInRig,
                rnTDir, THUMB_RADIUS, true,
                handMesh[Right], handMeshIndices[Right],
                handFilterDistanceSq[Right]
        );
    }

    //input.hands[HandIndex::Left].joints[HandJointIndex::Wrist].translationInRig

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

            // Filter on std deviation and depth clipping
            if (dev < MAX_STD_VAL && depth > depthNearClip && depth < depthFarClip) {
            //if (depth > depthNearClip && depth < depthFarClip) {
                auto pointInCam = ((float) depth / 1000.0f) * lut[ind];
                auto pointInRig = XMVector3Transform(pointInCam, cam2rig);
                pointInRig /= XMVectorGetW(pointInRig);

                // Filter by hand mesh
                bool inMesh = inHandMesh(pointInRig);
                if (!inMesh) {
                    XMFLOAT3 f;
                    XMStoreFloat3(&f, pointInRig);
                    frame.second.push_back(f.x);
                    frame.second.push_back(f.y);
                    frame.second.push_back(-f.z);
                }
            }
        }
    }

    // Save the data
    {
        std::lock_guard<std::mutex> lock(pcdMutex);
        pcdRawFrames.emplace(std::move(frame));
        auto mPair = std::make_pair(frame.first, handMesh);
        auto iPair = std::make_pair(frame.first, handMeshIndices);

        // not sure about all the moves here...
        HandDebugFrame hdFrame;
        hdFrame.timestamp = frame.first;
        hdFrame.lhMesh = std::move(handMesh[Left]);
        hdFrame.lhIndices = std::move(handMeshIndices[Left]);
        hdFrame.rhMesh = std::move(handMesh[Right]);
        hdFrame.rhIndices = std::move(handMeshIndices[Right]);

        handMeshDebugQueue.emplace(std::move(hdFrame));
    }

    return true;
}
