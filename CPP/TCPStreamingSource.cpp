//
// Created by Zikai Liu on 11/12/22.
//

#include "TCPStreamingSource.h"
#include <iostream>
#include <ctime>

TCPStreamingSource::TCPStreamingSource() :
        socketServer(tcpIOContext, PORT, [](auto s) {
            // Setup a server with automatic acceptance
            std::cout << "TerminalSocketServer: disconnected" << std::endl;
            s->startAccept();
        }),
        tcpIOThread(new std::thread([this] {
            boost::asio::executor_work_guard<boost::asio::io_context::executor_type> workGuard(
                    tcpIOContext.get_executor());
            tcpIOContext.run();  // this operation is blocking, until ioContext is deleted
        })) {

    socketServer.startAccept();
    socketServer.setCallbacks(nullptr,
                              nullptr,
                              [this](auto name, auto buf, auto size) { handleRecvBytes(name, buf, size); },
                              nullptr);
}

TCPStreamingSource::~TCPStreamingSource() {
    if (tcpIOThread) tcpIOThread->join();
}

bool TCPStreamingSource::getAHATExtrinsics(DirectX::XMMATRIX &extrinsics) {
    std::lock_guard<std::mutex> lock(ahatStaticDataMutex);
    if (!ahatExtrinsicsValid) return false;
    extrinsics = ahatExtrinsics;
    return true;
}

bool TCPStreamingSource::getAHATDepthLUT(AHATLUT &lut) {
    std::lock_guard<std::mutex> lock(ahatStaticDataMutex);
    if (ahatLUT.empty()) return false;
    lut = ahatLUT;
    return true;
}

bool TCPStreamingSource::getNextAHATFrame(timestamp_t &timestamp, AHATDepth &depth, DirectX::XMMATRIX &rig2world) {
    std::lock_guard<std::mutex> lock(ahatFrameMutex);
    if (ahatFrames.empty()) return false;
    timestamp = ahatFrames.front().timestamp;
    depth = std::move(ahatFrames.front().depth);
    rig2world = ahatFrames.front().rig2world;
    ahatFrames.pop();
    return true;
}

bool TCPStreamingSource::getNextInteractionFrame(timestamp_t &timestamp, InteractionFrame &frame) {
    std::lock_guard<std::mutex> lock(interactionMutex);
    if (interactionFrames.empty()) return false;
    timestamp = interactionFrames.front().first;
    frame = interactionFrames.front().second;
    interactionFrames.pop();
    return true;
}

bool TCPStreamingSource::getNextPCD(timestamp_t &timestamp, PCD &pcd) {
    std::lock_guard<std::mutex> lock(pcdMutex);
    if (pcdFrames.empty()) return false;
    timestamp = pcdFrames.front().first;
    pcd = std::move(pcdFrames.front().second);
    pcdFrames.pop();
    return true;
}

void TCPStreamingSource::handleRecvBytes(std::string_view name, const uint8_t *buf, size_t size) {

    if (name == "e") {  // AHAT extrinsics
        static constexpr size_t EXPECTED_SIZE = 16 * sizeof(float);
        if (size != EXPECTED_SIZE) {
            std::cerr << "Invalid AHAT extrinsics size: " << size << ", expecting " << EXPECTED_SIZE << std::endl;
            return;
        }

        // Save the data
        {
            std::lock_guard<std::mutex> lock(ahatStaticDataMutex);
            ahatExtrinsics = DirectX::XMLoadFloat4x4((const DirectX::XMFLOAT4X4 *) buf);
            ahatExtrinsicsValid = true;
        }

    } else if (name == "l") {  // AHAT LUT
        static constexpr size_t EXPECTED_SIZE = 3 * AHAT_WIDTH * AHAT_HEIGHT * sizeof(float);
        if (size != EXPECTED_SIZE) {
            std::cerr << "Invalid AHAT LUT size: " << size << ", expecting " << EXPECTED_SIZE << std::endl;
            return;
        }

        // Save the data
        {
            std::lock_guard<std::mutex> lock(ahatStaticDataMutex);
            auto fbuf = (const float *) buf;
            ahatLUT = std::vector<float>(fbuf, fbuf + (EXPECTED_SIZE / sizeof(float)));
        }

    } else if (name == "d") {  // AHAT depth
        AHATFrame frame;

        // TODO: use timestamp from HoloLens
        frame.timestamp = time(nullptr);

        static constexpr size_t EXPECTED_DEPTH_SIZE = AHAT_WIDTH * AHAT_HEIGHT * sizeof(uint16_t);
        static constexpr size_t EXPECTED_RIG2WORLD_SIZE = 16 * sizeof(float);
        if (size != EXPECTED_DEPTH_SIZE + EXPECTED_RIG2WORLD_SIZE) {
            std::cerr << "Invalid point cloud size: " << size << ", expecting "
                      << EXPECTED_DEPTH_SIZE + EXPECTED_RIG2WORLD_SIZE << std::endl;
            return;
        }

        auto ubuf = (const uint16_t *) buf;
        frame.depth.assign(ubuf, ubuf + (EXPECTED_DEPTH_SIZE / sizeof(uint16_t)));

        auto fbuf = (const float *) (buf + EXPECTED_DEPTH_SIZE);
        frame.rig2world = DirectX::XMLoadFloat4x4((const DirectX::XMFLOAT4X4 *) fbuf);

        // Save the data
        {
            std::lock_guard<std::mutex> lock(ahatFrameMutex);
            ahatFrames.emplace(std::move(frame));
        }
    } else if (name == "i") {  // interaction
        std::pair<timestamp_t, InteractionFrame> frame;

        // TODO: use timestamp from HoloLens
        frame.first = time(nullptr);

        static constexpr size_t EXPECTED_SIZE =
                (16 + HandIndexCount * (1 + HandJointIndexCount * (1 + 16)) + 1 + 4 + 4) * sizeof(float);
        if (size != EXPECTED_SIZE) {
            std::cerr << "Invalid interaction frame size: " << size << ", expecting " << EXPECTED_SIZE << std::endl;
            return;
        }

        auto &f = frame.second;
        auto fbuf = (const float *) buf;

        // Head
        f.headTransformation = DirectX::XMLoadFloat4x4((const DirectX::XMFLOAT4X4 *) fbuf);
        fbuf += 16;

        // Hands
        for (auto &hand: f.hands) {
            hand.tracked = (*fbuf == 1.0f);
            fbuf++;

            for (auto &joint: hand.joints) {
                joint.tracked = (*fbuf == 1.0f);
                fbuf++;

                joint.transformation = DirectX::XMLoadFloat4x4((const DirectX::XMFLOAT4X4 *) fbuf);
                fbuf += 16;
            }
        }

        // Eye
        f.eyeTracked = (*fbuf == 1.0f);
        fbuf++;

        f.eyeOrigin = DirectX::XMLoadFloat4((const DirectX::XMFLOAT4 *) fbuf);
        fbuf += 4;

        f.eyeDirection = DirectX::XMLoadFloat4((const DirectX::XMFLOAT4 *) fbuf);
        fbuf += 4;

        // Save the data
        {
            std::lock_guard<std::mutex> lock(interactionMutex);
            interactionFrames.emplace(std::move(frame));
        }

    } else if (name == "p") {  // point cloud
        std::pair<timestamp_t, PCD> frame;

        /* memcpy(&frame.first, buf, sizeof(timestamp_t));
        buf += sizeof(timestamp_t);
        size -= sizeof(timestamp_t); */
        // TODO: use timestamp from HoloLens
        frame.first = time(nullptr);

        if (size % 12 != 0) {
            std::cerr << "Invalid point cloud size: " << size << ", expecting multiples of 12" << std::endl;
            return;
        }
        size_t count = size / 12;
        frame.second.reserve(count);
        auto fbuf = (const float *) buf;
        for (size_t i = 0; i < count; i++) {
            frame.second.emplace_back(fbuf[0], fbuf[1], fbuf[2]);
            fbuf += 3;
        }

        // Save the data
        {
            std::lock_guard<std::mutex> lock(pcdMutex);
            pcdFrames.emplace(std::move(frame));
        }
    }
}
