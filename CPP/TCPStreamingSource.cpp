//
// Created by Zikai Liu on 11/12/22.
//

#include "TCPStreamingSource.h"
#include <iostream>

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
    tcpIOThread->join();
}

bool TCPStreamingSource::getNextPCDSince(timestamp_t &timestamp, PCD &pcd) {
    std::lock_guard<std::mutex> lock(mu);
    if (PCDs.empty()) return false;
    timestamp = PCDs.front().first;
    pcd = std::move(PCDs.front().second);
    PCDs.pop();
    return true;
}

void TCPStreamingSource::handleRecvBytes(std::string_view name, const uint8_t *buf, size_t size) {
    if (name == "p") {  // point cloud
        std::pair<timestamp_t, PCD> frame;

        memcpy(&frame.first, buf, sizeof(timestamp_t));

        buf += sizeof(timestamp_t);
        size -= sizeof(timestamp_t);
        if (size % 12 != 0) {
            std::cerr << "Invalid point cloud size: " << size << std::endl;
            return;
        }
        size_t count = size / 12;
        frame.second.reserve(count);
        auto fbuf = (const float *) buf;
        for (size_t i = 0; i < count; i++) {
            frame.second.emplace_back(fbuf[0], fbuf[1], fbuf[2]);
            fbuf += 3;
        }

        {
            std::lock_guard<std::mutex> lock(mu);
            PCDs.emplace(std::move(frame));
        }
    }
}
