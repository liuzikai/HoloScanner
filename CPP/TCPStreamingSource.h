//
// Created by Zikai Liu on 11/12/22.
//

#ifndef HOLOSCANNER_TCPSTREAMINGSOURCE_H
#define HOLOSCANNER_TCPSTREAMINGSOURCE_H

#include "DataTypes.h"
#include "TerminalSocket.h"
#include <queue>

class TCPStreamingSource : public PCDSource {
public:
    explicit TCPStreamingSource();

    ~TCPStreamingSource();

    bool getNextPCDSince(timestamp_t &timestamp, PCD &pcd) override;

private:

    static constexpr int PORT = 9090;

    boost::asio::io_context tcpIOContext;
    std::thread *tcpIOThread = nullptr;

    TerminalSocketServer socketServer;

    void handleRecvBytes(std::string_view name, const uint8_t *buf, size_t size);

    std::queue<std::pair<long long, PCD>> PCDs;

    std::mutex mu;
};


#endif //HOLOSCANNER_TCPSTREAMINGSOURCE_H
