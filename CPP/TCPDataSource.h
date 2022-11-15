//
// Created by Zikai Liu on 11/12/22.
//

#ifndef HOLOSCANNER_TCPDATASOURCE_H
#define HOLOSCANNER_TCPDATASOURCE_H

#include "DepthDataTypes.h"
#include "InteractionDataTypes.h"
#include "PCDDataTypes.h"
#include "TerminalSocket.h"
#include <queue>

class TCPDataSource : public AHATSource, public InteractionSource, public PCDSource {
public:
    explicit TCPDataSource();

    ~TCPDataSource();

    bool getAHATExtrinsics(DirectX::XMMATRIX &extrinsics) override;

    bool getAHATDepthLUT(AHATLUT &lut) override;

    bool getNextAHATFrame(timestamp_t &timestamp, AHATDepth &depth, DirectX::XMMATRIX &rig2world) override;

    bool getNextInteractionFrame(timestamp_t &timestamp, InteractionFrame &frame) override;

    bool getNextPCD(timestamp_t &timestamp, PCD &pcd) override;

private:

    static constexpr int PORT = 9090;

    boost::asio::io_context tcpIOContext;
    std::thread *tcpIOThread = nullptr;

    TerminalSocketServer socketServer;

    void handleRecvBytes(std::string_view name, const uint8_t *buf, size_t size);

    bool ahatExtrinsicsValid = false;
    DirectX::XMMATRIX ahatExtrinsics;
    std::vector<float> ahatLUT;
    std::mutex ahatStaticDataMutex;

    std::queue<AHATFrame> ahatFrames;
    std::mutex ahatFrameMutex;

    std::queue<std::pair<timestamp_t, PCD>> pcdFrames;
    std::mutex pcdMutex;

    std::queue<std::pair<timestamp_t, InteractionFrame>> interactionFrames;
    std::mutex interactionMutex;
};


#endif //HOLOSCANNER_TCPDATASOURCE_H
