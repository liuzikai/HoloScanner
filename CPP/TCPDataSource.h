//
// Created by Zikai Liu on 11/12/22.
//

#ifndef HOLOSCANNER_TCPDATASOURCE_H
#define HOLOSCANNER_TCPDATASOURCE_H

#include "RawDataTypes.h"
#include "PCDDataTypes.h"
#include "TerminalSocket.h"
#include <queue>

class TCPDataSource : public RawDataSource, public PCDSource {
public:
    explicit TCPDataSource();

    ~TCPDataSource();

    bool getAHATExtrinsics(DirectX::XMMATRIX &extrinsics) override;

    bool getAHATDepthLUT(AHATLUT &lut) override;

    bool getNextRawDataFrame(RawDataFrame &frame) override;

    bool getNextPCD(timestamp_t &timestamp, PCD &pcd) override;

    bool sendReconstructedPCD(const Eigen::RowVector3d &pointColor, const PCD &pcd,
                              const DirectX::XMMATRIX &rig2world) override;

    bool receivedStopSignal() const { return m_stopSignalReceived; }

    void resetStopSignal() {
        m_stopSignalReceived = false;
    }

private:

    static constexpr int PORT = 9090;
    static constexpr int MAX_PENDING_FRAMES = 3;

    boost::asio::io_context tcpIOContext;
    std::thread *tcpIOThread = nullptr;

    TerminalSocketServer socketServer;

    void handleRecvBytes(std::string_view name, const uint8_t *buf, size_t size);

    bool ahatExtrinsicsValid = false;
    DirectX::XMMATRIX ahatExtrinsics;
    std::vector<float> ahatLUT;
    std::mutex ahatStaticDataMutex;

    std::queue<RawDataFrame> rawDataFrames;
    std::mutex rawDataFrameMutex;

    std::queue<std::pair<timestamp_t, PCD>> pcdFrames;
    std::mutex pcdMutex;
	
	bool m_stopSignalReceived = false;
};


#endif //HOLOSCANNER_TCPDATASOURCE_H
