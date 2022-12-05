//
// Created by Zikai Liu on 11/14/22.
//

#pragma once

#include "Timestamp.h"
#include <vector>
#include <Eigen/Eigen>
#include <DirectXMath.h>

using PCD = std::vector<Eigen::Vector3d>;  // point cloud data (Open3D use Vector3d not Vector3f)

class PCDSource {
public:

    virtual bool getNextPCD(timestamp_t &timestamp, PCD &pcd) = 0;

    /**
     * @brief Send back the reconstructed PCD to the device
     * 
     * @param pcd the reconstructed PCD
     * @param rig2world
     * @return true if successfully sent
     */
    // FIXME: should not be here
    virtual bool sendReconstructedPCD(const PCD &pcd, const DirectX::XMMATRIX &rig2world) = 0;
};