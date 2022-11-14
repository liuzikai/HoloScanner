//
// Created by Zikai Liu on 11/14/22.
//

#pragma once

#include "Timestamp.h"
#include <vector>
#include <DirectXMath.h>

static constexpr size_t AHAT_WIDTH = 512;
static constexpr size_t AHAT_HEIGHT = 512;

using AHATLUT = std::vector<float>;
using AHATDepth = std::vector<uint16_t>;

class AHATSource {
public:

    /**
     * Get the AHAT extrinsics matrix.
     * @param extrinsics  [out] the extrinsics matrix
     * @return true if the extrinsics is available, otherwise the output variable is not touched
     */
    virtual bool getAHATExtrinsics(DirectX::XMMATRIX &extrinsics) = 0;

    /**
     * Get the AHAT depth LUT (x/y/z coefficients to be timed with depth at each pixel to get the 3D point)
     * @param lut  [out] buffer of LUT in flatten row-major order (3 * AHAT_WIDTH * AHAT_HEIGHT)
     * @return true if the LUT is available, otherwise the output variable is not touched
     */
    virtual bool getAHATDepthLUT(AHATLUT &lut) = 0;

    /**
     * Get the next AHAT frame of depth and rig2world matrix.
     * @param timestamp  [out] timestamp of the frame
     * @param depth      [out] output depth in row-major order (AHAT_WIDTH * AHAT_HEIGHT)
     * @param rig2world  [out] rig to world matrix
     * @return true if there are new frame available, otherwise the output variables are not touched
     */
    virtual bool getNextAHATFrame(timestamp_t &timestamp, AHATDepth &depth, DirectX::XMMATRIX &rig2world) = 0;
};

using PCDRaw = std::vector<float>;
