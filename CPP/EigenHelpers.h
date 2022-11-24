//
// Created by Zikai Liu on 11/14/22.
//

#pragma once

#include <Eigen/Eigen>
#include <DirectXMath.h>

static inline Eigen::Vector3d XMVectorToEigenVector3d(const DirectX::XMVECTOR &v) {
    DirectX::XMFLOAT4 f;
    DirectX::XMStoreFloat4(&f, v);
    // NOTICE: negated z
    return Eigen::Vector3d{f.x, f.y, -f.z} / f.w;
}