//
// Created by Zikai Liu on 11/14/22.
//

#pragma once

#include <Eigen/Eigen>
#include <DirectXMath.h>

static inline DirectX::XMVECTOR XMTransformToTranslate(const DirectX::XMMATRIX &m) {
    return m.r[3];  // see XMMatrixDecompose
}

static inline Eigen::Vector3d XMVectorToEigenVector3d(const DirectX::XMVECTOR &v) {
    DirectX::XMFLOAT3 f;
    DirectX::XMStoreFloat3(&f, v);
    // NOTICE: negated z
    return Eigen::Vector3d{f.x, f.y, -f.z};
}