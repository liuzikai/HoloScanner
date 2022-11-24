//
// Created by Zikai Liu on 11/23/22.
//

#pragma once

#include <DirectXMath.h>

static inline DirectX::XMVECTOR XMTransformToTranslate(const DirectX::XMMATRIX &m) {
    return m.r[3];  // see XMMatrixDecompose
}

static inline DirectX::XMVECTOR XMVectorMidpoint(const DirectX::XMVECTOR &a, const DirectX::XMVECTOR &b) {
    static const DirectX::XMVECTOR HALF = DirectX::XMVectorSet(0.5f, 0.5f, 0.5f, 0.5f);
    return DirectX::XMVectorMultiply(DirectX::XMVectorAdd(a, b), HALF);
}
