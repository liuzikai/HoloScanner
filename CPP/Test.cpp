//
// Created by Zikai Liu on 11/12/22.
//

//#include <DirectXMath.h>

#include <igl/read_triangle_mesh.h>
#include <igl/readTGF.h>
#include <igl/readDMAT.h>
#include <igl/opengl/glfw/Viewer.h>

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/slice.h>

#include <iostream>
//using namespace DirectX;

using Viewer = igl::opengl::glfw::Viewer;

int main() {

    /*auto m = XMMatrixIdentity();
    XMFLOAT4X4 fView;
    XMStoreFloat4x4(&fView, m);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::cout << fView.m[i][j] << " ";
        }
        std::cout << std::endl;
    }*/

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    // Load a mesh in OFF format
    igl::readOFF("/Users/liuzikai/Downloads/bunny.off", V, F);

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.launch();
}