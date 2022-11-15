//
// Created by Zikai Liu on 11/12/22.
//

#include <igl/read_triangle_mesh.h>
#include <igl/readTGF.h>
#include <igl/readDMAT.h>
#include <igl/opengl/glfw/Viewer.h>

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/slice.h>

#include <iostream>

#include "TCPDataSource.h"
#include "DepthProcessor.h"
#include "Adapters.h"
#include "FileDataSource.h"

class DepthProcessorWrapper : public DepthProcessor, public PCDSource {
public:

    DepthProcessorWrapper(const DirectX::XMMATRIX &extrinsics, const float *lut)
            : DepthProcessor(extrinsics, lut) {}

    bool getNextPCD(timestamp_t &timestamp, PCD &pcd) override {
        PCDRaw pcdRaw;
        if (DepthProcessor::getNextPCDRaw(timestamp, pcdRaw)) {
            assert(pcdRaw.size() % 3 == 0 && "PCDRaw does not contain not flatten 3D vectors");
            pcd.clear();
            pcd.reserve(pcdRaw.size() / 3);
            for (size_t i = 0; i < pcdRaw.size() / 3; i++) {
                pcd.emplace_back(pcdRaw[i * 3], pcdRaw[i * 3 + 1], pcdRaw[i * 3 + 2]);
            }
            return true;
        } else {
            return false;
        }
    }
};

std::unique_ptr<FileDataSource> fileDataSource;

TCPDataSource tcpStreamingSource;

AHATSource *ahatSource = nullptr;
InteractionSource *interactionSource = nullptr;

std::unique_ptr<DepthProcessorWrapper> depthProcessor;

bool callBackPerDraw(igl::opengl::glfw::Viewer &viewer) {
    // static to hold the latest data
    static timestamp_t pcdTimestamp;
    static PCD pcd;

    static timestamp_t interationTimestamp;
    static InteractionFrame interactionFrame;

    if (!depthProcessor) {
        DirectX::XMMATRIX ahatExtrinsics;
        std::vector<float> ahatLUT;
        if (!ahatSource || !ahatSource->getAHATExtrinsics(ahatExtrinsics)) return false;
        if (!ahatSource || !ahatSource->getAHATDepthLUT(ahatLUT)) return false;
        depthProcessor = std::make_unique<DepthProcessorWrapper>(ahatExtrinsics, ahatLUT.data());
        std::cout << "Create DepthProcessor with AHAT extrinsics and LUT" << std::endl;
    }

    bool updated = false;

    timestamp_t ahatTimestamp;
    AHATDepth depth;
    DirectX::XMMATRIX rig2world;
    while (ahatSource && ahatSource->getNextAHATFrame(ahatTimestamp, depth, rig2world)) {
        /*std::cout << "[DEPTH] " << ahatTimestamp << std::endl;*/
        depthProcessor->updateAHAT(ahatTimestamp, depth.data(), rig2world);
    }

    while (depthProcessor->getNextPCD(pcdTimestamp, pcd)) {
        /*std::cout << "[PCD] " << pcdTimestamp << std::endl;*/
        updated = true;
    }

    while (interactionSource && interactionSource->getNextInteractionFrame(interationTimestamp, interactionFrame)) {
        /*std::cout << "[INT] " << interationTimestamp
                  << " Left Hand Tracked: " << interactionFrame.hands[Left].tracked
                  << " Right Hand Tracked: " << interactionFrame.hands[Right].tracked
                  << std::endl;*/
        updated = true;
    }

    if (updated) {
        viewer.data().clear();

        // PCD
        Eigen::MatrixXd points(pcd.size(), 3);
        for (int i = 0; i < pcd.size(); i++) {
            points.row(i) = pcd[i].cast<double>();
        }
        viewer.data().add_points(points, Eigen::RowVector3d(1, 1, 1));

        // Interaction
        static const Eigen::RowVector3d handColors[HandIndexCount] = {Eigen::RowVector3d(0, 0, 1),
                                                                      Eigen::RowVector3d(1, 0, 0)};
        for (int i = 0; i < HandIndexCount; i++) {
            if (!interactionFrame.hands[i].tracked) continue;

            Eigen::MatrixXd jointPoints((int) HandJointIndexCount, 3);
            for (int j = 0; j < HandJointIndexCount; j++) {
                jointPoints.row(j) = XMVectorToEigenVector3d(
                        XMTransformToTranslate(interactionFrame.hands[i].joints[j].transformation));
            }

            static const Eigen::MatrixXi edgesBetweenJoints = (Eigen::MatrixXi(24, 2) << 1, 2,
                    2, 3,
                    3, 4,
                    4, 5,

                    1, 6,
                    6, 7,
                    7, 8,
                    8, 9,
                    9, 10,

                    1, 11,
                    11, 12,
                    12, 13,
                    13, 14,
                    14, 15,

                    1, 16,
                    16, 17,
                    17, 18,
                    18, 19,
                    19, 20,

                    1, 21,
                    21, 22,
                    22, 23,
                    23, 24,
                    24, 25).finished();

//            std::cout << jointPoints << std::endl;

            viewer.data().set_edges(jointPoints, edgesBetweenJoints, handColors[i]);
        }

        // Set camera on first frame
        static bool is_first_frame = true;
        if (is_first_frame) {
            viewer.core().align_camera_center(points);
            is_first_frame = false;
        }
    }

    return false;
}

int main() {

    ahatSource = static_cast<AHATSource *>(&tcpStreamingSource);
    interactionSource = static_cast<InteractionSource *>(&tcpStreamingSource);

//    fileDataSource = std::make_unique<FileDataSource>("/Users/liuzikai/Downloads/2022-11-01-201827-AHAT-PV-EYE-Green-Book-Slow");
//    ahatSource = static_cast<AHATSource *>(fileDataSource.get());
//    interactionSource = static_cast<InteractionSource *>(fileDataSource.get());

    igl::opengl::glfw::Viewer viewer;
    viewer.callback_pre_draw = callBackPerDraw;
    viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
    viewer.data().point_size = 2;
    viewer.core().is_animating = true;
    Eigen::Vector4f color(1, 1, 1, 1);
    viewer.core().background_color = color * 0.2f;
    viewer.launch();
}