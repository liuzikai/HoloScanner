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

bool discardDelayedFrames = false;

AHATSource *ahatSource = nullptr;
InteractionSource *interactionSource = nullptr;

std::unique_ptr<DepthProcessorWrapper> depthProcessor;

int countTrackedJoints(const Hand &hand) {
    int result = 0;
    for (const auto &joint: hand.joints) {
        if (joint.tracked) result++;
    }
    return result;
}

static const Eigen::RowVector3d HAND_COLOR[HandIndexCount] = {Eigen::RowVector3d(0, 1, 0),
                                                              Eigen::RowVector3d(1, 0, 0)};
Eigen::MatrixXd BOTH_HANDS_EDGE_COLORS(48, 3);

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

    bool redraw = false;
    bool lostTracking = false;

    if (ahatSource) {
        timestamp_t ahatTimestamp;
        AHATDepth depth;
        DirectX::XMMATRIX rig2world;

        do {
            if (!ahatSource->getNextAHATFrame(ahatTimestamp, depth, rig2world)) break;
            // std::cout << "[DEPTH] " << ahatTimestamp << std::endl;
            depthProcessor->updateAHAT(ahatTimestamp, depth.data(), rig2world);
        } while (discardDelayedFrames);  // continue the loop if discardDelayedFrames
    }

    if (depthProcessor) {
        do {
            if (!depthProcessor->getNextPCD(pcdTimestamp, pcd)) break;
            // std::cout << "[PCD] " << pcdTimestamp << std::endl;
//            redraw = true;
// FIXME: not trigger update for now
        } while (discardDelayedFrames);  // continue the loop if discardDelayedFrames
    }

    if (interactionSource) {
        do {
            timestamp_t newInterationTimestamp;
            InteractionFrame newInteractionFrame;
            if (!interactionSource->getNextInteractionFrame(newInterationTimestamp, newInteractionFrame)) break;
            std::cout << "[INT] "
                      << " L: " << newInteractionFrame.hands[Left].tracked << "/"
                      << countTrackedJoints(newInteractionFrame.hands[Left])
                      << " R: " << newInteractionFrame.hands[Right].tracked << "/"
                      << countTrackedJoints(newInteractionFrame.hands[Right])
                      << std::endl;
            redraw = true;

            if (newInteractionFrame.hands[Left].tracked && countTrackedJoints(newInteractionFrame.hands[Left]) == 0 ||
                newInteractionFrame.hands[Right].tracked && countTrackedJoints(newInteractionFrame.hands[Right]) == 0) {
                std::cout << "Discard frame on tracking" << std::endl;
                lostTracking = true;
                // should redraw but should not update interaction data
            } else {
                lostTracking = false;
            }

            if (!lostTracking) {
                interationTimestamp = newInterationTimestamp;
                interactionFrame = newInteractionFrame;
            }

        } while (discardDelayedFrames);  // continue the loop if discardDelayedFrames
    }

    bool leftTracked = interactionFrame.hands[Left].tracked && countTrackedJoints(interactionFrame.hands[Left]);
    bool rightTracked = interactionFrame.hands[Right].tracked && countTrackedJoints(interactionFrame.hands[Right]);
    if (!leftTracked || !rightTracked) lostTracking = true;

    if (redraw) {
        viewer.data().clear();

        // PCD
        Eigen::MatrixXd points(pcd.size(), 3);
        for (int i = 0; i < pcd.size(); i++) {
            points.row(i) = pcd[i].cast<double>();
        }
        viewer.data().add_points(points,
                                 lostTracking ? Eigen::RowVector3d(1, 0, 0) : Eigen::RowVector3d(1, 1, 1) * 0.8f);

        // Interaction
        if (!lostTracking) {
            if (leftTracked && rightTracked) {
                // Both hands

                static const Eigen::MatrixXi EDGES_BOTH_HANDS = (Eigen::MatrixXi(48, 2)
                        <<
                        /* Left  */
                        /* Thumb  */ 1, 2, /**/ 2, 3, /**/ 3, 4, /**/ 4, 5,
                        /* Index  */ 1, 6, /**/ 6, 7, /**/ 7, 8, /**/ 8, 9, /**/ 9, 10,
                        /* Middle */ 1, 11, /**/ 11, 12, /**/ 12, 13, /**/ 13, 14, /**/ 14, 15,
                        /* Ring   */ 1, 16, /**/ 16, 17, /**/ 17, 18, /**/ 18, 19, /**/ 19, 20,
                        /* Pinky  */ 1, 21, /**/ 21, 22, /**/ 22, 23, /**/ 23, 24, /**/ 24, 25,
                        /* Right  */
                        /* Thumb  */ 27, 28, /**/ 28, 29, /**/ 29, 30, /**/ 30, 31,
                        /* Index  */ 27, 32, /**/ 32, 33, /**/ 33, 34, /**/ 34, 35, /**/ 35, 36,
                        /* Middle */ 27, 37, /**/ 37, 38, /**/ 38, 39, /**/ 39, 40, /**/ 40, 41,
                        /* Ring   */ 27, 42, /**/ 42, 43, /**/ 43, 44, /**/ 44, 45, /**/ 45, 46,
                        /* Pinky  */ 27, 47, /**/ 47, 48, /**/ 48, 49, /**/ 49, 50, /**/ 50, 51).finished();

                Eigen::MatrixXd jointPoints((int) HandJointIndexCount * 2, 3);

                for (int i = 0; i < HandIndexCount; i++) {
                    for (int j = 0; j < HandJointIndexCount; j++) {
                        jointPoints.row(i * HandJointIndexCount + j) = XMVectorToEigenVector3d(
                                XMTransformToTranslate(interactionFrame.hands[i].joints[j].transformation));
                    }
                }
                viewer.data().set_edges(jointPoints, EDGES_BOTH_HANDS, BOTH_HANDS_EDGE_COLORS);

            } else if (leftTracked || rightTracked) {
                // Single hand

                int i = leftTracked ? Left : Right;
                static const Eigen::MatrixXi EDGES_SINGLE_HAND = (Eigen::MatrixXi(24, 2)
                        <<
                        /* Thumb  */ 1, 2, /**/ 2, 3, /**/ 3, 4, /**/ 4, 5,
                        /* Index  */ 1, 6, /**/ 6, 7, /**/ 7, 8, /**/ 8, 9, /**/ 9, 10,
                        /* Middle */ 1, 11, /**/ 11, 12, /**/ 12, 13, /**/ 13, 14, /**/ 14, 15,
                        /* Ring   */ 1, 16, /**/ 16, 17, /**/ 17, 18, /**/ 18, 19, /**/ 19, 20,
                        /* Pinky  */ 1, 21, /**/ 21, 22, /**/ 22, 23, /**/ 23, 24, /**/ 24, 25).finished();

                Eigen::MatrixXd jointPoints((int) HandJointIndexCount, 3);
                for (int j = 0; j < HandJointIndexCount; j++) {
                    jointPoints.row(j) = XMVectorToEigenVector3d(
                            XMTransformToTranslate(interactionFrame.hands[i].joints[j].transformation));
                }
                viewer.data().set_edges(jointPoints, EDGES_SINGLE_HAND, HAND_COLOR[i]);
            }
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
    std::ios::sync_with_stdio(false);

    discardDelayedFrames = true;
    ahatSource = static_cast<AHATSource *>(&tcpStreamingSource);
    interactionSource = static_cast<InteractionSource *>(&tcpStreamingSource);

//    discardDelayedFrames = false;
//    fileDataSource = std::make_unique<FileDataSource>("/Users/liuzikai/Downloads/2022-11-01-201827-AHAT-PV-EYE-Green-Book-Slow");
//    ahatSource = static_cast<AHATSource *>(fileDataSource.get());
//    interactionSource = static_cast<InteractionSource *>(fileDataSource.get());

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 24; j++) {
            BOTH_HANDS_EDGE_COLORS.row(i * 24 + j) = HAND_COLOR[i];
        }
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.callback_pre_draw = callBackPerDraw;
    viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
    viewer.data().point_size = 2;
    viewer.data().line_width = 10;
    viewer.core().is_animating = true;
    Eigen::Vector4f color(1, 1, 1, 1);
    viewer.core().background_color = color * 0.0f;
    viewer.launch();
}