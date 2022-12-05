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

#ifdef BOOST_AVAILABLE

#include "TCPDataSource.h"

#endif

#include "DepthProcessor.h"
#include "DirectXHelpers.h"
#include "EigenHelpers.h"
#include "PCDDataTypes.h"
#include "Registrator.h"

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
                // NOTICE: swapping x and y and both negated
                pcd.emplace_back(-pcdRaw[i * 3 + 1], -pcdRaw[i * 3], pcdRaw[i * 3 + 2]);
            }
            return true;
        } else {
            return false;
        }
    }

    bool sendReconstructedPCD(const PCD& pcd) override {
        //TODO implement (probably not necessary actually since we are already visualizing locally)
        return false;
    }
};

Registrator registrator;

Eigen::MatrixXd ReconstructedPCD;

bool discardDelayedFrames = false;

RawDataSource *rawDataSource = nullptr;

std::unique_ptr<DepthProcessorWrapper> depthProcessor;

static const Eigen::RowVector3d HAND_COLOR[HandIndexCount] = {Eigen::RowVector3d(0, 1, 0),
                                                              Eigen::RowVector3d(1, 0, 0)};
static const Eigen::RowVector3d HAND_MESH_COLOR[HandIndexCount] = {Eigen::RowVector3d(0, 1, 1),
                                                                   Eigen::RowVector3d(1, 0, 1)};
Eigen::MatrixXd BOTH_HANDS_EDGE_COLORS(48, 3);

bool callBackPerDraw(igl::opengl::glfw::Viewer &viewer) {
    // static to hold the latest data to redraw on tracking lost
    static timestamp_t pcdTimestamp;
    static PCD pcd;
    static bool lostTracking = false;

    static timestamp_t handTimestamp;
    static HandDebugFrame debugHandFrame;
    bool hasDebugHand = false;

    RawDataFrame rawDataFrame;

    static int warm_up_frame = 100;

    if (!depthProcessor) {
        DirectX::XMMATRIX ahatExtrinsics;
        std::vector<float> ahatLUT;
        if (!rawDataSource || !rawDataSource->getAHATExtrinsics(ahatExtrinsics)) return false;
        if (!rawDataSource || !rawDataSource->getAHATDepthLUT(ahatLUT)) return false;
        depthProcessor = std::make_unique<DepthProcessorWrapper>(ahatExtrinsics, ahatLUT.data());
        std::cout << "Create DepthProcessor with AHAT extrinsics and LUT" << std::endl;
    }

    bool redraw = false;

    if (rawDataSource) {
        do {
            if (!rawDataSource->getNextRawDataFrame(rawDataFrame)) break;
#if 0
            std::cout << "[Raw] " << rawDataFrame.timestamp << "    lostTracking = " << lostTracking << std::endl;
#endif
        } while (discardDelayedFrames);  // continue the loop if discardDelayedFrames

        bool newLostTracking = !depthProcessor->update(rawDataFrame);
        if (newLostTracking != lostTracking) {
            redraw = true;
        }
        lostTracking = newLostTracking;
    }

    bool merge_successful = false;
    if (depthProcessor) {
        bool pcdUpdated = false;
        do {
            if (!depthProcessor->getNextPCD(pcdTimestamp, pcd)) break;
            hasDebugHand = depthProcessor->getNextHandDebugFrame(handTimestamp, debugHandFrame);
#if 1
            std::cout << "[PCD] " << pcd.size() << std::endl;
#endif
            pcdUpdated = true;
            redraw = true;

        } while (discardDelayedFrames);  // continue the loop if discardDelayedFrames

        //merge current pcd with previous data
        if (pcdUpdated && warm_up_frame == 0) {
#ifdef USE_DBSCAN
            merge_successful = registrator.mergePCD(pcd, depthProcessor->handMesh);
#else
            merge_successful = registrator.mergePCD(pcd);
#endif
        }
    }

    if (redraw) {
        viewer.data().clear();

        // PCD
        {
            //Display Registration
            if (registrator.getReconstructedPCDInEigenFormat(ReconstructedPCD)) {
                std::cout << "[ReconstructedPCD] " << ReconstructedPCD.size() << "  merge_successful = " << merge_successful << std::endl;
                viewer.data().add_points(ReconstructedPCD,
                                         merge_successful ? Eigen::RowVector3d(1, 1, 1) : Eigen::RowVector3d(1, 1, 0));
                //Send merged point cloud
                if (depthProcessor)
                    depthProcessor->sendReconstructedPCD(*registrator.getReconstructedPCD());
            }

            // Set camera on first frame
            if (warm_up_frame > 0 && pcd.size() > 200) {
                warm_up_frame--;
                if (warm_up_frame == 0) {
                    std::cout << "============================================================" << std::endl;
                    Eigen::MatrixXd points(pcd.size(), 3);
                    for (int i = 0; i < pcd.size(); i++) {
                        points.row(i) = pcd[i].cast<double>();
                    }
                    viewer.core().align_camera_center(points);
                }
            }
        }

        // Hand meshes
        {
            for (int h = 0; h < HandIndexCount; h++) {
                const auto &meshPoints = depthProcessor->handMesh[h];
                Eigen::MatrixXd points(meshPoints.size(), 3);
                for (int i = 0; i < meshPoints.size(); i++) {
                    points.row(i) = XMVectorToEigenVector3d(meshPoints[i]);
                }
                viewer.data().add_points(points, HAND_MESH_COLOR[h]);
            }
        }

        // Hand edges
#if 1
        bool leftTracked = rawDataFrame.hands[Left].strictlyTracked;
        bool rightTracked = rawDataFrame.hands[Right].strictlyTracked;

        if (!lostTracking && hasDebugHand) {

            // TODO: clean up the duplicate code
            if (leftTracked && rightTracked) {
                int lhvSize = static_cast<int>(debugHandFrame.lhMesh.size());
                int lhiSize = static_cast<int>(debugHandFrame.lhIndices.size());
                int rhvSize = static_cast<int>(debugHandFrame.rhMesh.size());
                int rhiSize = static_cast<int>(debugHandFrame.rhIndices.size());
                Eigen::MatrixXd jointPoints(lhvSize + rhvSize, 3);
                Eigen::MatrixXi jointIndices(lhiSize + rhiSize, 2);
                Eigen::MatrixXd colors(lhiSize + rhiSize, 3);

                for (int j = 0; j < lhvSize; j++) {
                    jointPoints.row(j) = XMVectorToEigenVector3d(debugHandFrame.lhMesh[j]);
                }
                for (int j = 0; j < rhvSize; j++) {
                    jointPoints.row(j + lhvSize) = XMVectorToEigenVector3d(debugHandFrame.rhMesh[j]);
                }

                for (int j = 0; j < lhiSize; j++) {
                    jointIndices.row(j) << debugHandFrame.lhIndices[j][0], debugHandFrame.lhIndices[j][1];
                    colors.row(j) = Eigen::RowVector3d(0, 1, 0);
                }
                for (int j = 0; j < rhiSize; j++) {
                    jointIndices.row(j + lhiSize) << debugHandFrame.rhIndices[j][0] + lhvSize,
                            debugHandFrame.rhIndices[j][1] + lhvSize;
                    colors.row(j + lhiSize) = Eigen::RowVector3d(1, 0, 0);
                }

                viewer.data().set_edges(jointPoints, jointIndices, colors);

            } else if (leftTracked) {

                int lhvSize = static_cast<int>(debugHandFrame.lhMesh.size());
                int lhiSize = static_cast<int>(debugHandFrame.lhIndices.size());
                Eigen::MatrixXd jointPoints(lhvSize, 3);
                Eigen::MatrixXi jointIndices(lhiSize, 2);

                for (int j = 0; j < lhvSize; j++) {
                    jointPoints.row(j) = XMVectorToEigenVector3d(debugHandFrame.lhMesh[j]);
                }

                for (int j = 0; j < lhiSize; j++) {
                    jointIndices.row(j) << debugHandFrame.lhIndices[j][0], debugHandFrame.lhIndices[j][1];
                }

                viewer.data().set_edges(jointPoints, jointIndices, Eigen::RowVector3d(0, 1, 0));

            } else if (rightTracked) {

                int rhvSize = static_cast<int>(debugHandFrame.rhMesh.size());
                int rhiSize = static_cast<int>(debugHandFrame.rhIndices.size());
                Eigen::MatrixXd jointPoints(rhvSize, 3);
                Eigen::MatrixXi jointIndices(rhiSize, 2);

                for (int j = 0; j < rhvSize; j++) {
                    jointPoints.row(j) = XMVectorToEigenVector3d(debugHandFrame.rhMesh[j]);
                }

                for (int j = 0; j < rhiSize; j++) {
                    jointIndices.row(j) << debugHandFrame.rhIndices[j][0], debugHandFrame.rhIndices[j][1];
                }

                viewer.data().set_edges(jointPoints, jointIndices, Eigen::RowVector3d(1, 0, 0));
            }
        }
#endif
    }

    return false;
}

int main() {
    std::ios::sync_with_stdio(false);

#ifdef BOOST_AVAILABLE
    TCPDataSource tcpStreamingSource;
    discardDelayedFrames = true;
    rawDataSource = static_cast<RawDataSource *>(&tcpStreamingSource);
#else
#error "Boost not available"
#endif

//    discardDelayedFrames = false;
//    std::unique_ptr<FileDataSource> fileDataSource = std::make_unique<FileDataSource>("/Users/liuzikai/Files/MR-Local/2022-11-01-201827-AHAT-PV-EYE-Green-Book-Slow");
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