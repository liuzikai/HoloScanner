//
// Created by Noureddine Gueddach on 21/11/2022.
//

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <open3d/geometry/PointCloud.h>
#include <PCDDataTypes.h>

//using PCD = std::vector<Eigen::Vector3d>; //Note: Open3D requires Vector3d, not Vector3f

class Registrator {
public:

    //Using default params
    Registrator() {}

    //Using custom params
    Registrator(float max_corr_dist_transformation, float max_corr_dist_evaluation, 
        float max_rmse, float min_fitness) : 
        m_max_corr_dist_transformation(m_max_corr_dist_transformation),
        m_max_corr_dist_evaluation(max_corr_dist_evaluation),
        m_max_rmse(max_rmse), m_min_fitness(min_fitness) { }

    /**
     * @brief Try to merge the given PCD into the global PCD. Returns whether the merge succeeds or not
     * @param pcd [in] point to cloud to merge into the global point cloud
     * @return true if the merge succeeds, false otherwise
     */
    bool mergePCD(const PCD& pcd);

    /**
     * @brief Returns the latest reconstruction result
     * @return The point cloud reconstructed so far as a PCD type
     */
    std::unique_ptr<PCD> getReconstructedPCD() const;

    /**
     * @brief Construct a mesh from the point cloud and save it to disk
     * @param save_path path to which the mesh should be saved
     */
    void saveReconstructedMesh(const std::string& save_path) const;

private:
    std::shared_ptr<open3d::geometry::PointCloud> m_pcd;
    float m_max_corr_dist_transformation = 0.004;
    float m_max_corr_dist_evaluation = 0.008;
    float m_max_rmse = 0.004;
    float m_min_fitness = 0.97;

    /**
     * @brief Construct a new is Registration Successful object
     * 
     * @param pcd new point cloud
     * @param T transformation from new point cloud to global point cloud
     */
    bool isRegistrationSuccessful(const open3d::geometry::PointCloud& pcd, const Eigen::Matrix4d& T) const;

    /**
     * @brief Get the Transformation from the given point cloud to the target one
     * 
     * @param source new point cloud 
     * @param target target point cloud
     * @param InfoMat [out] 6x6 matrix holding transformation information about whatever
     * @param kernel_param parameter used to indicate the stdev param of the TukeyLoss used internally
     * @return [Eigen::Matrix4d] Transformation from source to target
     */
    Eigen::Matrix4d getTransformation(const open3d::geometry::PointCloud& source,
        const open3d::geometry::PointCloud& target, Eigen::Matrix6d& InfoMat, 
        const double kernel_param);
};