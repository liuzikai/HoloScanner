//
// Created by Noureddine Gueddach on 21/11/2022.
//

#include "open3d/Open3D.h"
#include "Registrator.h"
#include <memory>

using namespace open3d;
using std::cout;
using std::endl;

std::unique_ptr<PCD> Registrator::getReconstructedPCD() const {
    if (!m_pcd) return nullptr;
    PCD pcd;
    pcd.reserve(m_pcd->points_.size());
    for (const auto &p: m_pcd->points_) {
        pcd.push_back(p);
    }
    return std::make_unique<PCD>(pcd);
}

bool Registrator::getReconstructedPCDInEigenFormat(Eigen::MatrixXd &Mat) const {
    if (!m_pcd) return false;
    Mat.resize(m_pcd->points_.size(), 3);
    for (int i = 0; i < m_pcd->points_.size(); i++) {
        Mat.row(i) = m_pcd->points_[i].transpose();
    }
    return true;
}

Eigen::Matrix4d Registrator::getTransformation(const geometry::PointCloud &source,
                                               const geometry::PointCloud &target, Eigen::Matrix6d &InfoMat,
                                               const double kernel_param) const {

    int nb_iterations = 300;

    double voxel_size = 0.01;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 3; ++i) {
        auto source_down = source.VoxelDownSample(voxel_size);
        source_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
                voxel_size * 2.0, 30));

        auto target_down = target.VoxelDownSample(voxel_size);
        target_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
                voxel_size * 2.0, 30));

        auto loss = pipelines::registration::TukeyLoss(kernel_param);
        auto kernel = loss.k_;
        auto result = pipelines::registration::RegistrationGeneralizedICP(
                *source_down, *target_down, m_max_corr_dist_transformation, T,
                pipelines::registration::TransformationEstimationForGeneralizedICP(kernel),
                pipelines::registration::ICPConvergenceCriteria(1e-7, 1e-7, nb_iterations));
        T = result.transformation_;
        voxel_size /= 2;
    }
    InfoMat = pipelines::registration::GetInformationMatrixFromPointClouds(source, target,
                                                                           m_max_corr_dist_transformation, T);
    return T;
}

bool Registrator::isRegistrationSuccessful(const geometry::PointCloud &pcd, const Eigen::Matrix4d &T) const {
    auto result = pipelines::registration::EvaluateRegistration(pcd, *m_pcd, m_max_corr_dist_evaluation, T);
    auto correspondance_set = result.correspondence_set_;
    auto fitness = result.fitness_; //Corresponds to: correspondance_set.size() / pcd.points_.size()
    auto rmse = result.inlier_rmse_;
    //bool most_of_pcd_is_inlier = correspondance_set.size() >= 0.8 * pcd.points_.size(); //same as fitness
    cout << fitness << " " << rmse << " " << endl;
    bool high_fitness = fitness > m_min_fitness;
    bool low_rmse = rmse < m_max_rmse;
    return high_fitness && low_rmse;
}

std::tuple<std::shared_ptr<geometry::PointCloud>,
    std::shared_ptr<pipelines::registration::Feature>>
    PreprocessPointCloud(const geometry::PointCloud& pcd, const float voxel_size) {
    auto pcd_down = pcd.VoxelDownSample(voxel_size);
    pcd_down->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(2 * voxel_size, 30));
    auto pcd_fpfh = pipelines::registration::ComputeFPFHFeature(
        *pcd_down, open3d::geometry::KDTreeSearchParamHybrid(5 * voxel_size, 100));
    return std::make_tuple(pcd_down, pcd_fpfh);
}

Eigen::Matrix4d global_registration(const geometry::PointCloud& source,
    const geometry::PointCloud& target, Eigen::Matrix6d& InfoMat,
    const float max_correspondance_dist,
    const double voxel_size) {
    std::shared_ptr<geometry::PointCloud> source_down, target_down;
    std::shared_ptr<pipelines::registration::Feature> source_fpfh, target_fpfh;
    std::tie(source_down, source_fpfh) = PreprocessPointCloud(source, voxel_size);
    std::tie(target_down, target_fpfh) = PreprocessPointCloud(target, voxel_size);

    pipelines::registration::RegistrationResult registration_result;

    // Prepare checkers
    std::vector<std::reference_wrapper<
        const pipelines::registration::CorrespondenceChecker>> correspondence_checker;
    auto correspondence_checker_edge_length =
        pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto correspondence_checker_distance =
        pipelines::registration::CorrespondenceCheckerBasedOnDistance(max_correspondance_dist);
    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);

    bool mutual_filter = false;
    float confidence = 0.999;
    int max_iterations = 4000;
    registration_result = pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
        *source_down, *target_down, *source_fpfh, *target_fpfh,
        mutual_filter, max_correspondance_dist,
        pipelines::registration::TransformationEstimationPointToPoint(false),
        3, correspondence_checker,
        pipelines::registration::RANSACConvergenceCriteria(max_iterations, confidence));
    return registration_result.transformation_;
}

#ifdef USE_DBSCAN
bool Registrator::mergePCD(const PCD &pcd_, std::vector<DirectX::XMVECTOR> handMesh[2]) 
#else 
bool Registrator::mergePCD(const PCD &pcd_) 
#endif
{
    if (pcd_.size() < 500) return false;
    auto pcd = geometry::PointCloud(pcd_);
    if (m_pcd == nullptr) { //First registration is always successful as it initializes the point cloud
        m_pcd = std::make_shared<geometry::PointCloud>(pcd);
        return true;
    }

    //DBSCAN
    // bool dbscan = false; //Note: DBSCAN is too slow for real-time (could be use) for a final pass though
    // if(dbscan) {
    //     std::vector<int> indices = pcd.ClusterDBSCAN(0.1, 0.7 * pcd.points_.size());
    //     PCD valid_points;
    //     for(int i = 0; i < indices.size(); i++) {
    //         if(indices[i] != -1)
    //             valid_points.push_back(pcd.points_[i]);
    //     }
    //     pcd.points_ = valid_points;
    //}
    //std::vector<size_t> index = std::get<1>(pcd.RemoveStatisticalOutliers(16, 0.8));
    //pcd = *pcd.SelectByIndex(index);

#ifdef USE_DBSCAN
    std::vector<size_t> index = std::get<1>(pcd.RemoveStatisticalOutliers(16, 0.8));
    pcd = *pcd.SelectByIndex(index);
    std::vector<int> labels = pcd.ClusterDBSCAN(0.005, 4);
    std::set<int> labels_unique;
    for (int i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0) {
            labels_unique.insert(labels[i]);
        }
    }
    // mean of the hand points
    Eigen::Vector3d hand_center(0.0, 0.0, 0.0);
    for(size_t i=0; i<2; ++i){
        for(size_t j=0; j<handMesh[i].size(); ++j){
            DirectX::XMFLOAT3 f;
            DirectX::XMStoreFloat3(&f, handMesh[i][j]);
            hand_center[0] -= f.y;
            hand_center[1] -= f.x;
            hand_center[2] += f.z;
        }
    }

    hand_center = hand_center / (2*handMesh[0].size());
    // some recording for each cluster
    std::vector<size_t> labels_num(labels_unique.size(), 0);
    std::vector<double> distances(labels_unique.size(), std::numeric_limits<double>::max());
    std::vector<std::vector<size_t>> labels_index;
    for (int i = 0; i < labels_unique.size(); ++i) {
        labels_index.push_back(std::vector<size_t>());
    }
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0){
            double distance = (pcd_[i] - hand_center).squaredNorm();
            if(distance < distances[labels[i]]){
                distances[labels[i]] = distance;
            }
            labels_num[labels[i]]++;
            labels_index[labels[i]].push_back(i);
        }
    }
    size_t argclose = std::distance(distances.begin(), std::min_element(distances.begin(),distances.end()));
    size_t argmax = std::distance(labels_num.begin(), std::max_element(labels_num.begin(), labels_num.end()));
    // usually argclose == argmax,  but argclose now
    pcd = *pcd.SelectByIndex(labels_index[argclose]);
#endif

    //Compute the transformation between the current and global point cloud
    Eigen::Matrix6d InfoMat;
    double kernel_param = 0.1;
    Eigen::Matrix4d T = getTransformation(pcd, *m_pcd, InfoMat, kernel_param);

    //Evaluate the registration
    bool success = isRegistrationSuccessful(pcd, T);
    //If not successful, keep the global point cloud as is, wait for the user to realign
    if (!success) return false;


    *m_pcd = m_pcd->Transform(T.inverse()); //Bring the global point cloud into the reference of the current frame
    *m_pcd += pcd; //Merge the current frame to the global point cloud
    m_pcd = m_pcd->VoxelDownSample(0.0025); //downsample for performance

    return true;
}

void Registrator::saveReconstructedMesh(const std::string &save_path) const {
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
    std::vector<double> densities;
    m_pcd->EstimateNormals();
    float scale = 3;
    std::tie(mesh, densities) = geometry::TriangleMesh::CreateFromPointCloudPoisson(*m_pcd, 8UL, 0, scale);
    io::WriteTriangleMesh(save_path, *mesh);
}