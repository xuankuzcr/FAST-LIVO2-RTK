#pragma once 
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <gnss_comm/GnssPVTSolnMsg.h>

#include <GeographicLib/LocalCartesian.hpp> 
#include <Eigen/StdVector>
#include <atomic>
#include <memory>
#include "LIVMapper.h"
#include "FastDTW/example.hpp"

struct KeyFrame
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double time = 0.0;
    gtsam::Pose3 pose = gtsam::Pose3::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    PointCloudXYZRGB::Ptr cloud;
};

using KeyFrameVector = std::vector<KeyFrame, Eigen::aligned_allocator<KeyFrame>>;

inline Eigen::Affine3f poseToAffine3f(const gtsam::Pose3& pose)
{ 
    const auto& t = pose.translation();
    const auto rpy = pose.rotation().rpy();
    return pcl::getTransformation(t.x(), t.y(), t.z(), rpy.x(), rpy.y(), rpy.z());
}

inline gtsam::Pose3 computeSVD(const std::vector<Eigen::Vector3d>& target, 
                               const std::vector<Eigen::Vector3d>& source)
{
    if (target.empty() || target.size() != source.size()) {
        return gtsam::Pose3::Identity(); 
    }

    Eigen::Vector3d target_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d source_center = Eigen::Vector3d::Zero();
    for (const auto& p : target) target_center += p;
    for (const auto& p : source) source_center += p;
    target_center /= target.size();
    source_center /= source.size();

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < source.size(); ++i) {
        W += (target[i] - target_center) * (source[i] - source_center).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) { 
        R = svd.matrixU() * Eigen::DiagonalMatrix<double, 3>(1, 1, -1) * svd.matrixV().transpose();
    }

    Eigen::Vector3d t = target_center - R * source_center;
    
    return gtsam::Pose3{gtsam::Rot3(R), gtsam::Point3(t)};
}

class optimization
{
public:
    optimization(ros::NodeHandle &nh);
    ~optimization();

    void loadData(const std::string& data_dir);
    void offlineOptimizationTask();
    void initialAlign();
    double calculateDtwTimeOffset(const std::vector<std::vector<double>>& gpsdata, const std::vector<std::vector<double>>& slamdata);
    double estimateVelocityTimeOffset(const std::vector<std::vector<double>>& gps_position_data,
                                      const std::vector<std::vector<double>>& slam_position_data,
                                      const std::vector<std::vector<double>>& gps_velocity_data,
                                      const std::vector<std::vector<double>>& slam_velocity_data,
                                      double coarse_time_offset,
                                      double& best_score,
                                      double& zero_score,
                                      int& best_pair_count,
                                      std::string& velocity_source);
    void buildBatchGraph();
    void waitForKeyFrameIdle(double idle_seconds);
    void saveKeyFrameAndFactor(const gtsam::Pose3& pose,
                               double time,
                               const Eigen::Vector3d& velocity,
                               const PointCloudXYZRGB::Ptr& cloud);
    void syncedCallback(const nav_msgs::Odometry::ConstPtr& odomMsg, const sensor_msgs::PointCloud2::ConstPtr& cloudMsg);
    void gpsHandler(const gnss_comm::GnssPVTSolnMsg::ConstPtr& pvtMsg);
    // void gpsHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg);

    void saveOptimizedGlobalMap();
    void savekeyframescan();
    void writeTumTrajectory(const std::string& path);
    void writeOptimizedTumTrajectory();
    void writeRtkTumTrajectory();
    
    template<typename T>
    void publishCloud(const ros::Publisher& pub, const T& cloud, const ros::Time& stamp, const std::string& frame_id)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        pub.publish(msg);
    }

public:
    ros::Subscriber subGPS;
    ros::Subscriber subGPS_pvt;

    ros::Publisher pubGpsOdom;

    message_filters::Subscriber<nav_msgs::Odometry> subOdom_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> subCloud_;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> SyncPolicy;


    gtsam::NonlinearFactorGraph gtSAMgraph; 
    gtsam::Values initialEstimate;        
    gtsam::Values isamCurrentEstimate;
    gtsam::Values optimizedEstimate;
    
    gtsam::Pose3 T_imu_rtk;


    KeyFrameVector keyFrames;

    std::thread optimization_thread_;
    std::atomic_bool optimization_shutdown_requested_{false};

    std::deque<nav_msgs::Odometry> gpsQueue;
    std::deque<nav_msgs::Odometry> gpsQueue_B;
    GeographicLib::LocalCartesian gps_trans_;

    float gpstimestamp = 0.0;
    double timeLaserInfoCur;
    ros::Time timeLaserInoStamp;

    vector<double> gps_extrinT;

    double gps_offset;

    double rtk_cov;
    double livo2_RPY_cov;
    double livo2_XYZ_cov;
       

    bool debug_mode = false;
    bool addrtkfactor_ = false;
    bool save_pcd_enable_ = false;
    bool is_optimized = false;
    bool gps_en;
    bool accepting_keyframes_ = true;
    ros::WallTime last_keyframe_wall_time_;

    string gps_topic;
    string outputfilepath;
    string debug_optdata_path_;
    string opt_tum_output_path_;
    string livo_tum_before_output_path_;
    string rtk_tum_output_path_;
    string opt_vel_output_path_;
    string gps_vel_output_path_;
    string global_map_pcd_path_;
    string keyframe_scan_pcd_path_;
    std::string pcd_save_directory_;

private:
    std::mutex mutex;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};
