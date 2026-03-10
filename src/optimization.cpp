#include "optimization.h"

optimization::optimization(ros::NodeHandle &nh)
{
    nh.param<std::string>("laserMapping/outputfilepath", outputfilepath, "");
    debug_optdata_path_ = outputfilepath + "/debug/";
    opt_tum_output_path_ = outputfilepath + "/TUM/opt_trajectory_after.txt";
    gps_tum_output_path_ = outputfilepath + "/TUM/opt_trajectory_before.txt";
    opt_vel_output_path_ = outputfilepath + "/vel/opt_vel.txt";
    gps_vel_output_path_ = outputfilepath + "/vel/gps_vel.txt";
    global_map_pcd_path_ = outputfilepath + "/global_pcd/";
    keyframe_scan_pcd_path_ = outputfilepath + "/scan_pcd/";

    nh.param<string>("gps/gps_topic", gps_topic, "/ublox_driver/receiver_lla");
    nh.param<double>("gps/gps_time_offset", gps_offset, 0.0);
    nh.param<bool>("gps/gps_en", gps_en, false);
    nh.param<vector<double>>("gps/extrinsic_T", gps_extrinT, vector<double>());

    nh.param<bool>("opt/debug_mode", debug_mode, false);
    nh.param<double>("opt/rtk_cov", rtk_cov, 0.02);
    nh.param<double>("opt/livo2_RPY_cov", livo2_RPY_cov, 1e-4);
    nh.param<double>("opt/livo2_XYZ_cov", livo2_XYZ_cov, 1e-4);

    // subGPS = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 2000, &optimization::gpsHandler, this);
    subGPS_pvt = nh.subscribe<gnss_comm::GnssPVTSolnMsg>(gps_topic, 2000, &optimization::gpsHandler, this);

    Eigen::Vector3d gps_lever_arm_; 
    gps_lever_arm_(0) = gps_extrinT[0]; 
    gps_lever_arm_(1) = gps_extrinT[1]; 
    gps_lever_arm_(2) = gps_extrinT[2]; 
    T_imu_rtk = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(gps_lever_arm_));

    subOdom_.subscribe(nh, "/odometry/fast_livo2", 2000);
    subCloud_.subscribe(nh, "/synced_cloud", 2000);
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), subOdom_, subCloud_);
    sync_->registerCallback(boost::bind(&optimization::syncedCallback, this, _1, _2));

    laserCloudSurfLastDS.reset(new PointCloudXYZRGB());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    for (int i = 0; i < 6; ++i) {
        transformTobeMapped[i] = 0;
    }

    if(debug_mode)
    {
        loadData(debug_optdata_path_);
    }

    ROS_INFO("Optimization mode: [OFFLINE]. Accumulating factors for batch optimization.");
    optimization_thread_ = std::thread(&optimization::offlineOptimizationTask, this);
}

optimization::~optimization() 
{
}

//Optimize threads
void optimization::offlineOptimizationTask() {
    std::cin.get();
    std::cout << "[Offline Optimization] Starting batch optimization..." << std::endl;
    
    initialAlign();
    std::cout << "[Offline Optimization] Initial alignment done." << std::endl;
    
    writeOptimizedTumTrajectory();
    saveOptimizedGlobalMap();
    std::cout << "[Offline Optimization] Initial global map and tum saved." << std::endl;
    
    gtsam::LevenbergMarquardtParams params;
    params.orderingType = gtsam::Ordering::METIS;
    params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
    params.setMaxIterations(500);
    params.setRelativeErrorTol(1e-9);
    params.setAbsoluteErrorTol(1e-9);
    
    if(gps_en)
    {
        addrtkfactor_ = true;
        buildBatchGraph();
        gtsam::LevenbergMarquardtOptimizer optimizer(gtSAMgraph, initialEstimate);
        gtsam::Values result = optimizer.optimize();
        initialEstimate = result;
        std::cout << "[Offline Optimization] First optimization pass done." <<  std::endl;
    }

    is_optimized = true;
    int numPoses = initialEstimate.size();

    for (int i = 0; i < numPoses; ++i) {
        gtsam::Pose3 rtk_optimizedPose = initialEstimate.at<gtsam::Pose3>(i);
        cloudKeyPoses6D->points[i].x     = rtk_optimizedPose.translation().x();
        cloudKeyPoses6D->points[i].y     = rtk_optimizedPose.translation().y();
        cloudKeyPoses6D->points[i].z     = rtk_optimizedPose.translation().z();
        cloudKeyPoses6D->points[i].roll  = rtk_optimizedPose.rotation().roll();
        cloudKeyPoses6D->points[i].pitch = rtk_optimizedPose.rotation().pitch();
        cloudKeyPoses6D->points[i].yaw   = rtk_optimizedPose.rotation().yaw();
    }

    writeOptimizedTumTrajectory();
    std::cout << "[Offline Optimization] Optimized TUM trajectory saved "  << std::endl;
    writeRtkTumTrajectory();
    std::cout << "[Offline Optimization] RTK TUM trajectory saved " << std::endl;

    for (int i = 0; i < numPoses; ++i) {
        gtsam::Pose3 rtk_optimizedPose = initialEstimate.at<gtsam::Pose3>(i);
        gtsam::Pose3 imu_optimizedPose = rtk_optimizedPose.compose((T_imu_rtk).inverse());
        cloudKeyPoses6D->points[i].x     = imu_optimizedPose.translation().x();
        cloudKeyPoses6D->points[i].y     = imu_optimizedPose.translation().y();
        cloudKeyPoses6D->points[i].z     = imu_optimizedPose.translation().z();
        cloudKeyPoses6D->points[i].roll  = imu_optimizedPose.rotation().roll();
        cloudKeyPoses6D->points[i].pitch = imu_optimizedPose.rotation().pitch();
        cloudKeyPoses6D->points[i].yaw   = imu_optimizedPose.rotation().yaw();
    }

    std::cout << "[Offline Optimization] Saving maps and trajectories..." << std::endl;
    //savekeyframescan();
    saveOptimizedGlobalMap();
    std::cout << "[Offline Optimization] Global map saved " << std::endl;

    std::cout << "[Offline Optimization] Finished." << std::endl;
}

void optimization::saveKeyFramesAndFactor()
{
    std::lock_guard<std::mutex> lock(mutex);
    
    // Store pose
    PointTypePose thisPose6D;
    gtsam::Pose3 currentPose = trans2gtsamPose(transformTobeMapped);
    initialEstimate.insert(cloudKeyPoses6D->size(), currentPose);

    thisPose6D.x = currentPose.translation().x();
    thisPose6D.y = currentPose.translation().y();
    thisPose6D.z = currentPose.translation().z();
    thisPose6D.intensity = cloudKeyPoses6D->size();
    thisPose6D.roll  = currentPose.rotation().roll();
    thisPose6D.pitch = currentPose.rotation().pitch();
    thisPose6D.yaw   = currentPose.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);
    
    // Storage point cloud
    PointCloudXYZRGB::Ptr thisSurfKeyFrame(new PointCloudXYZRGB());
    pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);
    
}

//DTW
double optimization::calculateDtwTimeOffset(
    const std::vector<std::vector<double>>& gpsdata,
    const std::vector<std::vector<double>>& slamdata)
{
    const double MIN_EFFECTIVE_VELOCITY = 0.5; // m/s
    const int IDX_TIME = 0;
    const int IDX_X = 1;
    const int IDX_Y = 2;
    const int IDX_Z = 3;

    auto process_data = [=](const std::vector<std::vector<double>>& data_in) 
        -> std::pair<std::vector<double>, std::vector<double>>
    {
        double duration = 50.0;
        const double t_start = data_in[0][IDX_TIME];
        const double t_end   = t_start + duration;

        std::vector<double> times;
        std::vector<double> norms;
        
        bool is_moving = false;
        Eigen::Vector3d origin_pos(0, 0, 0); 

        for (size_t i = 1; i < data_in.size(); ++i) {
            const auto& row_i = data_in[i];
            const auto& row_prev = data_in[i-1];

            const double current_time = row_i[IDX_TIME];
            if (current_time > t_end) {
                break; 
            }

            if (!is_moving) {
                double dt = row_i[IDX_TIME] - row_prev[IDX_TIME];
                if (dt < 1e-6) continue; 
                double dx = row_i[IDX_X] - row_prev[IDX_X];
                double dy = row_i[IDX_Y] - row_prev[IDX_Y];
                double dz = row_i[IDX_Z] - row_prev[IDX_Z];               
                double vel_norm = std::sqrt(dx*dx + dy*dy + dz*dz) / dt;

                double pos_norm = std::sqrt(row_i[IDX_X]*row_i[IDX_X] + row_i[IDX_Y]*row_i[IDX_Y] + row_i[IDX_Z]*row_i[IDX_Z]);
                if (vel_norm > MIN_EFFECTIVE_VELOCITY && pos_norm > MIN_EFFECTIVE_VELOCITY) {
                    is_moving = true;
                } else {
                    continue; 
                }
            } else {
                Eigen::Vector3d current_pos(row_i[IDX_X], row_i[IDX_Y], row_i[IDX_Z]);
                double pos_norm = current_pos.norm();

                times.push_back(row_i[IDX_TIME]);
                norms.push_back(pos_norm);
            }
        }
        
        return {times, norms};
    };

    auto [gps_pos_times, gps_pos_norms] = process_data(gpsdata);
    auto [slam_pos_times, slam_pos_norms] = process_data(slamdata);

    sample1 = &(slam_pos_norms[0]);
    sample2 = &(gps_pos_norms[0]);

    std::vector<std::pair<int, int>> dtw_path;
    DTW(slam_pos_norms, gps_pos_norms, dtw_path);

    double total_time_diff = 0.0;
    int valid_pairs = 0;

    for (const auto& pair : dtw_path) {
        if (pair.first < slam_pos_times.size() && pair.second < gps_pos_times.size()) {
            total_time_diff += (slam_pos_times[pair.first] - gps_pos_times[pair.second]);
            valid_pairs++;
        }
    }

    if (valid_pairs == 0) {
        return 0.0;
    }

    double avg_time_diff = total_time_diff / valid_pairs;

    std::ofstream slam_file(opt_vel_output_path_);
    if (slam_file.is_open()) {
        slam_file << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < slam_pos_norms.size(); ++i) {
            slam_file << slam_pos_times[i] << " " << slam_pos_norms[i] << std::endl;
        }
        slam_file.close();
    } else {
        ROS_WARN("[initialAlign] could not open gps_vel.txt");
    }

    std::ofstream gps_file(gps_vel_output_path_);
    if (gps_file.is_open()) {
        gps_file << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < gps_pos_norms.size(); ++i) {
            gps_file << gps_pos_times[i] << " " << gps_pos_norms[i] << std::endl;
        }
        gps_file.close();
    } else {
        ROS_WARN("[initialAlign] could not open gps_vel.txt ");
    }
    return avg_time_diff;
}

//Spacetime synchronization
void optimization::initialAlign()
{
    //using DTW
    std::vector<std::vector<double>> slamdata;
    slamdata.reserve(cloudKeyPoses6D->size());
    for (const auto& p : cloudKeyPoses6D->points) {
        slamdata.push_back({p.time, p.x, p.y, p.z});
    }
    
    std::vector<std::vector<double>> gpsdata;
    gpsdata.reserve(gpsQueue.size());
    for (const auto& g : gpsQueue) {
        gpsdata.push_back({
            g.header.stamp.toSec(),
            g.pose.pose.position.x,
            g.pose.pose.position.y,
            g.pose.pose.position.z
        });
    }

    double avg_time_diff = calculateDtwTimeOffset(gpsdata, slamdata);
    
    ros::Duration time_offset(avg_time_diff);

    for(auto& g : gpsQueue) {
        g.header.stamp += time_offset;
    }
 
    ROS_INFO("[initialAlign] 1. DTW calculated time offset: %.4f seconds.", avg_time_diff);

    //using spline
    std::cout << " [initialAlign] initial align (using Spline)... " << std::endl;

    if (cloudKeyPoses6D->empty() || gpsQueue.empty()) {
        ROS_WARN("[initialAlign] empty buffers.");
        return;
    }
    
    struct Sample { double t; Eigen::Vector3d p; };
    std::vector<Sample> samp; 
    samp.reserve(gpsQueue.size());
    
    for (const auto& g : gpsQueue) {
        const double t = g.header.stamp.toSec();
        const auto& p  = g.pose.pose.position; 
        samp.push_back({ t, Eigen::Vector3d(p.x, p.y, p.z) });
    }
    std::sort(samp.begin(), samp.end(),
              [](const Sample& a, const Sample& b){ return a.t < b.t; });

    std::vector<double> T; T.reserve(samp.size());
    std::vector<double> X, Y, Z;
    X.reserve(samp.size()); Y.reserve(samp.size()); Z.reserve(samp.size());
    
    for (size_t i = 0; i < samp.size(); ++i) {
        if (i > 0 && std::fabs(samp[i].t - samp[i-1].t) < 1e-12) continue; 
        T.emplace_back(samp[i].t);
        X.emplace_back(samp[i].p.x());
        Y.emplace_back(samp[i].p.y());
        Z.emplace_back(samp[i].p.z());
    }
    
    if (T.size() < 2) {
        ROS_WARN("[initialAlign] not enough RTK samples for spline.");
        return;
    }

    auto buildSpline = [](const std::vector<double>& t,
                          const std::vector<double>& y,
                          std::vector<double>& M) -> bool {
        const size_t n = t.size();
        M.assign(n, 0.0);
        if (n < 2) return false;
        if (n == 2) return true; // 两点线性
        
        std::vector<double> h(n-1);
        for (size_t i = 0; i < n - 1; ++i) {
            h[i] = t[i+1] - t[i];
            if (h[i] <= 0) return false; // 要严格递增
        }
        
        const size_t m = n - 2;
        std::vector<double> a(m), b(m), c(m), d(m);
        for (size_t i = 0; i < m; ++i) {
            a[i] = h[i] / 6.0;
            b[i] = (h[i] + h[i+1]) / 3.0;
            c[i] = h[i+1] / 6.0;
            d[i] = (y[i+2] - y[i+1]) / h[i+1] - (y[i+1] - y[i]) / h[i];
        }
        
        for (size_t i = 1; i < m; ++i) {
            double w = a[i] / b[i-1];
            b[i] -= w * c[i-1];
            d[i] -= w * d[i-1];
        }
        
        std::vector<double> Min(m, 0.0);
        if (m > 0) {
            Min[m-1] = d[m-1] / b[m-1];
            for (int i = int(m) - 2; i >= 0; --i)
                Min[i] = (d[i] - c[i] * Min[i+1]) / b[i];
        }
        for (size_t i = 0; i < m; ++i) M[i+1] = Min[i]; 
        return true;
    };

    auto evalSpline = [](const std::vector<double>& t,
                         const std::vector<double>& y,
                         const std::vector<double>& M,
                         double tq) -> double {
        const size_t n = t.size();
        if (n == 0) return 0.0;
        if (n == 1) return y[0];
        
        auto derivLeft = [&]() {
            double h = t[1] - t[0];
            return (y[1] - y[0]) / h - (h / 6.0) * (2.0 * M[0] + M[1]);
        };
        auto derivRight = [&]() {
            size_t k = n - 2; double h = t[k+1] - t[k];
            return (y[k+1] - y[k]) / h + (h / 6.0) * (2.0 * M[k+1] + M[k]);
        };
        
        if (tq <= t.front()) return y.front() + derivLeft()  * (tq - t.front());
        if (tq >= t.back())  return y.back()  + derivRight() * (tq - t.back());
        
        size_t k = std::upper_bound(t.begin(), t.end(), tq) - t.begin() - 1;
        double h = t[k+1] - t[k];
        double A = (t[k+1] - tq) / h;
        double B = (tq - t[k]) / h;
        
        return M[k] * (A * A * A) * h / 6.0
             + M[k+1] * (B * B * B) * h / 6.0
             + (y[k]   - M[k] * h * h / 6.0) * A
             + (y[k+1] - M[k+1] * h * h / 6.0) * B;
    };

    std::vector<double> Mx, My, Mz;
    if (!buildSpline(T, X, Mx) || !buildSpline(T, Y, My) || !buildSpline(T, Z, Mz)) {
        ROS_WARN("[initialAlign] spline build failed.");
        return;
    }

    auto find_nearest_gps = [&](double t_query) -> const nav_msgs::Odometry* {
        if (gpsQueue.empty()) return nullptr;

        auto it_lower = std::lower_bound(gpsQueue.begin(), gpsQueue.end(), t_query,
            [](const nav_msgs::Odometry& msg, double t) {
                return msg.header.stamp.toSec() < t;
            });

        if (it_lower == gpsQueue.begin()) {
            return &(*it_lower);
        }

        if (it_lower == gpsQueue.end()) {
            return &(*std::prev(it_lower));
        }

        const nav_msgs::Odometry& after = *it_lower;
        const nav_msgs::Odometry& before = *std::prev(it_lower);

        double dt_after = std::abs(after.header.stamp.toSec() - t_query);
        double dt_before = std::abs(t_query - before.header.stamp.toSec());

        if (dt_after < dt_before) {
            return &after;
        } else {
            return &before;
        }
    };
    
    //Aligning GPS queues and odometer spline
    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i) 
    {
        const double tk = cloudKeyPoses6D->points[i].time;

        nav_msgs::Odometry gps_odom_B;
        gps_odom_B.header.stamp = ros::Time().fromSec(tk);
        gps_odom_B.header.frame_id = "map";
        gps_odom_B.pose.pose.position.x = evalSpline(T, X, Mx, tk);
        gps_odom_B.pose.pose.position.y = evalSpline(T, Y, My, tk);
        gps_odom_B.pose.pose.position.z = evalSpline(T, Z, Mz, tk);
        gps_odom_B.pose.pose.orientation.w = 1.0; 

        const nav_msgs::Odometry* nearest_gps = find_nearest_gps(tk);
        gps_odom_B.pose.covariance[0]  = nearest_gps->pose.covariance[0]; 
        gps_odom_B.pose.covariance[7]  = nearest_gps->pose.covariance[7];  
        gps_odom_B.pose.covariance[14] = nearest_gps->pose.covariance[14];
        gpsQueue_B.push_back(gps_odom_B);
    }

    //Coordinate system transformation
    std::vector<Eigen::Vector3d> A_slam_gps; 
    std::vector<Eigen::Vector3d> B_enu_gps;  

    const double t_start = cloudKeyPoses6D->points[0].time;
    const double t_end_svd = t_start + 50.0; 

    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i) 
    {
        const auto& slam_pose_imu = cloudKeyPoses6D->points[i];
        const double tk = slam_pose_imu.time;
        
        if (tk > t_end_svd) break; 
        if (tk < T.front() || tk > T.back()) continue; 

        Eigen::Vector3d p_enu_gps;
        p_enu_gps.x() = evalSpline(T, X, Mx, tk);
        p_enu_gps.y() = evalSpline(T, Y, My, tk);
        p_enu_gps.z() = evalSpline(T, Z, Mz, tk);
        B_enu_gps.push_back(p_enu_gps);

        gtsam::Pose3 T_slam_imu = initialEstimate.at<gtsam::Pose3>(i); 
        gtsam::Pose3 T_slam_gps = T_slam_imu.compose(T_imu_rtk);
        
        A_slam_gps.push_back(T_slam_gps.translation());
    }

    if (A_slam_gps.size() < 3) {
        ROS_WARN("[initialAlign] too few spline pairs to align.");
        return;
    }

    gtsam::Pose3 T_enu_slam = computeSVD(B_enu_gps, A_slam_gps);

    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i) {
        gtsam::Pose3 T_slam_imu = initialEstimate.at<gtsam::Pose3>(i);
        gtsam::Pose3 T_slam_gps = T_slam_imu.compose(T_imu_rtk);
        gtsam::Pose3 T_enu_gps = T_enu_slam.compose(T_slam_gps);

        initialEstimate.update(i, T_enu_gps);
        
        const auto& p = T_enu_gps.translation();
        const auto& r = T_enu_gps.rotation().rpy();
        cloudKeyPoses6D->points[i].x     = p.x();
        cloudKeyPoses6D->points[i].y     = p.y();
        cloudKeyPoses6D->points[i].z     = p.z();
        cloudKeyPoses6D->points[i].roll  = r(0);
        cloudKeyPoses6D->points[i].pitch = r(1);
        cloudKeyPoses6D->points[i].yaw   = r(2);
    }
    
    ROS_INFO("[initialAlign] Spline-based alignment complete.");
}

//build gtsam graph
void optimization::buildBatchGraph()
{
    gtSAMgraph.resize(0); 

    ROS_INFO("rtk_cox, livo2_RPY_cov, livo2_XYZ_cov: %f, %f, %f", 
             rtk_cov, livo2_RPY_cov, livo2_XYZ_cov);
             
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = 
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << livo2_RPY_cov, livo2_RPY_cov, livo2_RPY_cov, livo2_XYZ_cov, livo2_XYZ_cov, livo2_XYZ_cov).finished());
    gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << rtk_cov, rtk_cov, rtk_cov).finished());

    size_t rtk_idx = 0; 

    for (size_t i = 0; i < initialEstimate.size(); ++i)
    {
        // addpriorfactor and betweenfactor
        if (i == 0)
        {
            gtsam::Pose3 priorPose_T_wa = initialEstimate.at<gtsam::Pose3>(0);
            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, priorPose_T_wa, priorNoise));
        }
        else 
        {
            gtsam::Pose3 poseFrom_T_wa = initialEstimate.at<gtsam::Pose3>(i-1);
            gtsam::Pose3 poseTo_T_wa   = initialEstimate.at<gtsam::Pose3>(i);
            gtsam::Pose3 T_rel_ant_from_svd = poseFrom_T_wa.between(poseTo_T_wa);
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(i-1, i, T_rel_ant_from_svd, odometryNoise));
        }

        // addrtkfactor
        if(addrtkfactor_)
        {
            auto it = gpsQueue_B[i];
            double x = it.pose.pose.position.x;
            double y = it.pose.pose.position.y;
            double z = it.pose.pose.position.z;
            gtsam::Point3 gps_point(x, y, z);

            double cov_x = it.pose.covariance[0]; 
            double cov_y = it.pose.covariance[7]; 
            double cov_z = it.pose.covariance[14]; 

            gtSAMgraph.add(gtsam::GPSFactor(i, gps_point, gps_noise));
        }
    }
} 

void optimization::syncedCallback(const nav_msgs::Odometry::ConstPtr& odomMsg, const sensor_msgs::PointCloud2::ConstPtr& cloudMsg)
{
    timeLaserInoStamp = cloudMsg->header.stamp;
    timeLaserInfoCur  = cloudMsg->header.stamp.toSec();

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odomMsg->pose.pose.orientation, orientation);
    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    transformTobeMapped[0] = roll;
    transformTobeMapped[1] = pitch;
    transformTobeMapped[2] = yaw;
    transformTobeMapped[3] = odomMsg->pose.pose.position.x;
    transformTobeMapped[4] = odomMsg->pose.pose.position.y;
    transformTobeMapped[5] = odomMsg->pose.pose.position.z;
    
    laserCloudSurfLastDS->clear();
    pcl::fromROSMsg(*cloudMsg, *laserCloudSurfLastDS);

    saveKeyFramesAndFactor();
}

void optimization::gpsHandler(const gnss_comm::GnssPVTSolnMsg::ConstPtr& gpsMsg)
{
    Eigen::Vector3d trans_local_;
    static bool first_gps = false;
    if (!first_gps) {
        first_gps = true;
        gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
    }

    // GPS time to UNIX time
    const double GPS_EPOCH_UNIX_TIME = 315964800.0;
    const double SECONDS_PER_WEEK = 604800.0; 
    const double LEAP_SECONDS = 18.0;
    
    int week = gpsMsg->time.week;
    double tow = gpsMsg->time.tow; 
    double total_gps_seconds = (double)week * SECONDS_PER_WEEK + tow;
    double timestamp_sec = total_gps_seconds + GPS_EPOCH_UNIX_TIME - LEAP_SECONDS;
    
    ros::Time stamp;
    stamp.fromSec(timestamp_sec);
    gpstimestamp = timestamp_sec; 

    gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, 
                       trans_local_[0], trans_local_[1], trans_local_[2]);

    nav_msgs::Odometry gps_odom;
    gps_odom.header.stamp = stamp - ros::Duration(gps_offset); 
    gps_odom.header.frame_id = "map";
    gps_odom.pose.pose.position.x = trans_local_[0];
    gps_odom.pose.pose.position.y = trans_local_[1];
    gps_odom.pose.pose.position.z = trans_local_[2];
    gps_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    
    gps_odom.twist.twist.linear.x = gpsMsg->vel_e;
    gps_odom.twist.twist.linear.y = gpsMsg->vel_n;
    gps_odom.twist.twist.linear.z = -gpsMsg->vel_d;

    gps_odom.pose.covariance[0]  = gpsMsg->h_acc; 
    gps_odom.pose.covariance[7]  = gpsMsg->h_acc; 
    gps_odom.pose.covariance[14] = gpsMsg->v_acc; 

    pubGpsOdom.publish(gps_odom);
    gpsQueue.push_back(gps_odom);
}

void optimization::savekeyframescan()
{
    for (size_t i = 0; i < surfCloudKeyFrames.size(); ++i)
    {
        PointCloudXYZRGB::Ptr keyframe_cloud(new PointCloudXYZRGB());
        *keyframe_cloud = *surfCloudKeyFrames[i];
        Eigen::Affine3f transform = pclPointToAffine3f(cloudKeyPoses6D->points[i]);
        pcl::transformPointCloud(*keyframe_cloud, *keyframe_cloud, transform);
        std::string save_path = keyframe_scan_pcd_path_ + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(save_path, *keyframe_cloud);
    }
}

void optimization::saveOptimizedGlobalMap()
{
    PointCloudXYZRGB::Ptr globalMapCloud(new PointCloudXYZRGB());

    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i)
    {
        Eigen::Affine3f transform = pclPointToAffine3f(cloudKeyPoses6D->points[i]);
        PointCloudXYZRGB::Ptr original_cloud = surfCloudKeyFrames[i];
        PointCloudXYZRGB::Ptr transformed_cloud(new PointCloudXYZRGB());
        pcl::transformPointCloud(*original_cloud, *transformed_cloud, transform);
        *globalMapCloud += *transformed_cloud;
    }
    
    std::string save_path;
    if (is_optimized) 
    {
        save_path = global_map_pcd_path_ + "after_optimization.pcd";
    } 
    else
    {
        save_path = global_map_pcd_path_ + "before_optimization.pcd";
    }
    pcl::io::savePCDFileBinary(save_path, *globalMapCloud);
}

void optimization::writeOptimizedTumTrajectory() {
    if (cloudKeyPoses6D->empty()) {
        ROS_WARN("No optimized keyframes to write to TUM file.");
        return;
    }

    std::string final_path = opt_tum_output_path_;
    std::ofstream tum_file(final_path);
    tum_file << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    tum_file << std::fixed << std::setprecision(6);
    for (const auto& pose : cloudKeyPoses6D->points) {
        tf::Quaternion q = tf::createQuaternionFromRPY(pose.roll, pose.pitch, pose.yaw);
        tum_file << pose.time << " "
                 << pose.x << " " << pose.y << " " << pose.z << " "
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    tum_file.close();
}

void optimization::writeRtkTumTrajectory() {
    if (gpsQueue.empty()) {
        ROS_WARN("No RTK data to write to TUM file.");
        return;
    }
    
    std::ofstream tum_file(gps_tum_output_path_);
    tum_file << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    tum_file << std::fixed << std::setprecision(6);

    for (const auto& odom_msg : gpsQueue) {
        tum_file << odom_msg.header.stamp.toSec() << " " 
                 << odom_msg.pose.pose.position.x << " " 
                 << odom_msg.pose.pose.position.y << " " 
                 << odom_msg.pose.pose.position.z << " "
                 << odom_msg.pose.pose.orientation.x << " " 
                 << odom_msg.pose.pose.orientation.y << " " 
                 << odom_msg.pose.pose.orientation.z << " " 
                 << odom_msg.pose.pose.orientation.w << std::endl;
    }

    tum_file.close();
}

void optimization::loadData(const std::string& data_dir)
{
    ROS_INFO("Loading data from directory: %s", data_dir.c_str());

    // Load RTK/GPS  
    std::string rtk_path = data_dir + "/rtk.txt"; 
    std::ifstream rtk_file(rtk_path);

    if (rtk_file.is_open()) {
        std::string line;
        int rtk_count = 0;
        gpsQueue.clear(); 

        while (std::getline(rtk_file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::stringstream ss(line);
            double t, x, y, z, vx, vy, vz, h_acc, v_acc;
            ss >> t >> x >> y >> z ;
            
            nav_msgs::Odometry gps_msg;
            gps_msg.header.stamp = ros::Time().fromSec(t) - ros::Duration(gps_offset);
            gps_msg.header.frame_id = "map"; 

            gps_msg.pose.pose.position.x = x;
            gps_msg.pose.pose.position.y = y;
            gps_msg.pose.pose.position.z = z;
            gps_msg.pose.pose.orientation.w = 1.0;

            gpsQueue.push_back(gps_msg);
            rtk_count++;
        }
        rtk_file.close();
        ROS_INFO("Loaded %d RTK measurements.", rtk_count);
    } else {
        ROS_WARN("Failed to open RTK file: %s", rtk_path.c_str());
    }

    // Load KeyFrames (Odom + PCD) 
    std::string odom_path = data_dir + "/odom.txt";
    std::string cov_path  = data_dir + "/cov.txt";
    std::string pcd_dir   = data_dir + "pcd/";

    std::ifstream odom_file(odom_path);
    std::ifstream cov_file(cov_path);

    if (!odom_file.is_open() || !cov_file.is_open()) {
        ROS_ERROR("Failed to open trajectory files. Check path: %s", data_dir.c_str());
        return;
    }

    std::string line_odom, line_cov;
    int processed_count = 0;

    while (std::getline(odom_file, line_odom) && std::getline(cov_file, line_cov)) {
        
        if (line_odom.empty() || line_odom[0] == '#') continue;
        if (line_cov.empty() || line_cov[0] == '#') continue; 

        std::stringstream ss_odom(line_odom);
        std::stringstream ss_cov(line_cov);

        double t_odom, t_cov;
        ss_odom >> t_odom;
        ss_cov >> t_cov;

        if (std::abs(t_odom - t_cov) > 1e-5) {
            ROS_ERROR("Critical Error: Timestamp mismatch at line %d!", processed_count + 1);
            ROS_ERROR("Odom Time: %.6f, Cov Time: %.6f", t_odom, t_cov);
            break; 
        }

        // Odom 
        double tx, ty, tz, qx, qy, qz, qw;
        ss_odom >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        timeLaserInfoCur = t_odom;
        timeLaserInoStamp = ros::Time().fromSec(t_odom);

        tf::Quaternion orientation(qx, qy, qz, qw);
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        
        transformTobeMapped[0] = roll;
        transformTobeMapped[1] = pitch;
        transformTobeMapped[2] = yaw;
        transformTobeMapped[3] = tx;
        transformTobeMapped[4] = ty;
        transformTobeMapped[5] = tz;

        // PCD
        std::stringstream ss_filename;
        ss_filename << std::setw(6) << std::setfill('0') << processed_count;
        std::string pcd_path = pcd_dir + ss_filename.str() + ".pcd";

        laserCloudSurfLastDS->clear();
        if (pcl::io::loadPCDFile(pcd_path, *laserCloudSurfLastDS) == -1) {
            ROS_ERROR("Missing PCD file for timestamp %.6f: %s", t_odom, pcd_path.c_str());
            continue; 
        }

        saveKeyFramesAndFactor();
        processed_count++;
    }
    
    odom_file.close();
    cov_file.close();
    ROS_INFO("Load complete. Total frames: %d", processed_count);
}