#include "optimization.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <numeric>

#include <sys/select.h>
#include <unistd.h>

namespace {
struct VelocitySeries
{
    std::vector<double> times;
    std::vector<double> values;
};

std::string shellQuote(const std::string& value)
{
    std::string quoted = "'";
    for (char c : value) {
        if (c == '\'') {
            quoted += "'\\''";
        } else {
            quoted += c;
        }
    }
    quoted += "'";
    return quoted;
}

void ensureDirectory(const std::string& path)
{
    if (path.empty()) {
        return;
    }
    const std::string command = "mkdir -p " + shellQuote(path);
    const int rc = std::system(command.c_str());
    if (rc != 0) {
        ROS_WARN("[optimization] failed to create output directory: %s", path.c_str());
    }
}

bool interpolateLinear(const VelocitySeries& series, double t, double& value)
{
    if (series.times.size() < 2 || t < series.times.front() || t > series.times.back()) {
        return false;
    }

    auto upper = std::upper_bound(series.times.begin(), series.times.end(), t);
    if (upper == series.times.begin()) {
        value = series.values.front();
        return true;
    }
    if (upper == series.times.end()) {
        value = series.values.back();
        return true;
    }

    const size_t i = static_cast<size_t>(upper - series.times.begin());
    const double t0 = series.times[i - 1];
    const double t1 = series.times[i];
    if (t1 <= t0) {
        return false;
    }

    const double a = (t - t0) / (t1 - t0);
    value = series.values[i - 1] * (1.0 - a) + series.values[i] * a;
    return true;
}

bool waitForOptimizationTrigger(const std::atomic_bool& shutdown_requested)
{
    while (ros::ok() && !shutdown_requested.load(std::memory_order_relaxed)) {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(STDIN_FILENO, &read_set);

        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;

        const int rc = select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout);
        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            ROS_WARN("[Offline Optimization] stdin wait failed, optimization trigger disabled.");
            return false;
        }

        if (rc == 0 || !FD_ISSET(STDIN_FILENO, &read_set)) {
            continue;
        }

        char ch = '\0';
        const ssize_t nread = read(STDIN_FILENO, &ch, 1);
        if (nread <= 0) {
            ROS_INFO("[Offline Optimization] stdin closed before optimization trigger.");
            return false;
        }
        if (ch == '\n') {
            return true;
        }
    }

    return false;
}

void smoothVelocitySeries(VelocitySeries& series)
{
    if (series.values.size() < 5) {
        return;
    }

    std::vector<double> smoothed(series.values.size(), 0.0);
    for (size_t i = 0; i < series.values.size(); ++i) {
        const size_t begin = (i < 2) ? 0 : i - 2;
        const size_t end = std::min(series.values.size() - 1, i + 2);
        double sum = 0.0;
        for (size_t j = begin; j <= end; ++j) {
            sum += series.values[j];
        }
        smoothed[i] = sum / static_cast<double>(end - begin + 1);
    }
    series.values.swap(smoothed);
}

bool normalizeVelocitySeries(VelocitySeries& series)
{
    if (series.values.size() < 20) {
        return false;
    }

    const double mean = std::accumulate(series.values.begin(), series.values.end(), 0.0)
                      / static_cast<double>(series.values.size());
    double sq_sum = 0.0;
    for (double v : series.values) {
        const double d = v - mean;
        sq_sum += d * d;
    }

    const double stddev = std::sqrt(sq_sum / static_cast<double>(series.values.size()));
    if (!std::isfinite(stddev) || stddev < 0.05) {
        return false;
    }

    for (double& v : series.values) {
        v = (v - mean) / stddev;
    }
    return true;
}

VelocitySeries buildVelocitySeries(const std::vector<std::vector<double>>& data, double time_shift)
{
    VelocitySeries series;
    if (data.size() < 3) {
        return series;
    }

    constexpr int IDX_TIME = 0;
    constexpr int IDX_X = 1;
    constexpr int IDX_Y = 2;
    constexpr int IDX_Z = 3;
    constexpr double MIN_DT = 1e-3;
    constexpr double MAX_DT = 2.0;
    constexpr double MAX_SPEED = 100.0;

    series.times.reserve(data.size() - 1);
    series.values.reserve(data.size() - 1);

    for (size_t i = 1; i < data.size(); ++i) {
        if (data[i].size() < 4 || data[i - 1].size() < 4) {
            continue;
        }

        const double t0 = data[i - 1][IDX_TIME];
        const double t1 = data[i][IDX_TIME];
        const double dt = t1 - t0;
        if (!std::isfinite(dt) || dt < MIN_DT || dt > MAX_DT) {
            continue;
        }

        const double dx = data[i][IDX_X] - data[i - 1][IDX_X];
        const double dy = data[i][IDX_Y] - data[i - 1][IDX_Y];
        const double dz = data[i][IDX_Z] - data[i - 1][IDX_Z];
        const double speed = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
        if (!std::isfinite(speed) || speed > MAX_SPEED) {
            continue;
        }

        series.times.push_back(0.5 * (t0 + t1) + time_shift);
        series.values.push_back(speed);
    }

    smoothVelocitySeries(series);
    return series;
}

VelocitySeries buildVelocitySeriesFromSamples(const std::vector<std::vector<double>>& data, double time_shift)
{
    VelocitySeries series;
    if (data.size() < 3) {
        return series;
    }

    constexpr int IDX_TIME = 0;
    constexpr int IDX_VX = 1;
    constexpr int IDX_VY = 2;
    constexpr int IDX_VZ = 3;
    constexpr double MIN_DT = 1e-3;
    constexpr double MAX_DT = 2.0;
    constexpr double MAX_SPEED = 100.0;

    series.times.reserve(data.size());
    series.values.reserve(data.size());

    double last_time = std::numeric_limits<double>::quiet_NaN();
    for (const auto& sample : data) {
        if (sample.size() < 4) {
            continue;
        }

        const double t = sample[IDX_TIME];
        if (!std::isfinite(t)) {
            continue;
        }
        if (std::isfinite(last_time)) {
            const double dt = t - last_time;
            if (dt < MIN_DT || dt > MAX_DT) {
                last_time = t;
                continue;
            }
        }

        const double vx = sample[IDX_VX];
        const double vy = sample[IDX_VY];
        const double vz = sample[IDX_VZ];
        const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (!std::isfinite(speed) || speed > MAX_SPEED) {
            last_time = t;
            continue;
        }

        series.times.push_back(t + time_shift);
        series.values.push_back(speed);
        last_time = t;
    }

    smoothVelocitySeries(series);
    return series;
}

double scoreVelocityOffset(const VelocitySeries& slam,
                           const VelocitySeries& gps,
                           double fine_offset,
                           int& pair_count)
{
    double sum_sg = 0.0;
    double sum_ss = 0.0;
    double sum_gg = 0.0;
    pair_count = 0;

    for (size_t i = 0; i < slam.times.size(); ++i) {
        double gps_value = 0.0;
        if (!interpolateLinear(gps, slam.times[i] - fine_offset, gps_value)) {
            continue;
        }

        const double slam_value = slam.values[i];
        sum_sg += slam_value * gps_value;
        sum_ss += slam_value * slam_value;
        sum_gg += gps_value * gps_value;
        ++pair_count;
    }

    if (pair_count < 30 || sum_ss <= 1e-12 || sum_gg <= 1e-12) {
        return -std::numeric_limits<double>::infinity();
    }

    return sum_sg / std::sqrt(sum_ss * sum_gg);
}

void writeVelocityAlignmentCsv(const std::string& csv_path,
                               const std::vector<std::vector<double>>& gps_position_data,
                               const std::vector<std::vector<double>>& slam_position_data,
                               const std::vector<std::vector<double>>& gps_velocity_data,
                               const std::vector<std::vector<double>>& slam_velocity_data,
                               const std::string& velocity_source,
                               double coarse_time_offset,
                               double applied_fine_offset)
{
    VelocitySeries slam_velocity;
    VelocitySeries gps_velocity;
    if (velocity_source == "direct_twist") {
        slam_velocity = buildVelocitySeriesFromSamples(slam_velocity_data, 0.0);
        gps_velocity = buildVelocitySeriesFromSamples(gps_velocity_data, coarse_time_offset);
    } else {
        slam_velocity = buildVelocitySeries(slam_position_data, 0.0);
        gps_velocity = buildVelocitySeries(gps_position_data, coarse_time_offset);
    }

    std::ofstream csv_file(csv_path);
    if (!csv_file.is_open()) {
        return;
    }

    csv_file << std::fixed << std::setprecision(9);
    csv_file << "time_sec,livo_speed_mps,rtk_speed_coarse_mps,rtk_speed_applied_mps\n";
    for (size_t i = 0; i < slam_velocity.times.size(); ++i) {
        const double t = slam_velocity.times[i];
        double gps_speed_coarse = 0.0;
        double gps_speed_applied = 0.0;
        const bool has_coarse = interpolateLinear(gps_velocity, t, gps_speed_coarse);
        const bool has_applied = interpolateLinear(gps_velocity, t - applied_fine_offset, gps_speed_applied);

        csv_file << t << "," << slam_velocity.values[i] << ",";
        if (has_coarse) {
            csv_file << gps_speed_coarse;
        }
        csv_file << ",";
        if (has_applied) {
            csv_file << gps_speed_applied;
        }
        csv_file << "\n";
    }
}
} // namespace

optimization::optimization(ros::NodeHandle &nh)
{
    nh.param<std::string>("laserMapping/outputfilepath", outputfilepath, "");
    debug_optdata_path_ = outputfilepath + "/debug/";
    opt_tum_output_path_ = outputfilepath + "/TUM/opt_trajectory_after.txt";
    livo_tum_before_output_path_ = outputfilepath + "/TUM/livo_trajectory_before.txt";
    rtk_tum_output_path_ = outputfilepath + "/TUM/rtk_trajectory_time_aligned.txt";
    opt_vel_output_path_ = outputfilepath + "/vel/opt_vel.txt";
    gps_vel_output_path_ = outputfilepath + "/vel/gps_vel.txt";
    global_map_pcd_path_ = outputfilepath + "/global_pcd/";
    keyframe_scan_pcd_path_ = outputfilepath + "/scan_pcd/";

    if (!outputfilepath.empty()) {
        ensureDirectory(debug_optdata_path_);
        ensureDirectory(debug_optdata_path_ + "pcd/");
        ensureDirectory(outputfilepath + "/TUM/");
        ensureDirectory(outputfilepath + "/vel/");
        ensureDirectory(global_map_pcd_path_);
        ensureDirectory(keyframe_scan_pcd_path_);
    }

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

    keyFrames.clear();

    if(debug_mode)
    {
        loadData(debug_optdata_path_);
    }
    last_keyframe_wall_time_ = ros::WallTime::now();

    ROS_INFO("Optimization mode: [OFFLINE]. Accumulating factors for batch optimization.");
    optimization_thread_ = std::thread(&optimization::offlineOptimizationTask, this);
}

optimization::~optimization() 
{
    optimization_shutdown_requested_.store(true, std::memory_order_relaxed);
    if (optimization_thread_.joinable()) {
        optimization_thread_.join();
    }
}

//Optimize threads
void optimization::offlineOptimizationTask() {
    if (!waitForOptimizationTrigger(optimization_shutdown_requested_)) {
        return;
    }

    waitForKeyFrameIdle(3.0);
    {
        std::lock_guard<std::mutex> lock(mutex);
        accepting_keyframes_ = false;
    }
    std::cout << "[Offline Optimization] Starting batch optimization..." << std::endl;
    initialAlign();
    std::cout << "[Offline Optimization] Initial alignment done." << std::endl;
    writeTumTrajectory(livo_tum_before_output_path_);
    saveOptimizedGlobalMap();
    std::cout << "[Offline Optimization] Initial global map and tum saved." << std::endl;
    
    gtsam::LevenbergMarquardtParams params;
    params.orderingType = gtsam::Ordering::METIS;
    params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
    params.setMaxIterations(500);
    params.setRelativeErrorTol(1e-9);
    params.setAbsoluteErrorTol(1e-9);
    
    buildBatchGraph();
    gtsam::LevenbergMarquardtOptimizer optimizer(gtSAMgraph, initialEstimate);
    gtsam::Values result = optimizer.optimize();
    initialEstimate = result;
    std::cout << "[Offline Optimization] First optimization pass done." <<  std::endl;

    is_optimized = true;
    int numPoses = initialEstimate.size();

    for (int i = 0; i < numPoses; ++i) {
        gtsam::Pose3 rtk_optimizedPose = initialEstimate.at<gtsam::Pose3>(i);
        keyFrames[i].pose = rtk_optimizedPose;
    }

    writeOptimizedTumTrajectory();
    std::cout << "[Offline Optimization] Optimized TUM trajectory saved "  << std::endl;
    writeRtkTumTrajectory();
    std::cout << "[Offline Optimization] RTK TUM trajectory saved " << std::endl;

    for (int i = 0; i < numPoses; ++i) {
        gtsam::Pose3 rtk_optimizedPose = initialEstimate.at<gtsam::Pose3>(i);
        gtsam::Pose3 imu_optimizedPose = rtk_optimizedPose.compose((T_imu_rtk).inverse());
        keyFrames[i].pose = imu_optimizedPose;
    }

    std::cout << "[Offline Optimization] Saving maps and trajectories..." << std::endl;
    //savekeyframescan();
    saveOptimizedGlobalMap();
    std::cout << "[Offline Optimization] Global map saved " << std::endl;

    std::cout << "[Offline Optimization] Finished." << std::endl;
}

void optimization::waitForKeyFrameIdle(double idle_seconds)
{
    const ros::WallDuration idle_duration(idle_seconds);
    size_t last_count = 0;
    while (ros::ok()) {
        size_t current_count = 0;
        ros::WallTime last_keyframe_time;
        {
            std::lock_guard<std::mutex> lock(mutex);
            current_count = keyFrames.size();
            last_keyframe_time = last_keyframe_wall_time_;
        }

        const ros::WallDuration idle_time = ros::WallTime::now() - last_keyframe_time;
        if (current_count > 0 && idle_time >= idle_duration) {
            ROS_INFO("[Offline Optimization] Keyframe buffer idle for %.2fs, frozen at %zu frames.",
                     idle_time.toSec(), current_count);
            return;
        }

        if (current_count != last_count) {
            ROS_INFO("[Offline Optimization] Waiting for keyframe buffer to drain: %zu frames.", current_count);
            last_count = current_count;
        }
        ros::WallDuration(0.5).sleep();
    }
}

void optimization::saveKeyFrameAndFactor(const gtsam::Pose3& pose,
                                         double time,
                                         const Eigen::Vector3d& velocity,
                                         const PointCloudXYZRGB::Ptr& cloud)
{
    std::lock_guard<std::mutex> lock(mutex);
    if (!accepting_keyframes_) {
        return;
    }

    const size_t keyframe_index = keyFrames.size();
    initialEstimate.insert(keyframe_index, pose);

    KeyFrame keyframe;
    keyframe.time = time;
    keyframe.pose = pose;
    keyframe.velocity = velocity;
    keyframe.cloud.reset(new PointCloudXYZRGB());
    if (cloud) {
        pcl::copyPointCloud(*cloud, *keyframe.cloud);
    }

    keyFrames.push_back(keyframe);
    last_keyframe_wall_time_ = ros::WallTime::now();
}

//DTW
double optimization::calculateDtwTimeOffset(
    const std::vector<std::vector<double>>& gpsdata,
    const std::vector<std::vector<double>>& slamdata)
{
    if (gpsdata.size() < 3 || slamdata.size() < 3) {
        ROS_WARN("[initialAlign] not enough samples for DTW time offset.");
        return std::numeric_limits<double>::quiet_NaN();
    }

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

    if (gps_pos_norms.empty() || slam_pos_norms.empty()) {
        ROS_WARN("[initialAlign] no moving samples for DTW time offset.");
        return std::numeric_limits<double>::quiet_NaN();
    }

    sample1 = slam_pos_norms.data();
    sample2 = gps_pos_norms.data();

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
        return std::numeric_limits<double>::quiet_NaN();
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

double optimization::estimateVelocityTimeOffset(
    const std::vector<std::vector<double>>& gps_position_data,
    const std::vector<std::vector<double>>& slam_position_data,
    const std::vector<std::vector<double>>& gps_velocity_data,
    const std::vector<std::vector<double>>& slam_velocity_data,
    double coarse_time_offset,
    double& best_score,
    double& zero_score,
    int& best_pair_count,
    std::string& velocity_source)
{
    best_score = -std::numeric_limits<double>::infinity();
    zero_score = -std::numeric_limits<double>::infinity();
    best_pair_count = 0;
    velocity_source = "none";

    auto canNormalize = [](const VelocitySeries& slam, const VelocitySeries& gps) {
        VelocitySeries slam_copy = slam;
        VelocitySeries gps_copy = gps;
        return normalizeVelocitySeries(slam_copy) && normalizeVelocitySeries(gps_copy);
    };

    VelocitySeries direct_slam_velocity = buildVelocitySeriesFromSamples(slam_velocity_data, 0.0);
    VelocitySeries direct_gps_velocity = buildVelocitySeriesFromSamples(gps_velocity_data, coarse_time_offset);

    VelocitySeries slam_velocity;
    VelocitySeries gps_velocity;
    if (canNormalize(direct_slam_velocity, direct_gps_velocity)) {
        slam_velocity = direct_slam_velocity;
        gps_velocity = direct_gps_velocity;
        velocity_source = "direct_twist";
    } else {
        slam_velocity = buildVelocitySeries(slam_position_data, 0.0);
        gps_velocity = buildVelocitySeries(gps_position_data, coarse_time_offset);
        velocity_source = "position_difference";
    }

    std::ofstream slam_file(opt_vel_output_path_);
    if (slam_file.is_open()) {
        slam_file << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < slam_velocity.values.size(); ++i) {
            slam_file << slam_velocity.times[i] << " " << slam_velocity.values[i] << std::endl;
        }
        slam_file.close();
    }

    std::ofstream gps_file(gps_vel_output_path_);
    if (gps_file.is_open()) {
        gps_file << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < gps_velocity.values.size(); ++i) {
            gps_file << gps_velocity.times[i] << " " << gps_velocity.values[i] << std::endl;
        }
        gps_file.close();
    }

    if (!normalizeVelocitySeries(slam_velocity) || !normalizeVelocitySeries(gps_velocity)) {
        ROS_WARN("[initialAlign] velocity time offset skipped: not enough speed variation.");
        velocity_source = "unusable";
        return std::numeric_limits<double>::quiet_NaN();
    }

    constexpr double SEARCH_RANGE_SEC = 10.0;
    constexpr double SEARCH_STEP_SEC = 0.05;

    int zero_pair_count = 0;
    zero_score = scoreVelocityOffset(slam_velocity, gps_velocity, 0.0, zero_pair_count);

    std::ofstream score_file(debug_optdata_path_ + "time_alignment_score.csv");
    if (score_file.is_open()) {
        score_file << std::fixed << std::setprecision(9);
        score_file << "fine_offset_sec,score,pairs\n";
    }

    double best_offset = 0.0;
    for (int step = static_cast<int>(std::round(-SEARCH_RANGE_SEC / SEARCH_STEP_SEC));
         step <= static_cast<int>(std::round(SEARCH_RANGE_SEC / SEARCH_STEP_SEC));
         ++step) {
        const double candidate_offset = static_cast<double>(step) * SEARCH_STEP_SEC;
        int pair_count = 0;
        const double score = scoreVelocityOffset(slam_velocity, gps_velocity, candidate_offset, pair_count);
        if (score_file.is_open()) {
            score_file << candidate_offset << ",";
            if (std::isfinite(score)) {
                score_file << score;
            } else {
                score_file << "nan";
            }
            score_file << "," << pair_count << "\n";
        }
        if (score > best_score) {
            best_score = score;
            best_offset = candidate_offset;
            best_pair_count = pair_count;
        }
    }

    if (!std::isfinite(best_score)) {
        ROS_WARN("[initialAlign] velocity time offset skipped: no overlapping velocity samples.");
        return std::numeric_limits<double>::quiet_NaN();
    }

    return best_offset;
}

//Spacetime synchronization
void optimization::initialAlign()
{
    //using spline
    std::cout << " [initialAlign] initial align (using Spline)... " << std::endl;
    gpsQueue_B.clear();

    if (keyFrames.empty() || gpsQueue.empty()) {
        ROS_WARN("[initialAlign] empty buffers.");
        return;
    }

    std::vector<std::vector<double>> slam_position_data;
    std::vector<std::vector<double>> slam_velocity_data;
    slam_position_data.reserve(keyFrames.size());
    slam_velocity_data.reserve(keyFrames.size());
    for (const auto& keyframe : keyFrames) {
        const auto& p = keyframe.pose.translation();
        slam_position_data.push_back({keyframe.time, p.x(), p.y(), p.z()});
        slam_velocity_data.push_back({
            keyframe.time,
            keyframe.velocity.x(),
            keyframe.velocity.y(),
            keyframe.velocity.z()});
    }

    std::vector<std::vector<double>> gps_position_data;
    std::vector<std::vector<double>> gps_velocity_data;
    gps_position_data.reserve(gpsQueue.size());
    gps_velocity_data.reserve(gpsQueue.size());
    for (const auto& g : gpsQueue) {
        const auto& p = g.pose.pose.position;
        const auto& v = g.twist.twist.linear;
        gps_position_data.push_back({g.header.stamp.toSec(), p.x, p.y, p.z});
        gps_velocity_data.push_back({g.header.stamp.toSec(), v.x, v.y, v.z});
    }

    const double slam_start = slam_position_data.front()[0];
    const double slam_end = slam_position_data.back()[0];
    const double gps_start = gps_position_data.front()[0];
    const double gps_end = gps_position_data.back()[0];
    const double raw_overlap = std::min(slam_end, gps_end) - std::max(slam_start, gps_start);
    const bool epoch_mismatch = raw_overlap < 5.0
                             || std::fabs(slam_start - gps_start) > 100.0
                             || std::fabs(slam_end - gps_end) > 100.0;

    const double coarse_time_offset = epoch_mismatch ? (slam_start - gps_start) : 0.0;
    double best_velocity_score = -std::numeric_limits<double>::infinity();
    double zero_velocity_score = -std::numeric_limits<double>::infinity();
    int best_velocity_pairs = 0;
    std::string velocity_source;
    const double fine_time_offset = estimateVelocityTimeOffset(
        gps_position_data, slam_position_data,
        gps_velocity_data, slam_velocity_data,
        coarse_time_offset, best_velocity_score, zero_velocity_score,
        best_velocity_pairs, velocity_source);

    double applied_time_offset = coarse_time_offset;
    bool fine_time_offset_used = false;
    if (std::isfinite(fine_time_offset) && std::fabs(fine_time_offset) <= 10.0 &&
        std::isfinite(best_velocity_score) && best_velocity_score > 0.25) {
        const bool score_improved = !std::isfinite(zero_velocity_score) ||
                                  (best_velocity_score - zero_velocity_score) > 0.05;
        if (epoch_mismatch || score_improved) {
            applied_time_offset += fine_time_offset;
            fine_time_offset_used = true;
        }
    }

    if (std::fabs(applied_time_offset) > 1e-9) {
        const ros::Duration time_offset(applied_time_offset);
        for (auto& g : gpsQueue) {
            g.header.stamp += time_offset;
        }
    }

    writeVelocityAlignmentCsv(debug_optdata_path_ + "time_alignment_velocity.csv",
                              gps_position_data, slam_position_data,
                              gps_velocity_data, slam_velocity_data,
                              velocity_source, coarse_time_offset,
                              fine_time_offset_used ? fine_time_offset : 0.0);

    ROS_INFO("[initialAlign] time offset raw_overlap: %.6f, coarse: %.6f, velocity_fine: %.6f, applied: %.6f, source: %s, score: %.3f, zero_score: %.3f, pairs: %d",
             raw_overlap, coarse_time_offset, fine_time_offset, applied_time_offset,
             velocity_source.c_str(), best_velocity_score, zero_velocity_score, best_velocity_pairs);

    std::ofstream time_offset_file(debug_optdata_path_ + "time_offset.txt");
    if (time_offset_file.is_open()) {
        time_offset_file << std::fixed << std::setprecision(9);
        time_offset_file << "slam_start " << slam_start << "\n";
        time_offset_file << "slam_end " << slam_end << "\n";
        time_offset_file << "gps_start_before " << gps_start << "\n";
        time_offset_file << "gps_end_before " << gps_end << "\n";
        time_offset_file << "raw_overlap_sec " << raw_overlap << "\n";
        time_offset_file << "epoch_mismatch " << (epoch_mismatch ? 1 : 0) << "\n";
        time_offset_file << "coarse_time_offset_sec " << coarse_time_offset << "\n";
        time_offset_file << "velocity_fine_time_offset_sec " << fine_time_offset << "\n";
        time_offset_file << "velocity_fine_used " << (fine_time_offset_used ? 1 : 0) << "\n";
        time_offset_file << "velocity_source " << velocity_source << "\n";
        time_offset_file << "velocity_best_score " << best_velocity_score << "\n";
        time_offset_file << "velocity_zero_score " << zero_velocity_score << "\n";
        time_offset_file << "velocity_best_pairs " << best_velocity_pairs << "\n";
        time_offset_file << "applied_time_offset_sec " << applied_time_offset << "\n";
        time_offset_file << "gps_start_after " << gpsQueue.front().header.stamp.toSec() << "\n";
        time_offset_file << "gps_end_after " << gpsQueue.back().header.stamp.toSec() << "\n";
        time_offset_file.close();
    }

    const std::string plot_script = std::string(ROOT_DIR) + "scripts/plot_time_alignment.py";
    const std::string plot_command = "python3 " + shellQuote(plot_script) +
                                     " --debug-dir " + shellQuote(debug_optdata_path_) +
                                     " >/dev/null 2>&1";
    const int plot_rc = std::system(plot_command.c_str());
    if (plot_rc != 0) {
        ROS_WARN("[initialAlign] failed to generate time alignment plot: %s", plot_script.c_str());
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
    for (size_t i = 0; i < keyFrames.size(); ++i)
    {
        const double tk = keyFrames[i].time;

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

    const double t_start = keyFrames.front().time;
    const double t_end_svd = t_start + 50.0; 

    for (size_t i = 0; i < keyFrames.size(); ++i)
    {
        const auto& keyframe = keyFrames[i];
        const double tk = keyframe.time;
        
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

    for (size_t i = 0; i < keyFrames.size(); ++i) {
        gtsam::Pose3 T_slam_imu = initialEstimate.at<gtsam::Pose3>(i);
        gtsam::Pose3 T_slam_gps = T_slam_imu.compose(T_imu_rtk);
        gtsam::Pose3 T_enu_gps = T_enu_slam.compose(T_slam_gps);

        initialEstimate.update(i, T_enu_gps);
        keyFrames[i].pose = T_enu_gps;
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
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << rtk_cov, rtk_cov, 1).finished());

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

void optimization::syncedCallback(const nav_msgs::Odometry::ConstPtr& odomMsg, const sensor_msgs::PointCloud2::ConstPtr& cloudMsg)
{
    timeLaserInoStamp = cloudMsg->header.stamp;
    timeLaserInfoCur  = cloudMsg->header.stamp.toSec();

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odomMsg->pose.pose.orientation, orientation);
    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    gtsam::Pose3 pose(
        gtsam::Rot3::RzRyRx(roll, pitch, yaw),
        gtsam::Point3(odomMsg->pose.pose.position.x,
                      odomMsg->pose.pose.position.y,
                      odomMsg->pose.pose.position.z));

    const auto& velocity_msg = odomMsg->twist.twist.linear;
    Eigen::Vector3d velocity(velocity_msg.x, velocity_msg.y, velocity_msg.z);

    PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB());
    pcl::fromROSMsg(*cloudMsg, *cloud);

    saveKeyFrameAndFactor(pose, timeLaserInfoCur, velocity, cloud);
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
    for (size_t i = 0; i < keyFrames.size(); ++i)
    {
        PointCloudXYZRGB::Ptr keyframe_cloud(new PointCloudXYZRGB());
        *keyframe_cloud = *keyFrames[i].cloud;
        Eigen::Affine3f transform = poseToAffine3f(keyFrames[i].pose);
        pcl::transformPointCloud(*keyframe_cloud, *keyframe_cloud, transform);
        std::string save_path = keyframe_scan_pcd_path_ + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(save_path, *keyframe_cloud);
    }
}

void optimization::saveOptimizedGlobalMap()
{
    PointCloudXYZRGB::Ptr globalMapCloud(new PointCloudXYZRGB());

    for (size_t i = 0; i < keyFrames.size(); ++i)
    {
        Eigen::Affine3f transform = poseToAffine3f(keyFrames[i].pose);
        PointCloudXYZRGB::Ptr original_cloud = keyFrames[i].cloud;
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

void optimization::writeTumTrajectory(const std::string& path) {
    if (keyFrames.empty()) {
        ROS_WARN("No optimized keyframes to write to TUM file.");
        return;
    }

    std::ofstream tum_file(path);
    tum_file << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    tum_file << std::fixed << std::setprecision(6);
    for (const auto& keyframe : keyFrames) {
        const auto& t = keyframe.pose.translation();
        const auto q = keyframe.pose.rotation().toQuaternion();
        tum_file << keyframe.time << " "
                 << t.x() << " " << t.y() << " " << t.z() << " "
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    tum_file.close();
}

void optimization::writeOptimizedTumTrajectory() {
    writeTumTrajectory(opt_tum_output_path_);
}

void optimization::writeRtkTumTrajectory() {
    if (gpsQueue.empty()) {
        ROS_WARN("No RTK data to write to TUM file.");
        return;
    }
    
    std::ofstream tum_file(rtk_tum_output_path_);
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
            double t, x, y, z;
            double vx = 0.0, vy = 0.0, vz = 0.0;
            double h_acc = 0.0, v_acc = 0.0;
            ss >> t >> x >> y >> z;
            ss >> vx >> vy >> vz >> h_acc >> v_acc;
            
            nav_msgs::Odometry gps_msg;
            gps_msg.header.stamp = ros::Time().fromSec(t) - ros::Duration(gps_offset);
            gps_msg.header.frame_id = "map"; 

            gps_msg.pose.pose.position.x = x;
            gps_msg.pose.pose.position.y = y;
            gps_msg.pose.pose.position.z = z;
            gps_msg.pose.pose.orientation.w = 1.0;
            gps_msg.twist.twist.linear.x = vx;
            gps_msg.twist.twist.linear.y = vy;
            gps_msg.twist.twist.linear.z = vz;
            gps_msg.pose.covariance[0] = h_acc;
            gps_msg.pose.covariance[7] = h_acc;
            gps_msg.pose.covariance[14] = v_acc;

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
    std::string pcd_dir   = data_dir + "/pcd/";

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
        double vx = 0.0, vy = 0.0, vz = 0.0;
        ss_odom >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        ss_odom >> vx >> vy >> vz;

        timeLaserInfoCur = t_odom;
        timeLaserInoStamp = ros::Time().fromSec(t_odom);

        tf::Quaternion orientation(qx, qy, qz, qw);
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        gtsam::Pose3 pose(
            gtsam::Rot3::RzRyRx(roll, pitch, yaw),
            gtsam::Point3(tx, ty, tz));
        Eigen::Vector3d velocity(vx, vy, vz);

        // PCD
        std::stringstream ss_filename;
        ss_filename << std::setw(6) << std::setfill('0') << processed_count;
        std::string pcd_path = pcd_dir + ss_filename.str() + ".pcd";

        PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB());
        if (pcl::io::loadPCDFile(pcd_path, *cloud) == -1) {
            ROS_ERROR("Missing PCD file for timestamp %.6f: %s", t_odom, pcd_path.c_str());
            continue; 
        }

        saveKeyFrameAndFactor(pose, timeLaserInfoCur, velocity, cloud);
        processed_count++;
    }
    
    odom_file.close();
    cov_file.close();
    ROS_INFO("Load complete. Total frames: %d", processed_count);
}
