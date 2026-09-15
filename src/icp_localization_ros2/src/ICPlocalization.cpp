/*
 * ICPlocalization.cpp
 *
 *  Created on: Apr 23, 2021
 *      Author: jelavice
 */
#include "icp_localization_ros2/ICPlocalization.hpp"
#include "icp_localization_ros2/RangeDataAccumulator.hpp"
#include "icp_localization_ros2/common/typedefs.hpp"
#include "icp_localization_ros2/helpers.hpp"
#include "icp_localization_ros2/transform/FrameTracker.hpp"
#include "icp_localization_ros2/transform/ImuTracker.hpp"
#include "icp_localization_ros2/transform/TfPublisher.hpp"
#include "pointmatcher/IO.h"
#include "pointmatcher/PointMatcher.h"
// #include "pointmatcher_ros/deserialization.h"
// #include "pointmatcher_ros/serialization.h"
// #include "pointmatcher_ros/transform.h"
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <memory>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pointmatcher_ros/StampedPointCloud.h>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace icp_loco {

namespace {
const double kRadToDeg = 180.0 / M_PI;
}

ICPlocalization::ICPlocalization(const rclcpp::NodeOptions &options)
    : Node("icp_localization", options) {
  std::vector<double> leafSize =
      this->declare_parameter("leaf_size", std::vector<double>{0.1, 0.1, 0.1});
  if (leafSize.size() != 3) {
    RCLCPP_ERROR(this->get_logger(), "Leaf size must have 3 elements");
    leafSize = {0.1, 0.1, 0.1};
  }
  mapCloudFilter_.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
}

ICPlocalization::~ICPlocalization() {
  icpWorker_.join();
  Rigid3d lastPose(lastPosition_, lastOrientation_);
  std::cout << "ICP_LOCO: Last transform map to range sensor: \n";
  std::cout << lastPose.asString() << "\n";
}

void ICPlocalization::setMapCloud(const Pointcloud::Ptr map) {
  mapCloudFilter_.setInputCloud(map);
  mapCloudFilter_.filter(mapCloud_);
  refCloud_ = fromPCL(mapCloud_);

  isMapSet_ = true;
  icp_.setMap(refCloud_);
  std::cout << "Map size is: " << map->points.size() << "(befor filter), "
            << mapCloud_.points.size() << "(after filter)" << std::endl;
}

void ICPlocalization::setInitialPose(const Eigen::Vector3d &p,
                                     const Eigen::Quaterniond &q) {
  std::cout << "Init pose set to, xyz: " << p.transpose();
  std::cout << ", q: " << q.coeffs().transpose()
            << ", rpy: " << kRadToDeg * icp_loco::toRPY(q).transpose()
            << " deg \n";
  lastPosition_ = p;
  lastOrientation_ = q;
  imuTracker_->setInitialPose(p, q);
  tfPublisher_->setInitialPose(p, q);
}

void ICPlocalization::set2DPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr &msg) {
  try {
    geometry_msgs::msg::Pose pose_received = msg->pose.pose;
    userSetPosition_ =
        Eigen::Vector3d(pose_received.position.x, pose_received.position.y,
                        pose_received.position.z);
    userSetQuaternion_ = Eigen::Quaterniond(
        pose_received.orientation.w, pose_received.orientation.x,
        pose_received.orientation.y, pose_received.orientation.z);
    // tf::pointMsgToEigen(msg->pose.pose.position, userSetPosition_);
    // tf::quaternionMsgToEigen(pose_received.orientation, userSetQuaternion_);
    isSetPoseFromUser_ = true;
  } catch (const std::exception &e) {
    std::cerr << "Caught exception while setting 2D pose: " << e.what() << '\n';
  }
  std::cout << "User set pose to :"
            << Rigid3d(userSetPosition_, userSetQuaternion_).asString()
            << std::endl;
}

// void ICPlocalization::set2DPoseCallback(  
//     const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr &msg) {  
//   try {  
//     geometry_msgs::msg::Pose pose_received = msg->pose.pose;  
      
//     // 获取range_sensor到odometry_source的变换  
//     const auto rangeSensorToOdometrySource =   
//         frameTracker_->getTransformOdomSourceToRangeSensor(  
//             fromRos(msg->header.stamp)).inverse();  
      
//     // 将base_link位姿转换为range_sensor位姿  
//     Rigid3d baseLinkPoseInMap(  
//         Eigen::Vector3d(pose_received.position.x, pose_received.position.y,  
//                         pose_received.position.z),  
//         Eigen::Quaterniond(pose_received.orientation.w, pose_received.orientation.x,  
//                            pose_received.orientation.y, pose_received.orientation.z));  
      
//     // 转换为range_sensor在map中的位姿  
//     Rigid3d rangeSensorPoseInMap = baseLinkPoseInMap * rangeSensorToOdometrySource;  
      
//     userSetPosition_ = rangeSensorPoseInMap.translation();  
//     userSetQuaternion_ = rangeSensorPoseInMap.rotation();  
//     isSetPoseFromUser_ = true;  
//   } catch (const std::exception &e) {  
//     std::cerr << "Caught exception while setting 2D pose: " << e.what() << '\n';  
//   }  
// }

void ICPlocalization::initializeInternal() {

  rangeDataAccumulator_ =
      std::make_shared<RangeDataAccumulatorRos>(this->shared_from_this());
  imuTracker_ = std::make_shared<ImuTracker>();
  frameTracker_ = std::make_shared<FrameTracker>(imuTracker_);
  tfPublisher_ = std::make_shared<TfPublisher>(this->shared_from_this(),
                                               frameTracker_, imuTracker_);

  lastPosition_.setZero();
  lastOrientation_.setIdentity();

  registeredCloudPublisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "registered_cloud", rclcpp::QoS(rclcpp::KeepLast(1)));
  posePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "range_sensor_pose", rclcpp::QoS(rclcpp::KeepLast(1)));
  // nh_.advertise<sensor_msgs::PointCloud2>("registered_cloud", 1, true);
  // posePub_ =
  // nh_.advertise<geometry_msgs::PoseStamped>("range_sensor_pose", 1, true);
  icp_.setDefault();

  // Initialite tf listener
  tfBuffer_.reset(new tf2_ros::Buffer(this->get_clock()));
  tfListener_.reset(new tf2_ros::TransformListener(*tfBuffer_));
  auto callable = [this]() { icpWorker(); };
  icpWorker_ = std::thread(callable);
}

void ICPlocalization::initialize() {

  const std::string configFileIcp =
      this->declare_parameter("icp_config_path", "");
  // std::cout<<"configFileIcp: "<<configFileIcp<<std::endl;
  try {
    std::ifstream in(configFileIcp);
    if (!in.is_open()) {
      throw std::runtime_error("config file icp opening failed");
    }
    icp_.loadFromYaml(in);
    in.close();
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
  }

  const std::string configFileFilters =
      this->declare_parameter("input_filters_config_path", "");
  try {
    namespace fs = std::filesystem;
    fs::path pkgPath = ament_index_cpp::get_package_share_directory("icp_localization_ros2");
    fs::path fullPath = pkgPath / configFileFilters;
    std::ifstream in(fullPath.string());
    if (!in.is_open()) {
      throw std::runtime_error("config file filters opening failed");
    }
    inputFilters_ = PM::DataPointsFilters(in);
    in.close();
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
  }

  std::string rangeDataTopic =
      this->declare_parameter("icp_localization_ros2.range_data_topic", "");

  if (rangeDataTopic.empty()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "failed to load range data topic");
  }
  std::string imuDataTopic =
      this->declare_parameter("icp_localization_ros2.imu_data_topic", "");
  if (imuDataTopic.empty()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "failed to load imu data topic");
  }
  std::string odometryDataTopic =
      this->declare_parameter("icp_localization_ros2.odometry_data_topic", "");
  if (odometryDataTopic.empty()) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "failed to load odometry data topic");
  }

  std::cout << "odometry data topic: " << odometryDataTopic << std::endl;
  std::cout << "imu data topic: " << imuDataTopic << std::endl;

  isUseOdometry_ =
      this->declare_parameter("icp_localization_ros2.is_use_odometry", false);
  std::cout << "Is use odometry: " << std::boolalpha << isUseOdometry_ << "\n";

  tfPublisher_->setOdometryTopic(odometryDataTopic);
  tfPublisher_->setImuTopic(imuDataTopic);
  tfPublisher_->setIsProvideOdomFrame(isUseOdometry_);

  const double gravityVectorFilterTimeConstant = this->declare_parameter(
      "icp_localization_ros2.gravity_vector_filter_time_constant", 0.01);
  std::cout << "Gravity vector filter time constant: "
            << gravityVectorFilterTimeConstant << "\n";
  imuTracker_->setGravityVectorFilterTimeConstant(
      gravityVectorFilterTimeConstant);

  const std::string imuLidarPrefix = "calibration.imu_to_range_sensor.";
  const Rigid3d imuToRangeSensor = Rigid3d(
      getPositionFromParameterServer(this->shared_from_this(), imuLidarPrefix),
      getOrientationFromParameterServer(this->shared_from_this(),
                                        imuLidarPrefix));

  const std::string odometrySourceLidarPrefix =
      "calibration.odometry_source_to_range_sensor.";
  const Rigid3d odometrySourceToRangeSensor =
      Rigid3d(getPositionFromParameterServer(this->shared_from_this(),
                                             odometrySourceLidarPrefix),
              getOrientationFromParameterServer(this->shared_from_this(),
                                                odometrySourceLidarPrefix));
  frameTracker_->setTransformOdometrySourceToRangeSensor(
      odometrySourceToRangeSensor);
  frameTracker_->setTransformImuToRangeSensor(imuToRangeSensor);
  frameTracker_->setIsUseOdometryForRangeSensorPosePrediction(isUseOdometry_);

  const int minNumOdomMeasurements = this->declare_parameter(
      "icp_localization_ros2.min_num_odom_msgs_before_ready", 300);

  frameTracker_->setMinNumOdomMeasurementsBeforeReady(minNumOdomMeasurements);
  std::cout << "Min num odom measurements before ready: "
            << minNumOdomMeasurements << std::endl;

  std::cout << "Calibration: \n";
  std::cout << "imu to range sensor: " << imuToRangeSensor.asString() << "\n\n";
  std::cout << "odometry source to range sensor: "
            << odometrySourceToRangeSensor.asString() << "\n\n";

  RangeDataAccumulatorParamRos rangeDataAccParam;
  rangeDataAccParam.numAccumulatedRangeData_ = this->declare_parameter(
      "icp_localization_ros2.num_accumulated_range_data", 1);
  rangeDataAccParam.inputRangeDataTopic_ =
      this->get_parameter("icp_localization_ros2.range_data_topic").as_string();

  std::cout << "range data parameters: \n";
  std::cout << "topic: " << rangeDataAccParam.inputRangeDataTopic_ << "\n";
  std::cout << "num range data accumulated: "
            << rangeDataAccParam.numAccumulatedRangeData_ << "\n \n";

  fixedFrame_ =
      this->declare_parameter("icp_localization_ros2.fixed_frame", "map");
  std::cout << "Setting fixed frame to: " << fixedFrame_ << std::endl;

  rangeDataAccumulator_->setParam(rangeDataAccParam);
  rangeDataAccumulator_->initialize();
  tfPublisher_->initialize();

  std::cout << "ICPlocalization: Initialized \n";

  // Set 2D pose subscribe
  initialPose_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", rclcpp::QoS(rclcpp::KeepLast(1)),
          std::bind(&ICPlocalization::set2DPoseCallback, this,
                    std::placeholders::_1));
}

DP ICPlocalization::fromPCL(const Pointcloud &pcl) {
  // todo this can result in data loss???
  sensor_msgs::msg::PointCloud2 ros;
  pcl::toROSMsg(pcl, ros);

  return rosMsgToPointMatcherCloud<float>(ros, ros.is_dense);
}

const std::string &ICPlocalization::getFixedFrame() const {
  return fixedFrame_;
}

void ICPlocalization::matchScans() {
  if (!icp_.hasMap()) {
    return;
  }

  Eigen::Vector3d initPosition = lastPosition_;
  Eigen::Quaterniond initOrientation = lastOrientation_;
  Rigid3d before(lastPosition_, lastOrientation_);
  if (!isFirstScanMatch_) {
    const Rigid3d lastPose(initPosition, initOrientation);
    const Rigid3d motionPoseChange =
        frameTracker_->getPoseChangeOfRangeSensorInMapFrame(
            lastOptimizedPoseTimestamp_, regCloudTimestamp_);
    const Rigid3d motionCorrectedPose =
        Rigid3d(initPosition, initOrientation) * motionPoseChange;
    initPosition = motionCorrectedPose.translation();
    initOrientation = motionCorrectedPose.rotation();
    //    std::cout << "Prediction diff: " << motionPoseChange.asString()
    //    << std::endl;
  }

  PM::TransformationParameters initPose;
  // Compute the transformation to express data in ref
  inputFilters_.apply(regCloud_);
  //	optimizedPose_ = initPose;
  if (!isSetPoseFromUser_) {
    try {
      initPose = getTransformationMatrix<float>(toFloat(initPosition),
                                                toFloat(initOrientation));
      optimizedPose_ = icp_(regCloud_, initPose);

      // 简化版本：尝试获取ICP迭代过程中的误差
      // 这个需要查看您的ICP配置，如果保存了误差信息的话
      
      // 或者简单计算变换后点云与参考点云的距离
      // 这里提供一个简单的替代方案
      
      // 计算当前位姿与初始位姿的差异
      PM::TransformationParameters poseDiff = optimizedPose_ * initPose.inverse();
      Eigen::Matrix3f R = poseDiff.topLeftCorner<3, 3>();
      Eigen::Vector3f t = poseDiff.topRightCorner<3, 1>();
      
      // 计算旋转和平移的变化量作为简单指标
      float rotationChange = std::acos((R.trace() - 1.0f) / 2.0f);
      float translationChange = t.norm();
      
      // std::cout << "ICP配准质量指标 - 旋转变化: " << rotationChange 
      //           << " rad, 平移变化: " << translationChange << " m" << std::endl;

      // ============ 新增部分开始 ============
      // 静态变量用于保存数据到文件
      static std::vector<float> rotationChanges;
      static std::vector<float> translationChanges;
      static std::ofstream outFile;
      static bool isFileOpened = false;
      
      // 保存到内存列表
      rotationChanges.push_back(rotationChange);
      translationChanges.push_back(translationChange);
      
      // 首次使用时打开文件
      if (!isFileOpened) {
        // 获取当前时间
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm* now_tm = std::localtime(&now_time);
        
        // 创建文件名：icp_metrics_YYYYMMDD_HHMMSS.txt
        char filename[100];
        std::strftime(filename, sizeof(filename), "icp_metrics_%Y%m%d_%H%M%S.txt", now_tm);
        
        // 创建目录（如果不存在）
        std::string dirPath = "/home/agv/wzb/my_log/icp_log";
        std::string command = "mkdir -p " + dirPath;
        std::system(command.c_str());
        
        // 完整文件路径
        std::string filePath = dirPath + "/" + filename;
        
        outFile.open(filePath);
        isFileOpened = true;
        
        std::cout << "ICP指标将保存到: " << filePath << std::endl;
      }
      
      // 写入到文件
      if (outFile.is_open()) {
        outFile << rotationChange << " " << translationChange << "\n";
        outFile.flush(); // 立即写入，避免程序崩溃时数据丢失
      }
      // ============ 新增部分结束 ============

    } catch (const std::exception &e) {
      std::cerr << "Caught exception while scan matching: " << e.what()
                << std::endl;
      optimizedPose_ = initPose;

      // ============ 异常处理时也记录数据 ============
      static std::ofstream outFile;
      static bool isFileOpened = false;
      
      if (!isFileOpened) {
        // 获取当前时间
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm* now_tm = std::localtime(&now_time);
        
        // 创建文件名：icp_metrics_YYYYMMDD_HHMMSS.txt
        char filename[100];
        std::strftime(filename, sizeof(filename), "icp_metrics_%Y%m%d_%H%M%S.txt", now_tm);
        
        // 创建目录（如果不存在）
        std::string dirPath = "/home/agv/wzb/my_log/icp_log";
        std::string command = "mkdir -p " + dirPath;
        std::system(command.c_str());
        
        // 完整文件路径
        std::string filePath = dirPath + "/" + filename;
        
        outFile.open(filePath);
        isFileOpened = true;
      }
      
      if (outFile.is_open()) {
        outFile << "0.0 0.0\n";
        outFile.flush();
      }
      // ============ 新增部分结束 ============
    }
  } else {
    optimizedPose_ = getTransformationMatrix<float>(
        toFloat(userSetPosition_), toFloat(userSetQuaternion_));
    isSetPoseFromUser_ = false;
  }

  optimizedPoseTimestamp_ = regCloudTimestamp_;
  getPositionAndOrientation<double>(toDouble(optimizedPose_), &lastPosition_,
                                    &lastOrientation_);

  // limit translation if too big
  //  Rigid3d poseDiffIcp = before.inverse()*Rigid3d(lastPosition_,
  //  lastOrientation_); auto &t = poseDiffIcp.translation(); const
  //  Eigen::Vector3d limitdXYZ(0.25,0.25,0.25); t.x() = std::fabs(t.x())
  //  > limitdXYZ.x() ? limitdXYZ.x() : t.x(); t.y() = std::fabs(t.y()) >
  //  limitdXYZ.y() ? limitdXYZ.y() : t.y(); t.z() = std::fabs(t.z()) >
  //  limitdXYZ.z() ? limitdXYZ.z() : t.z(); lastPosition_ = (before *
  //  poseDiffIcp).translation();

  //  std::cout << "Ground truth diff: " << poseDiffIcp.asString() <<
  //  "\n\n";

  frameTracker_->setTransformMapToRangeSensor(TimestampedTransform{
      optimizedPoseTimestamp_, Rigid3d(lastPosition_, lastOrientation_)});
  lastOptimizedPoseTimestamp_ = optimizedPoseTimestamp_;

  isFirstScanMatch_ = false;
}

void ICPlocalization::publishPose() const {
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.pose.position.x = optimizedPose_(0, 3);
  pose_msg.pose.position.y = optimizedPose_(1, 3);
  pose_msg.pose.position.z = optimizedPose_(2, 3);
  Eigen::Isometry3f iso;
  iso.matrix() = optimizedPose_;
  Eigen::Quaternionf q;
  q = iso.rotation();

  q.normalize();
  pose_msg.pose.orientation.w = q.w();
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.header.frame_id = fixedFrame_;
  pose_msg.header.stamp = toRos(optimizedPoseTimestamp_);
  posePub_->publish(pose_msg);

  if (isUseOdometry_) {
    tfPublisher_->publishMapToOdom(optimizedPoseTimestamp_);
  } else {
    tfPublisher_->publishMapToRangeSensor(optimizedPoseTimestamp_);
  }
}

void ICPlocalization::publishRegisteredCloud() const {
  DP data_out(icp_.getReadingFiltered());
  icp_.transformations.apply(data_out, optimizedPose_);
  sensor_msgs::msg::PointCloud2 ros_msg =
      pointMatcherCloudToRosMsg<float>(data_out, fixedFrame_, this->now());
  registeredCloudPublisher_->publish(ros_msg);
}

void ICPlocalization::icpWorker() {
  rclcpp::Rate r(100);
  // ros::Rate r(100);
  while (rclcpp::ok()) {
    if (!rangeDataAccumulator_->isAccumulatedRangeDataReady() ||
        !frameTracker_->isReady()) {
      r.sleep();
      continue;
    }
    regCloudTimestamp_ =
        rangeDataAccumulator_->getAccumulatedRangeDataTimestamp();
    regCloud_ = rangeDataAccumulator_->popAccumulatedRangeData().data_;
    namespace ch = std::chrono;
    const auto startTime = ch::steady_clock::now();
    matchScans();
    const auto endTime = ch::steady_clock::now();
    const unsigned int nUs =
        ch::duration_cast<ch::microseconds>(endTime - startTime).count();
    const double timeMs = nUs / 1000.0;
    //    std::string infoStr = "Scan matching took: " +
    //    std::to_string(timeMs)
    //    + " ms \n"; ROS_INFO_STREAM(infoStr);

    //    ROS_INFO_STREAM_THROTTLE(10.0, "Scan matching took: " << timeMs
    //    << " ms");

    publishPose();
    publishRegisteredCloud();

    r.sleep();
  }
}

} // namespace icp_loco
