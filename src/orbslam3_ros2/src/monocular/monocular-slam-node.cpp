#include "monocular-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <cmath>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    
    // Image subscriber
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/image",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    
    // Path publisher - ADD THIS
    m_path_publisher = this->create_publisher<nav_msgs::msg::Path>(
        "/orbslam3/camera_path", 10);
    
    // Initialize path message header - ADD THIS
    m_camera_path.header.frame_id = "map";
    
    std::cout << "MonocularSlamNode initialized with Path publisher" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout << "Processing frame..." << std::endl;
    
    // Track monocular - this returns the camera pose as a Sophus::SE3f (Tcw)
    Sophus::SE3f Tcw_se3 = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    // Validate the returned pose (check translation is finite)
    Eigen::Vector3f tcw = Tcw_se3.translation();
    bool valid_pose = std::isfinite(tcw[0]) && std::isfinite(tcw[1]) && std::isfinite(tcw[2]);

    if (valid_pose)
    {
        // ORB-SLAM3 returns Tcw (Transform from world to camera) as SE3
        // Convert to rotation matrix and translation (Eigen)
        Eigen::Matrix3f Rcw = Tcw_se3.rotationMatrix();
        Eigen::Matrix3f Rwc = Rcw.transpose();
        Eigen::Vector3f twc_eig = -Rwc * tcw;
        
        // Create PoseStamped message
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = msg->header.stamp;
        pose_stamped.header.frame_id = "map";
        
        // Set position from translation vector
        pose_stamped.pose.position.x = static_cast<double>(twc_eig(0));
        pose_stamped.pose.position.y = static_cast<double>(twc_eig(1));
        pose_stamped.pose.position.z = static_cast<double>(twc_eig(2));
        
        // Convert rotation matrix to quaternion
        tf2::Matrix3x3 rotation_matrix(
            static_cast<double>(Rwc(0, 0)), static_cast<double>(Rwc(0, 1)), static_cast<double>(Rwc(0, 2)),
            static_cast<double>(Rwc(1, 0)), static_cast<double>(Rwc(1, 1)), static_cast<double>(Rwc(1, 2)),
            static_cast<double>(Rwc(2, 0)), static_cast<double>(Rwc(2, 1)), static_cast<double>(Rwc(2, 2))
        );
        
        tf2::Quaternion quaternion;
        rotation_matrix.getRotation(quaternion);
        
        pose_stamped.pose.orientation.x = quaternion.x();
        pose_stamped.pose.orientation.y = quaternion.y();
        pose_stamped.pose.orientation.z = quaternion.z();
        pose_stamped.pose.orientation.w = quaternion.w();
        
        // Add pose to path
        m_camera_path.poses.push_back(pose_stamped);
        m_camera_path.header.stamp = msg->header.stamp;
        
        // Optional: Limit path length to prevent memory issues
        const size_t MAX_PATH_SIZE = 10000;
        if (m_camera_path.poses.size() > MAX_PATH_SIZE)
        {
            m_camera_path.poses.erase(m_camera_path.poses.begin());
        }
        
        // Publish the path
        m_path_publisher->publish(m_camera_path);
        
        std::cout << "Published path with " << m_camera_path.poses.size() << " poses" << std::endl;
    }
    else
    {
        std::cout << "Tracking lost or invalid pose" << std::endl;
    }
}