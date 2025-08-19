#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <deque>
#include <numeric>
#include <iostream>
#include <cmath>
// 定义同步器类型别名
using Sync = message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>>;
using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;

class BananaGrasp
{
public:
    // 构造函数：初始化 ROS 节点、MoveIt 规划组、订阅器和发布器
    BananaGrasp() : nh_(), tf_buffer_(), tf_listener_(tf_buffer_), move_group_("panda_arm")
    {
        // 订阅相机内参信息，用于计算香蕉的三维坐标
        camera_info_sub_ = nh_.subscribe("/camera/rgb/camera_info", 1, &BananaGrasp::cameraInfoCallback, this);

        // 订阅深度和 RGB 图像，使用消息同步器确保时间同步
        depth_sub_.subscribe(nh_, "/camera/depth/image_raw", 1);
        rgb_sub_.subscribe(nh_, "/camera/rgb/image_raw", 1);
        sync_ = std::make_shared<Sync>(MySyncPolicy(10), depth_sub_, rgb_sub_);
        sync_->registerCallback(std::bind(&BananaGrasp::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        // 发布处理后的图像，用于调试和可视化
        image_pub_ = nh_.advertise<sensor_msgs::Image>("/banana_grasp_image", 1);

        // 创建定时器，每 0.5 秒检查香蕉重心稳定性并尝试抓取
        timer_ = nh_.createTimer(ros::Duration(0.5), &BananaGrasp::timerCallback, this);

        // 从参数服务器获取机械臂 ID，默认为 "panda"
        nh_.param<std::string>("arm_id", arm_id_, "panda");

        // 初始化 MoveIt 规划组，设置规划时间和尝试次数
        move_group_.setPlanningTime(10.0);
        move_group_.setNumPlanningAttempts(5);
        move_group_.allowReplanning(true);

        // 初始化夹爪服务客户端
        gripper_client_ = new actionlib::SimpleActionClient<franka_gripper::GraspAction>("/franka_gripper/grasp", true);
        while (!gripper_client_->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for gripper server...");
            std::cout << "等待夹爪服务启动..." << std::endl;
        }
    }

    // 相机信息回调函数：获取相机内参和相机在世界坐标系中的位姿
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                "world", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
            camera_pose_.x = transform.transform.translation.x;
            camera_pose_.y = transform.transform.translation.y;
            camera_pose_.z = transform.transform.translation.z;
            fx_ = msg->K[0];
            fy_ = msg->K[4];
            cx_ = msg->K[2];
            cy_ = msg->K[5];
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Camera transform failed: %s", ex.what());
            std::cout << "警告：相机坐标变换失败: " << ex.what() << std::endl;
        }
    }

    // 计算两点间的曼哈顿距离，用于判断香蕉重心是否稳定
    double manhattanDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
    {
        return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y) + std::abs(p1.z - p2.z);
    }

    // 图像同步回调函数：处理深度和 RGB 图像，检测香蕉并计算其重心
    void syncCallback(const sensor_msgs::ImageConstPtr &depth_msg,
                      const sensor_msgs::ImageConstPtr &rgb_msg)
    {
        try
        {
            cv::Mat depth = cv_bridge::toCvShare(depth_msg, "32FC1")->image;
            cv::Mat rgb = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;

            if (depth.empty() || rgb.empty())
            {
                ROS_ERROR("Empty image received");
                std::cout << "错误：接收到空图像" << std::endl;
                return;
            }

            cv::Mat valid_mask = depth == depth;
            depth.setTo(0, ~valid_mask);
            double min_val, max_val;
            cv::minMaxLoc(depth, &min_val, &max_val, nullptr, nullptr, valid_mask);

            if (max_val <= min_val)
            {
                ROS_WARN("Invalid depth range: min=%.4f, max=%.4f", min_val, max_val);
                std::cout << "警告：深度范围无效: min=" << min_val << ", max=" << max_val << std::endl;
                depth_mean_ = 0.5;
            }
            else
            {
                depth_mean_ = (max_val + min_val) / 2.0;
            }

            cv::Mat hsv;
            cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

            cv::Scalar lower_yellow(20, 100, 100);
            cv::Scalar upper_yellow(40, 255, 255);
            cv::Mat mask;
            cv::inRange(hsv, lower_yellow, upper_yellow, mask);

            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

            cv::Mat edges;
            cv::Canny(mask, edges, 50, 150);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (!contours.empty())
            {
                auto max_contour = *std::max_element(contours.begin(), contours.end(),
                                                     [](const auto &a, const auto &b)
                                                     { return cv::contourArea(a) < cv::contourArea(b); });

                double area = cv::contourArea(max_contour);
                if (area < 100)
                {
                    ROS_WARN("Contour too small");
                    std::cout << "警告：轮廓太小，跳过" << std::endl;
                    return;
                }

                contour_ = max_contour;

                cv::Moments m = cv::moments(max_contour);
                if (m.m00 != 0)
                {
                    center_ = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
                }
                else
                {
                    ROS_WARN("Invalid contour moments");
                    std::cout << "警告：轮廓矩无效" << std::endl;
                    return;
                }

                cv::RotatedRect rect = cv::minAreaRect(max_contour);
                float width = rect.size.width;
                float height = rect.size.height;
                float angle = rect.angle;
                cv::Point2f axis;

                if (width <= height)
                {
                    axis = cv::Point2f(width / 2, 0);
                }
                else
                {
                    axis = cv::Point2f(0, height / 2);
                }

                float rad = angle * CV_PI / 180.0;
                dir_ = cv::Point2f(axis.x * cos(rad) - axis.y * sin(rad),
                                   axis.x * sin(rad) + axis.y * cos(rad));

                double scale_factor = 0.8;
                nh_.param("/banana_grasp/scale_factor", scale_factor, 0.8);
                dir_ = dir_ * static_cast<float>(scale_factor);

                cv::Point2f start = center_ - dir_;
                cv::Point2f end = center_ + dir_;
                if (start.x < 0 || start.x >= rgb.cols || start.y < 0 || start.y >= rgb.rows ||
                    end.x < 0 || end.x >= rgb.cols || end.y < 0 || end.y >= rgb.rows)
                {
                    ROS_WARN("green line out of camera frame");
                    std::cout << "警告：抓取线超出图像边界" << std::endl;
                    return;
                }

                try
                {
                    geometry_msgs::TransformStamped cam_transform = tf_buffer_.lookupTransform(
                        "world", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
                    double xdiff = (center_.x - cx_) * depth_mean_ / fx_;
                    double ydiff = (center_.y - cy_) * depth_mean_ / fy_;
                    double zdiff = depth_mean_;
                    double xc = cam_transform.transform.translation.x;
                    double yc = cam_transform.transform.translation.y;
                    double zc = cam_transform.transform.translation.z;
                    banana_centroid_.x = xc + xdiff;
                    banana_centroid_.y = yc - ydiff;
                    banana_centroid_.z = zc - zdiff + 0.03;
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN("Centroid transform failed: %s", ex.what());
                    std::cout << "警告：重心坐标变换失败: " << ex.what() << std::endl;
                    return;
                }

                centroid_buffer_.push_back(banana_centroid_);
                if (centroid_buffer_.size() > 5)
                {
                    centroid_buffer_.pop_front();
                }

                cv::drawContours(rgb, std::vector<std::vector<cv::Point>>{max_contour}, -1, cv::Scalar(0, 0, 255), 2);
                cv::line(rgb, center_ - dir_, center_ + dir_, cv::Scalar(0, 255, 0), 2);
            }
            else
            {
                ROS_WARN("No contours found");
                std::cout << "警告：未找到轮廓" << std::endl;
                centroid_buffer_.clear();
            }

            sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(rgb_msg->header, "bgr8", rgb).toImageMsg();
            image_pub_.publish(out_msg);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge error: %s", e.what());
            std::cout << "错误：cv_bridge 错误: " << e.what() << std::endl;
        }
        catch (cv::Exception &e)
        {
            ROS_ERROR("OpenCV error: %s", e.what());
            std::cout << "错误：OpenCV 错误: " << e.what() << std::endl;
        }
    } 
    // 定时器回调函数：定期检查香蕉重心稳定性并触发抓取流程
    void timerCallback(const ros::TimerEvent &)
    {
        try
        {
            geometry_msgs::TransformStamped left_finger = tf_buffer_.lookupTransform(
                "world", arm_id_ + "_leftfinger", ros::Time(0), ros::Duration(1.0));
            geometry_msgs::TransformStamped right_finger = tf_buffer_.lookupTransform(
                "world", arm_id_ + "_rightfinger", ros::Time(0), ros::Duration(1.0));

            geometry_msgs::Point gripper_center;
            gripper_center.x = (left_finger.transform.translation.x + right_finger.transform.translation.x) / 2.0;
            gripper_center.y = (left_finger.transform.translation.y + right_finger.transform.translation.y) / 2.0;
            gripper_center.z = (left_finger.transform.translation.z + right_finger.transform.translation.z) / 2.0;

            std::cout << "相机坐标: (" << camera_pose_.x << ", " << camera_pose_.y << ", " << camera_pose_.z << ")" << std::endl;
            std::cout << "夹爪中心: (" << gripper_center.x << ", " << gripper_center.y << ", " << gripper_center.z << ")" << std::endl;
            std::cout << "香蕉重心: (" << banana_centroid_.x << ", " << banana_centroid_.y << ", " << banana_centroid_.z << ")" << std::endl;

            if (centroid_buffer_.size() == 5)
            {
                double max_dist = 0.0;
                for (size_t i = 1; i < centroid_buffer_.size(); ++i)
                {
                    double dist = manhattanDistance(centroid_buffer_[i], centroid_buffer_[i - 1]);
                    max_dist = std::max(max_dist, dist);
                }
                if (max_dist < 0.01)
                {
                    ROS_INFO("Centroid stable, starting grasp");
                    std::cout << "香蕉重心稳定，开始抓取流程" << std::endl;
                    executeGrasp();
                }
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Gripper transform failed: %s", ex.what());
            std::cout << "警告：夹爪坐标变换失败: " << ex.what() << std::endl;
        }
    }
    void executeGrasp()
    {
        // 1. 初始化 action 客户端
        actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client("/franka_gripper/move", true);
        actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client("/franka_gripper/grasp", true);
        ros::Rate rate(10); // 10 Hz 控制循环

        // 等待服务器连接
        ROS_INFO("Waiting for gripper action servers...");
        std::cout << "等待夹爪动作服务器启动..." << std::endl;
        if (!move_client.waitForServer(ros::Duration(10.0)))
        {
            ROS_ERROR("Move action server not available");
            std::cout << "错误：移动动作服务器不可用" << std::endl;
            return;
        }
        if (!grasp_client.waitForServer(ros::Duration(10.0)))
        {
            ROS_ERROR("Grasp action server not available");
            std::cout << "错误：抓取动作服务器不可用" << std::endl;
            return;
        }
        ROS_INFO("Gripper action servers connected");
        std::cout << "夹爪动作服务器已连接" << std::endl;

        // 2. 忽略碰撞（调整力矩/力阈值）
        // 使用 MoveIt! 或 franka_control 设置较低的碰撞阈值
        move_group_.setMaxAccelerationScalingFactor(1); // 降低加速度，减少碰撞敏感性
        move_group_.setMaxVelocityScalingFactor(1);     // 降低速度
        // 注意：需要通过 franka_control 接口设置具体力矩阈值，这里简化处理

        // 3. 打开夹爪到最大
        franka_gripper::MoveGoal open_goal;
        open_goal.width = 0.08; // 最大宽度（根据硬件限制调整）
        open_goal.speed = 1;    // 张开速度
        move_client.sendGoal(open_goal);
        move_client.waitForResult(ros::Duration(5.0)); // 等待 5 秒
        ROS_INFO("Gripper open command sent, proceeding regardless of result");
        std::cout << "夹爪张开命令已发送，无论结果如何继续" << std::endl;

        // 4. 移动机械臂到香蕉重心上方 0.3m
        geometry_msgs::Pose target_pose;
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        try
        {
            geometry_msgs::TransformStamped left_finger = tf_buffer_.lookupTransform(
                "world", arm_id_ + "_leftfinger", ros::Time(0), ros::Duration(1.0));
            geometry_msgs::TransformStamped right_finger = tf_buffer_.lookupTransform(
                "world", arm_id_ + "_rightfinger", ros::Time(0), ros::Duration(1.0));
            geometry_msgs::Point gripper_center;
            gripper_center.x = (left_finger.transform.translation.x + right_finger.transform.translation.x) / 2.0;
            gripper_center.y = (left_finger.transform.translation.y + right_finger.transform.translation.y) / 2.0;
            gripper_center.z = (left_finger.transform.translation.z + right_finger.transform.translation.z) / 2.0;

            geometry_msgs::Point delta;
            delta.x = banana_centroid_.x - gripper_center.x;
            delta.y = banana_centroid_.y - gripper_center.y;
            delta.z = banana_centroid_.z - gripper_center.z;

            move_group_.setPlannerId("PRMstarkConfigDefault");
            move_group_.setPlanningTime(15.0);
            move_group_.setStartStateToCurrentState();
            // 计算抓取线的 yaw 角度（基于 dir_）
            double yaw = std::atan2(dir_.y, dir_.x); // 图像平面中的角度
            tf2::Quaternion q;
            q.setRPY(3.14159, 0, 3.14159/4-yaw); // 保持 roll=π, pitch=0，调整 yaw
            geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q);
            geometry_msgs::Pose current_pose = move_group_.getCurrentPose().pose;
            target_pose = current_pose;
            target_pose.position.x += delta.x;
            target_pose.position.y += delta.y;
            target_pose.position.z += delta.z + 0.1;
            target_pose.orientation = grasp_orientation; // 应用计算得到的朝向
            move_group_.setPoseTarget(target_pose);

            moveit::planning_interface::MoveItErrorCode plan_result = move_group_.plan(arm_plan);
            if (plan_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                moveit::core::MoveItErrorCode move_result = move_group_.execute(arm_plan);
                if (move_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO("Arm moved to centroid successfully");
                    std::cout << "夹爪中心已对齐香蕉重心上方 0.3m" << std::endl;
                }
                else
                {
                    ROS_ERROR("Arm move failed, error code: %d", move_result.val);
                    std::cout << "错误：机械臂移动失败，错误码: " << move_result.val << std::endl;
                    return;
                }
            }
            else
            {
                ROS_ERROR("Arm planning failed, error code: %d", plan_result.val);
                std::cout << "错误：机械臂规划失败，错误码: " << plan_result.val << std::endl;
                return;
            }

            // 5. 夹爪下降 0.3m 到香蕉重心位置
            target_pose.position.z -= 0.08;
            move_group_.setPoseTarget(target_pose);
            plan_result = move_group_.plan(arm_plan);
            if (plan_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                moveit::core::MoveItErrorCode move_result = move_group_.execute(arm_plan);
                if (move_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO("Arm descended 0.3m to centroid successfully");
                    std::cout << "夹爪下降 0.3m 到香蕉重心" << std::endl;
                }
                else
                {
                    ROS_ERROR("Arm descend failed, error code: %d", move_result.val);
                    std::cout << "错误：夹爪下降失败，错误码: " << move_result.val << std::endl;
                    return;
                }
            }
            else
            {
                ROS_ERROR("Arm descend planning failed, error code: %d", plan_result.val);
                std::cout << "错误：夹爪下降规划失败，错误码: " << plan_result.val << std::endl;
                return;
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Gripper transform failed: %s", ex.what());
            std::cout << "警告：夹爪坐标变换失败: " << ex.what() << std::endl;
            return;
        }

        // 6. 无条件闭合夹爪（抓取）
        franka_gripper::GraspGoal grasp_goal;
        grasp_goal.width = 0.02;         // 目标宽度（0.0 表示完全闭合）
        grasp_goal.epsilon.inner = 0.01; // 内部容差，允许一定误差
        grasp_goal.epsilon.outer = 0.01; // 外部容差，适应对象大小
        grasp_goal.speed = 1;            // 闭合速度
        grasp_goal.force = 20.0;        // 施加的力（根据硬件限制调整）
        grasp_client.sendGoal(grasp_goal); 
        ROS_INFO("Gripper grasp command sent, proceeding regardless of result");
        std::cout << "夹爪抓取命令已发送，无论结果如何继续" << std::endl;

        // 7. 机械臂向上移动 0.5m
        target_pose.position.z += 0.5;
        move_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveItErrorCode plan_result = move_group_.plan(arm_plan);
        if (plan_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            moveit::core::MoveItErrorCode move_result = move_group_.execute(arm_plan);
            if (move_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("Arm moved up 0.5m successfully");
                std::cout << "机械臂向上移动0.5m" << std::endl;
            }
            else
            {
                ROS_ERROR("Arm move up failed, error code: %d", move_result.val);
                std::cout << "错误：机械臂移动失败，错误码: " << move_result.val << std::endl;
                return;
            }
        }
        else
        {
            ROS_ERROR("Arm planning failed, error code: %d", plan_result.val);
            std::cout << "错误：机械臂规划失败，错误码: " << plan_result.val << std::endl;
            return;
        }
        franka_gripper::MoveGoal release_goal;
        release_goal.width = 0.08; // 最大宽度（根据硬件限制调整）
        release_goal.speed = 1;    // 张开速度
        move_client.sendGoal(release_goal);
  
        // 8. 抓取完成，退出程序
        ROS_INFO("Grasp completed, exiting");
        std::cout << "抓取完成，程序退出" << std::endl;
        ros::shutdown();
    }

private:
    ros::NodeHandle nh_; // ROS 节点句柄
    // 图像订阅器和同步器
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_, rgb_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher image_pub_;                                                   // 图像发布器
    ros::Timer timer_;                                                           // 定时器
    std::shared_ptr<Sync> sync_;                                                 // 消息同步器
    std::vector<cv::Point> contour_;                                             // 香蕉轮廓
    cv::Point2f center_, dir_;                                                   // 图像中的质心和抓取方向
    geometry_msgs::Point camera_pose_, banana_centroid_;                         // 相机和香蕉重心坐标
    double depth_mean_ = 0.0;                                                    // 平均深度
    std::string arm_id_;                                                         // 机械臂 ID
    tf2_ros::Buffer tf_buffer_;                                                  // TF 变换缓冲区
    tf2_ros::TransformListener tf_listener_;                                     // TF 监听器
    moveit::planning_interface::MoveGroupInterface move_group_;                  // 机械臂 MoveIt 规划组
    std::deque<geometry_msgs::Point> centroid_buffer_;                           // 重心坐标缓存队列
    double fx_, fy_, cx_, cy_;                                                   // 相机内参
    actionlib::SimpleActionClient<franka_gripper::GraspAction> *gripper_client_; // 夹爪服务客户端
};

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "banana_grasp");
    // 使用异步 spinner，支持多线程处理回调
    ros::AsyncSpinner spinner(2);
    spinner.start();
    // 创建 BananaGrasp 实例
    BananaGrasp bg;
    // 进入主循环，等待回调
    ros::waitForShutdown();
    return 0;
}