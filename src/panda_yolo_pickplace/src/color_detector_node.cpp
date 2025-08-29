#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <image_transport/image_transport.hpp>

// cv_bridge: Jazzy'de .hpp olabilir, yol distroya göre değişebilir
#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge/cv_bridge.hpp>
#else
  #error "cv_bridge.hpp bulunamadı. ros-jazzy-cv-bridge kurulu mu?"
#endif

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class ColorDetectorNode : public rclcpp::Node
{
public:
  ColorDetectorNode()
  : Node("color_detector_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    color_sub_ = image_transport::create_subscription(
      this, "/camera/color/image_raw",
      std::bind(&ColorDetectorNode::colorCb, this, _1), "raw");

    depth_sub_ = image_transport::create_subscription(
      this, "/camera/aligned_depth_to_color/image_raw",
      std::bind(&ColorDetectorNode::depthCb, this, _1), "raw");

    caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", 10,
      std::bind(&ColorDetectorNode::infoCb, this, std::placeholders::_1));

    pub_red_   = this->create_publisher<geometry_msgs::msg::PoseStamped>("/red_block/pose", 10);
    pub_green_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/green_block/pose", 10);
  }

private:
  // Kamera iç parametrelerini sakla
  void infoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // K matrisi: [ fx 0 cx; 0 fy cy; 0 0 1 ]
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    cam_frame_ = msg->header.frame_id; // bu framede yayınlayacağız
    have_info_ = true;
  }

  void depthCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg) { last_depth_ = msg; }

  void colorCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    if(!have_info_ || !last_depth_) return;

    cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    cv::Mat hsv; cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    auto findLargestCenter = [&](const std::vector<cv::Scalar> &lower,
                                 const std::vector<cv::Scalar> &upper)
    {
      cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8U);
      for(size_t i=0;i<lower.size();++i){
        cv::Mat m; cv::inRange(hsv, lower[i], upper[i], m);
        cv::bitwise_or(mask, m, mask);
      }
      cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);
      cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      double best_area = 0.0;
      cv::Point best_c(-1,-1);
      for(const auto &c : contours){
        double area = cv::contourArea(c);
        if(area > best_area){
          auto mu = cv::moments(c);
          if (mu.m00 > 1e-6)
            best_c = cv::Point(static_cast<int>(mu.m10/mu.m00),
                               static_cast<int>(mu.m01/mu.m00));
          best_area = area;
        }
      }
      return std::make_pair(best_c, best_area);
    };

    // Kırmızı (iki aralık) ve yeşil aralıkları
    auto red   = findLargestCenter(
                   {cv::Scalar(0,120,70), cv::Scalar(170,120,70)},
                   {cv::Scalar(10,255,255), cv::Scalar(180,255,255)});
    auto green = findLargestCenter(
                   {cv::Scalar(35,70,70)},
                   {cv::Scalar(85,255,255)});

    if(red.second   > 200.0)  publishPose("red_block",   red.first);
    if(green.second > 200.0)  publishPose("green_block", green.first);
  }

  void publishPose(const std::string &color, const cv::Point &px)
  {
    if(px.x < 0 || px.y < 0) return;

    // Derinlik medyanı (küçük pencere)
    cv::Mat depth = cv_bridge::toCvShare(
        last_depth_, sensor_msgs::image_encodings::TYPE_16UC1)->image; // mm
    const int w = 5;
    std::vector<float> vals; vals.reserve(w*w);
    for(int dy=-w; dy<=w; ++dy){
      for(int dx=-w; dx<=w; ++dx){
        int u = std::clamp(px.x+dx, 0, depth.cols-1);
        int v = std::clamp(px.y+dy, 0, depth.rows-1);
        uint16_t d = depth.at<uint16_t>(v,u);
        if(d>0) vals.push_back(d * 0.001f); // metre
      }
    }
    if(vals.empty()) return;
    std::nth_element(vals.begin(), vals.begin()+vals.size()/2, vals.end());
    const float Z = vals[vals.size()/2];

    // K'dan geri izdüşüm (camera frame'inde)
    const double u = static_cast<double>(px.x);
    const double v = static_cast<double>(px.y);
    const double X = (u - cx_) / fx_ * Z;
    const double Y = (v - cy_) / fy_ * Z;

    geometry_msgs::msg::PoseStamped P;
    P.header.stamp = this->get_clock()->now();
    P.header.frame_id = cam_frame_;
    P.pose.position.x = X;
    P.pose.position.y = Y;
    P.pose.position.z = Z;
    P.pose.orientation.w = 1.0;

    try{
      auto Pw = tf_buffer_.transform(P, "world", tf2::durationFromSec(0.05));
      if(color == "red_block")   pub_red_->publish(Pw);
      else                       pub_green_->publish(Pw);
    }catch(const tf2::TransformException &ex){
      RCLCPP_WARN(this->get_logger(), "TF: %s", ex.what());
    }
  }

  // Subscribers / publishers
  image_transport::Subscriber color_sub_, depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_red_, pub_green_;
  sensor_msgs::msg::Image::ConstSharedPtr last_depth_;

  // Intrinsics
  bool have_info_ = false;
  double fx_=0, fy_=0, cx_=0, cy_=0;
  std::string cam_frame_ = "camera_link";

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
