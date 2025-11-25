#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using std::placeholders::_1;

class CloudToGridNode : public rclcpp::Node {
public:
  CloudToGridNode() :
      Node("cloud_to_grid_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
  {
    // Parameters
    declare_parameter<std::string>("cloud_topic", "/roi_point_cloud");
    declare_parameter<std::string>("map_frame", "livox_frame");
    declare_parameter<std::string>("base_frame", "base_link");

    declare_parameter<double>("length_x", 60.0);   // 60 meters forward
    declare_parameter<double>("length_y", 10.0);   // 10 meters wide
    declare_parameter<double>("resolution", 0.05); // 5cm resolution

    get_parameter("cloud_topic", cloud_topic_);
    get_parameter("map_frame", map_frame_);
    get_parameter("base_frame", base_frame_);
    get_parameter("length_x", length_x_);
    get_parameter("length_y", length_y_);
    get_parameter("resolution", resolution_);

    // Grid map layers
    map_ = grid_map::GridMap({"elevation", "variance"});
    map_.setFrameId(map_frame_);
    map_.setGeometry(grid_map::Length(length_x_, length_y_), resolution_);

    map_["elevation"].setConstant(NAN);
    map_["variance"].setConstant(0.0);

    // Sub and Pub
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&CloudToGridNode::cloudCb, this, _1));

    pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map_out", 10);

    RCLCPP_INFO(get_logger(),
                "GridMap Node Ready — 60m × 10m — Listening on %s",
                cloud_topic_.c_str());
  }

private:
  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Move grid with vehicle (center = base_link projection)
    try {
      auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      Eigen::Vector2d center(tf.transform.translation.x, tf.transform.translation.y);
      if ((center - map_.getPosition()).norm() > 0.01)
        map_.move(center);
    } catch (...) {}

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // ROI filtering: X and Y only (Z already filtered by ROI node)
    pcl::PassThrough<pcl::PointXYZ> pass;

    // X filter (front distance)
    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, length_x_);
    pcl::PointCloud<pcl::PointXYZ> cloud_x;
    pass.filter(cloud_x);

    // Y filter (left-right width)
    pass.setInputCloud(cloud_x.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-length_y_/2, length_y_/2);
    pcl::PointCloud<pcl::PointXYZ> cloud_xy;
    pass.filter(cloud_xy);

    // Short aliases
    auto &elev = map_["elevation"];
    auto &var  = map_["variance"];

    // --- INSERT POINTS INTO GRID ---------------------------------------
    for (const auto &p : cloud_xy.points) {
      grid_map::Position pos(p.x, p.y);
      grid_map::Index idx;
      if (!map_.getIndex(pos, idx)) continue;

      float &h = elev(idx(0), idx(1));

      // First measurement for this cell
      if (std::isnan(h)) {
        h = p.z;
        var(idx(0), idx(1)) = 0.0;
        continue;
      }

      // Ground = MIN Z (best for bumps/potholes)
      if (p.z < h)
        h = p.z;

      // Simple variance (height confidence)
      var(idx(0), idx(1)) = fabs(p.z - h);
    }

    // --- SPATIAL MEDIAN SMOOTHING (edge-preserving) ----------------------
    grid_map::Matrix elev_smoothed = elev;
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
      grid_map::Index idx(*it);
      std::vector<float> neighbors;

      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          grid_map::Index n(idx(0) + dx, idx(1) + dy);
          if (!map_.isValid(n)) continue;
          float v = elev(n(0), n(1));
          if (!std::isnan(v))
            neighbors.push_back(v);
        }
      }

      if (neighbors.size() > 2) {
        std::nth_element(neighbors.begin(),
                         neighbors.begin() + neighbors.size()/2,
                         neighbors.end());
        elev_smoothed(idx(0), idx(1)) = neighbors[neighbors.size()/2];
      }
    }

    elev = elev_smoothed;

    // Publish final map
    map_.setTimestamp(this->now().nanoseconds());
    auto msg_out = grid_map::GridMapRosConverter::toMessage(map_);
    pub_->publish(std::move(*msg_out));
  }

  std::string cloud_topic_, map_frame_, base_frame_;
  double length_x_, length_y_, resolution_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
  grid_map::GridMap map_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudToGridNode>());
  rclcpp::shutdown();
  return 0;
}

