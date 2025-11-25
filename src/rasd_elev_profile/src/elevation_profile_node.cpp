#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <string>

using std::placeholders::_1;

// Simple clamp for float
inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

class GroundPlaneSpeedbumpNode : public rclcpp::Node
{
public:
    GroundPlaneSpeedbumpNode()
        : Node("ground_plane_speedbump_node")
    {
        // ---------------------------------------------------------
        //                      PARAMETERS
        // ---------------------------------------------------------
        declare_parameter<std::string>("cloud_topic", "/roi_point_cloud");
        declare_parameter<std::string>("frame_id", "base_link");

        // ROI & binning along X
        declare_parameter<double>("x_min", 0.0);
        declare_parameter<double>("x_max", 60.0);     // <== up to 60 m
        declare_parameter<double>("bin_size", 0.20);
        declare_parameter<double>("lane_width", 4.0); // ±2 m

        // Smoothing window (bins) around each bin for z_smooth
        declare_parameter<int>("smooth_window", 2);   // 2 → 5-bin moving avg

        // Speed bump vertical size [meters]
        declare_parameter<double>("bump_min_height", 0.05); // 5 cm
        declare_parameter<double>("bump_max_height", 0.25); // 25 cm

        // Speed bump physical width constraints [meters]
        declare_parameter<double>("bump_min_width", 0.20);
        declare_parameter<double>("bump_max_width", 2.50);

        // Detection X range (we only care in this window)
        declare_parameter<double>("detect_min_x", 1.5);
        declare_parameter<double>("detect_max_x", 60.0);

        // Min points per bin to trust profile
        declare_parameter<int>("min_points_per_bin", 3);

        // Ground plane fitting window
        declare_parameter<double>("plane_fit_x_min", 1.0);
        declare_parameter<double>("plane_fit_x_max", 5.0);
        declare_parameter<int>("plane_fit_min_points", 300);
        declare_parameter<double>("plane_refine_cutoff_m", 0.03); // 3 cm residual cutoff

        // Load parameters
        get_parameter("cloud_topic", cloud_topic_);
        get_parameter("frame_id", frame_id_);
        get_parameter("x_min", x_min_);
        get_parameter("x_max", x_max_);
        get_parameter("bin_size", bin_size_);
        get_parameter("lane_width", lane_width_);

        get_parameter("smooth_window", smooth_window_);
        get_parameter("bump_min_height", bump_min_height_);
        get_parameter("bump_max_height", bump_max_height_);
        get_parameter("bump_min_width", bump_min_width_);
        get_parameter("bump_max_width", bump_max_width_);

        get_parameter("detect_min_x", detect_min_x_);
        get_parameter("detect_max_x", detect_max_x_);

        get_parameter("min_points_per_bin", min_points_per_bin_);

        get_parameter("plane_fit_x_min", plane_fit_x_min_);
        get_parameter("plane_fit_x_max", plane_fit_x_max_);
        get_parameter("plane_fit_min_points", plane_fit_min_points_);
        get_parameter("plane_refine_cutoff_m", plane_refine_cutoff_m_);

        num_bins_ = static_cast<int>((x_max_ - x_min_) / bin_size_);

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&GroundPlaneSpeedbumpNode::cloudCallback, this, _1));

        vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/rasd/speedbump_groundplane_vis", 10);

        RCLCPP_INFO(get_logger(),
                    "GroundPlane Speedbump Node READY. topic=%s, x=[%.1f..%.1f], bin=%.2f, bins=%d",
                    cloud_topic_.c_str(), x_min_, x_max_, bin_size_, num_bins_);
    }

private:
    struct Bump
    {
        int start_bin;
        int end_bin;
        float x_front;
        float x_center;
        float width;
        float height;      // max dz above ground
        float confidence;  // 0..1
        float height_score;
        float density_score;
        float range_score;
    };

    // ---------------------------------------------------------
    //         ROBUST LEAST-SQUARES GROUND PLANE FIT
    //         z ≈ a*x + b*y + c   (no rotation needed)
    // ---------------------------------------------------------
    bool robustFitGroundPlane(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                              double& a, double& b, double& c)
    {
        const float half_w = static_cast<float>(lane_width_ * 0.5f);

        std::vector<Eigen::Vector3d> pts;
        pts.reserve(cloud.points.size());

        // Collect candidate ground points
        for (const auto& p : cloud.points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;

            if (p.x < plane_fit_x_min_ || p.x > plane_fit_x_max_)
                continue;

            if (std::fabs(p.y) > half_w)
                continue;

            // Optional: light Z filter to avoid crazy outliers
            if (p.z < -1.0f || p.z > 0.2f)
                continue;

            pts.emplace_back(p.x, p.y, p.z);
        }

        int N = static_cast<int>(pts.size());
        if (N < plane_fit_min_points_)
        {
            RCLCPP_WARN(get_logger(),
                        "[PLANE] Not enough points for ground fit: %d < %d",
                        N, plane_fit_min_points_);
            return false;
        }

        auto fitPlaneLS = [&](const std::vector<Eigen::Vector3d>& subset,
                              double& oa, double& ob, double& oc) -> bool
        {
            int M = static_cast<int>(subset.size());
            if (M < 3) return false;

            Eigen::MatrixXd A(M, 3);
            Eigen::VectorXd Z(M);

            for (int i = 0; i < M; ++i)
            {
                const auto& v = subset[i];
                A(i, 0) = v.x();
                A(i, 1) = v.y();
                A(i, 2) = 1.0;
                Z(i)    = v.z();
            }

            Eigen::Vector3d coeff = A.colPivHouseholderQr().solve(Z);
            oa = coeff(0);
            ob = coeff(1);
            oc = coeff(2);
            return true;
        };

        // ---- First LS fit ----
        double a0 = 0.0, b0 = 0.0, c0 = 0.0;
        if (!fitPlaneLS(pts, a0, b0, c0))
        {
            RCLCPP_WARN(get_logger(), "[PLANE] Initial LS fit failed.");
            return false;
        }

        // ---- Compute residuals ----
        std::vector<double> residuals;
        residuals.reserve(N);
        for (const auto& v : pts)
        {
            double zx = a0 * v.x() + b0 * v.y() + c0;
            residuals.push_back(v.z() - zx);
        }

        // Median absolute residual
        std::vector<double> abs_res = residuals;
        for (auto& r : abs_res) r = std::fabs(r);
        size_t mid = abs_res.size() / 2;
        std::nth_element(abs_res.begin(), abs_res.begin() + mid, abs_res.end());
        double mad = abs_res[mid]; // median abs deviation (not scaled)

        double cutoff = std::max(plane_refine_cutoff_m_, 2.0 * mad);
        if (!std::isfinite(cutoff) || cutoff < 0.01)
            cutoff = plane_refine_cutoff_m_;

        // ---- Refine: keep only points with |residual| <= cutoff ----
        std::vector<Eigen::Vector3d> inliers;
        inliers.reserve(N);
        for (int i = 0; i < N; ++i)
        {
            if (std::fabs(residuals[i]) <= cutoff)
                inliers.push_back(pts[i]);
        }

        if (static_cast<int>(inliers.size()) < plane_fit_min_points_ / 2)
        {
            // If refinement fails, fall back to first plane
            a = a0;
            b = b0;
            c = c0;
            RCLCPP_WARN(get_logger(),
                        "[PLANE] Refinement kept only %zu pts, using initial plane.",
                        inliers.size());
        }
        else
        {
            if (!fitPlaneLS(inliers, a, b, c))
            {
                a = a0; b = b0; c = c0;
                RCLCPP_WARN(get_logger(),
                            "[PLANE] Refinement LS failed, using initial plane.");
            }
        }

        RCLCPP_INFO(get_logger(),
                    "[PLANE] z ≈ a*x + b*y + c  | a=%.6f, b=%.6f, c=%.4f | N=%d, inliers≈%zu, cutoff=%.3f m",
                    a, b, c, N, inliers.size(), cutoff);

        return true;
    }

    // ---------------------------------------------------------
    //                  MAIN CLOUD CALLBACK
    // ---------------------------------------------------------
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        if (cloud.empty())
            return;

        // 1) Fit ground plane
        double a = 0.0, b = 0.0, c = 0.0;
        if (!robustFitGroundPlane(cloud, a, b, c))
        {
            RCLCPP_WARN(get_logger(), "[PLANE] Skipping frame: no valid plane.");
            return;
        }

        const float half_w = static_cast<float>(lane_width_ * 0.5f);

        // 2) BIN ACCUMULATION (using height above plane)
        std::vector<std::vector<float>> z_bins(num_bins_);
        std::vector<int> counts(num_bins_, 0);

        for (const auto& p : cloud.points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;

            if (p.x < x_min_ || p.x > x_max_)
                continue;

            if (std::fabs(p.y) > 0.5f)
                continue;

            int bin = static_cast<int>((p.x - x_min_) / bin_size_);
            if (bin < 0 || bin >= num_bins_)
                continue;

            // Height above fitted plane (approx vertical, slopes are small)
            double z_plane = a * p.x + b * p.y + c;
            float dz = static_cast<float>(p.z - z_plane);

            z_bins[bin].push_back(dz);
            counts[bin]++;
        }

        // 3) MEDIAN HEIGHT PROFILE
        std::vector<float> z_med(num_bins_, std::numeric_limits<float>::quiet_NaN());
        for (int i = 0; i < num_bins_; ++i)
        {
            auto& v = z_bins[i];
            if (static_cast<int>(v.size()) < min_points_per_bin_)
                continue;

            size_t mid = v.size() / 2;
            std::nth_element(v.begin(), v.begin() + mid, v.end());
            z_med[i] = v[mid];
        }

        // 4) SMOOTHED PROFILE (moving average on medians)
        std::vector<float> z_smooth = z_med;
        for (int i = 0; i < num_bins_; ++i)
        {
            int W = smooth_window_;
            float sum = 0.0f;
            int c = 0;
            for (int k = -W; k <= W; ++k)
            {
                int idx = i + k;
                if (idx < 0 || idx >= num_bins_) continue;
                if (!std::isfinite(z_med[idx])) continue;
                sum += z_med[idx];
                c++;
            }
            if (c >= 3)
                z_smooth[i] = sum / static_cast<float>(c);
        }

        // Debug: print every 20th bin
        for (int i = 0; i < num_bins_; i += 20)
        {
            float x = static_cast<float>(x_min_ + (i + 0.5f) * bin_size_);
            RCLCPP_INFO(get_logger(),
                        "[DEBUG] BIN %d | x=%.2f | count=%d | dz_med=%.3f | dz_smooth=%.3f",
                        i, x, counts[i], z_med[i], z_smooth[i]);
        }

        // 5) SIMPLE BUMP DETECTION ON z_smooth
        std::vector<Bump> bumps;

        int i = 0;
        while (i < num_bins_)
        {
            if (!std::isfinite(z_smooth[i]) ||
                z_smooth[i] < static_cast<float>(bump_min_height_))
            {
                ++i;
                continue;
            }

            // Start of a potential bump
            int start = i;
            int end   = i;
            float max_dz = z_smooth[i];
            int sum_count = counts[i];

            ++i;
            while (i < num_bins_ &&
                   std::isfinite(z_smooth[i]) &&
                   z_smooth[i] >= static_cast<float>(bump_min_height_))
            {
                end = i;
                if (z_smooth[i] > max_dz) max_dz = z_smooth[i];
                sum_count += counts[i];
                ++i;
            }

            float width = static_cast<float>((end - start + 1) * bin_size_);
            int center_bin = (start + end) / 2;
            float x_front  = static_cast<float>(x_min_ + (start + 0.5f) * bin_size_);
            float x_center = static_cast<float>(x_min_ + (center_bin + 0.5f) * bin_size_);

            // Range filtering
            if (x_center < static_cast<float>(detect_min_x_) ||
                x_center > static_cast<float>(detect_max_x_))
                continue;

            // Width constraints
            if (width < static_cast<float>(bump_min_width_) ||
                width > static_cast<float>(bump_max_width_))
                continue;

            // Height constraint
            if (max_dz > static_cast<float>(bump_max_height_))
                continue;

            int len = end - start + 1;
            float avg_count = static_cast<float>(sum_count) / static_cast<float>(len);

            // ----- Scores -----
            float height_score = 0.0f;
            if (bump_max_height_ > bump_min_height_)
            {
                height_score = clampf(
                    (max_dz - static_cast<float>(bump_min_height_)) /
                    static_cast<float>(bump_max_height_ - bump_min_height_),
                    0.0f, 1.0f);
            }

            // density_score: (3..15 pts/bin) → (0..1)
            float density_score = clampf((avg_count - 3.0f) / 12.0f, 0.0f, 1.0f);

            // range_score: 1 near detect_min, 0 near detect_max
            float range_score = 1.0f;
            {
                float denom = static_cast<float>(detect_max_x_ - detect_min_x_);
                if (denom > 0.0f)
                {
                    float t = (x_center - static_cast<float>(detect_min_x_)) / denom;
                    range_score = 1.0f - clampf(t, 0.0f, 1.0f);
                }
            }

            float confidence =
                0.5f * height_score +
                0.3f * density_score +
                0.2f * range_score;

            Bump b;
            b.start_bin = start;
            b.end_bin = end;
            b.x_front = x_front;
            b.x_center = x_center;
            b.width = width;
            b.height = max_dz;
            b.confidence = confidence;
            b.height_score = height_score;
            b.density_score = density_score;
            b.range_score = range_score;

            bumps.push_back(b);

            // High-level bump debug
            RCLCPP_WARN(get_logger(),
                        "[BUMP] BINS %d..%d | x_front=%.2f | x_center=%.2f | "
                        "width=%.2f | height=%.3f | avgCnt=%.1f | "
                        "hScore=%.2f | dens=%.2f | range=%.2f | CONF=%.2f",
                        start, end,
                        b.x_front, b.x_center,
                        b.width, b.height,
                        avg_count,
                        b.height_score,
                        b.density_score,
                        b.range_score,
                        b.confidence);

            // Per-bin inside bump
            for (int j = start; j <= end; ++j)
            {
                float x = static_cast<float>(x_min_ + (j + 0.5f) * bin_size_);
                RCLCPP_INFO(get_logger(),
                            "[BUMP] BIN %d | x=%.2f | count=%d | dz_med=%.3f | dz_smooth=%.3f | conf=%.2f",
                            j, x, counts[j], z_med[j], z_smooth[j], b.confidence);
            }
        }

        // 6) VISUALIZATION
        publishVisualization(z_smooth, bumps);
    }

    // ---------------------------------------------------------
    //          RVIZ VISUALIZATION (PROFILE + BUMPS)
    // ---------------------------------------------------------
    void publishVisualization(const std::vector<float>& z_smooth,
                              const std::vector<Bump>& bumps)
    {
        visualization_msgs::msg::MarkerArray arr;

        // Delete old markers
        {
            visualization_msgs::msg::Marker del;
            del.header.frame_id = frame_id_;
            del.header.stamp = now();
            del.ns = "profile";
            del.id = 0;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            arr.markers.push_back(del);

            visualization_msgs::msg::Marker del2 = del;
            del2.ns = "baseline";
            arr.markers.push_back(del2);

            visualization_msgs::msg::Marker del3 = del;
            del3.ns = "bumps";
            arr.markers.push_back(del3);
        }

        // ----- Baseline at z=0 -----
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = now();
            m.ns = "baseline";
            m.id = 1;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.03;
            m.color.r = 0.2f;
            m.color.g = 0.2f;
            m.color.b = 0.8f;
            m.color.a = 0.9f;

            for (int i = 0; i < num_bins_; ++i)
            {
                float x = static_cast<float>(x_min_ + (i + 0.5f) * bin_size_);
                geometry_msgs::msg::Point p;
                p.x = x;
                p.y = 0.0;
                p.z = 0.0;
                m.points.push_back(p);
            }

            arr.markers.push_back(m);
        }

        // ----- Height profile (z_smooth) -----
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = now();
            m.ns = "profile";
            m.id = 2;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.03;
            m.color.r = 0.0f;
            m.color.g = 0.9f;
            m.color.b = 0.0f;
            m.color.a = 0.9f;

            for (int i = 0; i < num_bins_; ++i)
            {
                if (!std::isfinite(z_smooth[i])) continue;

                float x = static_cast<float>(x_min_ + (i + 0.5f) * bin_size_);
                geometry_msgs::msg::Point p;
                p.x = x;
                p.y = 0.0;
                p.z = z_smooth[i];
                m.points.push_back(p);
            }

            arr.markers.push_back(m);
        }

        // ----- Bump markers -----
        int bump_id = 10;
        for (const auto& b : bumps)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = now();
            m.ns = "bumps";
            m.id = bump_id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;

            m.pose.position.x = b.x_center;
            m.pose.position.y = 0.0;
            m.pose.position.z = b.height;

            m.scale.x = b.width;
            m.scale.y = 0.5f * b.width;
            m.scale.z = 0.2f;

            float c = clampf(b.confidence, 0.0f, 1.0f);
            m.color.r = c;
            m.color.g = 1.0f - 0.5f * c;
            m.color.b = 0.0f;
            m.color.a = 1.0f;

            arr.markers.push_back(m);
        }

        vis_pub_->publish(arr);
    }

    // ---------------------------------------------------------
    //               MEMBER VARIABLES
    // ---------------------------------------------------------
    std::string cloud_topic_, frame_id_;
    double x_min_, x_max_, bin_size_, lane_width_;
    double bump_min_height_, bump_max_height_;
    double bump_min_width_, bump_max_width_;
    double detect_min_x_, detect_max_x_;
    int    min_points_per_bin_;

    double plane_fit_x_min_, plane_fit_x_max_;
    int    plane_fit_min_points_;
    double plane_refine_cutoff_m_;

    int smooth_window_;
    int num_bins_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundPlaneSpeedbumpNode>());
    rclcpp::shutdown();
    return 0;
}

