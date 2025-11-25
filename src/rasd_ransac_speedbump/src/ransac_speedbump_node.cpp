// ================================================================
//  RASD – Ground Plane Speedbump Detector (Early + Temporal V3)
//  Mode: Early detection, low conf far, strong conf close
//  Date: 2025-11-23
// ================================================================

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <vector>
#include <deque>
#include <limits>
#include <cmath>
#include <algorithm>
#include <string>

using std::placeholders::_1;

// ===============================================================
//  Helper
// ===============================================================
inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---------------------------------------------------------------
//  SPEEDBUMP NODE
// ---------------------------------------------------------------
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

        // ROI region (should match your ROI filter)
        declare_parameter<double>("x_min", 0.0);
        declare_parameter<double>("x_max", 50.0);
        declare_parameter<double>("bin_size", 0.30);
        declare_parameter<double>("lane_width", 4.0);

        // vertical profile smoothing
        declare_parameter<int>("smooth_window", 1);

        // bump geometry
        declare_parameter<double>("bump_min_height", 0.06);
        declare_parameter<double>("bump_max_height", 0.29);
        declare_parameter<double>("bump_min_width", 0.40);
        declare_parameter<double>("bump_max_width", 3.50);

        // detection distance range
        declare_parameter<double>("detect_min_x", 2.0);   // start "early" but not too noisy
        declare_parameter<double>("detect_max_x", 35.0);  // useful early zone

        // minimum points per bin (bin is ~20cm long)
        declare_parameter<int>("min_points_per_bin", 6);

        // ground plane fit region
        declare_parameter<double>("plane_fit_x_min", 1.5);
        declare_parameter<double>("plane_fit_x_max", 8.0);
        declare_parameter<int>("plane_fit_min_points", 250);
        declare_parameter<double>("plane_refine_cutoff_m", 0.05);

        // intensity (debug only)
        declare_parameter<double>("intensity_min", 10.0);
        declare_parameter<double>("intensity_max", 80.0);

        // temporal logic parameters (can be tuned via params)
        declare_parameter<int>("temporal_history_frames", 3);      // we look across N frames
        declare_parameter<double>("temporal_match_tolerance", 1.5); // meters between matches
        declare_parameter<int>("temporal_min_matches", 2);          // appear in ≥2 frames

        // ---------------------------------------------------------
        //                 LOAD PARAMETERS
        // ---------------------------------------------------------
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

        get_parameter("intensity_min", intensity_min_);
        get_parameter("intensity_max", intensity_max_);

        get_parameter("temporal_history_frames", temporal_history_frames_);
        get_parameter("temporal_match_tolerance", temporal_match_tolerance_);
        get_parameter("temporal_min_matches", temporal_min_matches_);

        // ---------------------------------------------------------
        //       SUBSCRIBERS + RVIZ PUBLISHER + LiDAR PUB
        // ---------------------------------------------------------
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&GroundPlaneSpeedbumpNode::cloudCallback, this, _1));

        vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/rasd/speedbump_groundplane_vis", 10);

        lidar_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/rasd/lidar_bump", 10);

        num_bins_ = static_cast<int>((x_max_ - x_min_) / bin_size_);

        RCLCPP_INFO(get_logger(),
            "✅ Speedbump Node READY — bins=%d | detect_range=[%.1f..%.1f] | temporal=%d frames",
            num_bins_, detect_min_x_, detect_max_x_, temporal_history_frames_);
    }

private:
    // ---------------------------------------------------------
    //                        BUMP STRUCT
    // ---------------------------------------------------------
    struct Bump
    {
        int   start_bin;
        int   end_bin;
        float x_center;
        float width;
        float height;
        float confidence;

        float height_score;
        float density_score;
        float shape_score;
        float range_score;
        float intensity_score;
        float coverage;      // fraction of bins with real data
    };

    // temporal history of bumps (one vector per frame)
    std::deque<std::vector<Bump>> bump_history_;

    // ---------------------------------------------------------
    //           ROBUST LEAST-SQUARES GROUND PLANE FIT
    // ---------------------------------------------------------
    bool robustFitGroundPlane(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                              double& a, double& b, double& c)
    {
        const float half_w = static_cast<float>(lane_width_ * 0.5f);

        std::vector<Eigen::Vector3d> pts;
        pts.reserve(cloud.points.size());

        for (const auto& p : cloud)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;

            if (p.x < plane_fit_x_min_ || p.x > plane_fit_x_max_)
                continue;

            if (std::fabs(p.y) > half_w)
                continue;

            // assume ground around [-1, 0.5] in base_link
            if (p.z < -1.0f || p.z > 0.5f)
                continue;

            pts.emplace_back(p.x, p.y, p.z);
        }

        int N = pts.size();
        if (N < plane_fit_min_points_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "⚠️ Not enough points for plane fit: %d < %d", N, plane_fit_min_points_);
            return false;
        }

        Eigen::MatrixXd A(N, 3);
        Eigen::VectorXd Z(N);

        for (int i = 0; i < N; ++i)
        {
            A(i, 0) = pts[i].x();
            A(i, 1) = pts[i].y();
            A(i, 2) = 1.0;
            Z(i)    = pts[i].z();
        }

        Eigen::Vector3d coeff = A.colPivHouseholderQr().solve(Z);
        double a0 = coeff(0), b0 = coeff(1), c0 = coeff(2);

        // Residuals
        std::vector<double> residuals(N);
        for (int i = 0; i < N; ++i)
        {
            double z_est = a0*pts[i].x() + b0*pts[i].y() + c0;
            residuals[i] = pts[i].z() - z_est;
        }

        std::vector<double> abs_res = residuals;
        for (auto& r : abs_res) r = std::fabs(r);

        size_t mid = abs_res.size() / 2;
        std::nth_element(abs_res.begin(), abs_res.begin() + mid, abs_res.end());
        double mad = abs_res[mid];

        double cutoff = std::max(plane_refine_cutoff_m_, 2.5 * mad);

        std::vector<Eigen::Vector3d> inliers;
        inliers.reserve(N);

        for (int i = 0; i < N; ++i)
        {
            if (std::fabs(residuals[i]) <= cutoff)
                inliers.push_back(pts[i]);
        }

        if (inliers.size() < plane_fit_min_points_/2)
        {
            a = a0; b = b0; c = c0;
            return true;
        }

        // Refined LS
        int M = inliers.size();
        Eigen::MatrixXd A2(M, 3);
        Eigen::VectorXd Z2(M);

        for (int i = 0; i < M; ++i)
        {
            A2(i, 0) = inliers[i].x();
            A2(i, 1) = inliers[i].y();
            A2(i, 2) = 1.0;
            Z2(i)    = inliers[i].z();
        }

        Eigen::Vector3d coeff2 = A2.colPivHouseholderQr().solve(Z2);
        a = coeff2(0); b = coeff2(1); c = coeff2(2);

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 3000,
            "Plane fit: a=%.5f, b=%.5f, c=%.5f (pts %d→%d)",
            a, b, c, N, M);

        return true;
    }

    // ==========================================================
    //                     MAIN CALLBACK
    // ==========================================================
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        if (cloud.empty())
            return;

        // ------------------------------------------------------
        // 1) FIT GROUND PLANE
        // ------------------------------------------------------
        double a = 0.0, b = 0.0, c = 0.0;
        if (!robustFitGroundPlane(cloud, a, b, c))
        {
            RCLCPP_WARN(get_logger(), "Plane fit failed. Skipping frame.");
            return;
        }

        const float half_w = lane_width_ * 0.5f;

        // ------------------------------------------------------
        // 2) BINNING
        // ------------------------------------------------------
        std::vector<std::vector<float>> z_bins(num_bins_);
        std::vector<std::vector<float>> intensity_bins(num_bins_);
        std::vector<int> counts(num_bins_, 0);

        for (const auto& p : cloud)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;

            if (p.x < x_min_ || p.x > x_max_)
                continue;

            if (std::fabs(p.y) > half_w)
                continue;

            int bin = static_cast<int>((p.x - x_min_) / bin_size_);
            if (bin < 0 || bin >= num_bins_)
                continue;

            float z_est = static_cast<float>(a * p.x + b * p.y + c);
            float dz = p.z - z_est;

            z_bins[bin].push_back(dz);
            intensity_bins[bin].push_back(p.intensity);
            counts[bin]++;
        }

        // ------------------------------------------------------
        // 3) MEDIAN PROFILE
        // ------------------------------------------------------
        std::vector<float> z_med(num_bins_, NAN);
        for (int i = 0; i < num_bins_; ++i)
        {
            auto& v = z_bins[i];
            if (static_cast<int>(v.size()) < min_points_per_bin_)
                continue;

            size_t mid = v.size() / 2;
            std::nth_element(v.begin(), v.begin() + mid, v.end());
            z_med[i] = v[mid];
        }

        // ------------------------------------------------------
        // 4) SAFE SMOOTHING (no hallucinated bumps)
        // ------------------------------------------------------
        std::vector<float> z_smooth(num_bins_, NAN);

        for (int i = 0; i < num_bins_; ++i)
        {
            int W = smooth_window_;
            float sum = 0.0f;
            int valid = 0;
            int total = 0;

            for (int k = -W; k <= W; ++k)
            {
                int idx = i + k;
                if (idx < 0 || idx >= num_bins_)
                    continue;

                total++;

                if (std::isfinite(z_med[idx]))
                {
                    sum += z_med[idx];
                    valid++;
                }
            }

            // need ≥3 valid and ≥60% coverage to smooth
            if (total > 0 &&
                valid >= 3 &&
                valid >= static_cast<int>(0.6f * total))
            {
                z_smooth[i] = sum / valid;
            }
        }

        // ------------------------------------------------------
        // 5) BUMP DETECTION – ALL CANDIDATES (PER FRAME)
        // ------------------------------------------------------
        std::vector<Bump> current_bumps;
        int i = 0;

        while (i < num_bins_)
        {
            if (!std::isfinite(z_smooth[i]) ||
                z_smooth[i] < bump_min_height_)
            {
                i++;
                continue;
            }

            int start = i;
            int end   = i;
            float max_dz = z_smooth[i];
            int sum_count = counts[i];

            i++;
            while (i < num_bins_ &&
                   std::isfinite(z_smooth[i]) &&
                   z_smooth[i] >= bump_min_height_)
            {
                end = i;
                sum_count += counts[i];
                if (z_smooth[i] > max_dz)
                    max_dz = z_smooth[i];
                i++;
            }

            float width = (end - start + 1) * bin_size_;
            float x_center = x_min_ + ((start + end) / 2.0f + 0.5f) * bin_size_;
            float height = max_dz;

            // Range & width checks
            if (x_center < detect_min_x_ || x_center > detect_max_x_)
                continue;

            if (width < bump_min_width_ || width > bump_max_width_)
                continue;

            // Data coverage (bins that actually had enough points)
            int total_bins = end - start + 1;
            int bins_with_data = 0;
            for (int k = start; k <= end; ++k)
            {
                if (std::isfinite(z_smooth[k]) &&
                    counts[k] >= min_points_per_bin_)
                {
                    bins_with_data++;
                }
            }
            float coverage = static_cast<float>(bins_with_data) /
                             static_cast<float>(std::max(total_bins, 1));

            // dynamic coverage threshold: near needs more coverage than far
            float coverage_min = (x_center < 20.0f) ? 0.6f : 0.4f;
            if (coverage < coverage_min)
            {
                RCLCPP_DEBUG(get_logger(),
                    "❌ Reject candidate @ X=%.1fm – coverage=%.2f < %.2f",
                    x_center, coverage, coverage_min);
                continue;
            }

            // --- height_score ---
            float height_score = 0.0f;
            if (bump_max_height_ > bump_min_height_)
            {
                height_score =
                    (height - static_cast<float>(bump_min_height_)) /
                    static_cast<float>(bump_max_height_ - bump_min_height_);
                height_score = clampf(height_score, 0.0f, 1.0f);
            }

            // --- density_score (points per bin averaged) ---
            float avg_pts_per_bin =
                static_cast<float>(sum_count) / static_cast<float>(total_bins);
            float density_score = clampf(avg_pts_per_bin / 10.0f, 0.0f, 1.0f);

            // --- shape_score ---
            float shape_score = 0.0f;
            {
                std::vector<float> vals;
                vals.reserve(total_bins);
                for (int k = start; k <= end; ++k)
                {
                    if (std::isfinite(z_smooth[k]))
                        vals.push_back(z_smooth[k]);
                }

                if (vals.size() >= 2)
                {
                    float mean = 0.0f;
                    for (float v : vals) mean += v;
                    mean /= static_cast<float>(vals.size());

                    float var = 0.0f;
                    for (float v : vals)
                    {
                        float d = v - mean;
                        var += d * d;
                    }
                    var /= static_cast<float>(vals.size());
                    float stddev = std::sqrt(var);

                    // small stddev → smoother bump
                    shape_score = std::exp(-stddev * 10.0f);
                    shape_score = clampf(shape_score, 0.0f, 1.0f);
                }
            }

            // --- range_score (early detection: far is allowed, but lower) ---
            float t = (x_center - detect_min_x_) /
                      std::max(1e-3, (detect_max_x_ - detect_min_x_));
            t = clampf(t, 0.0f, 1.0f);
            // near → 1.0, far → ~0.3
            float range_score = 1.0f - 0.7f * t;

            // --- intensity_score (debug only) ---
            float intensity_score = 0.5f;
            {
                std::vector<float> ints;
                for (int k = start; k <= end; ++k)
                    for (float I : intensity_bins[k])
                        ints.push_back(I);

                if (!ints.empty())
                {
                    float sumI = 0.0f;
                    for (float I : ints) sumI += I;
                    float avgI = sumI / static_cast<float>(ints.size());

                    float denom = static_cast<float>(intensity_max_ - intensity_min_);
                    if (denom < 1e-3f) denom = 1.0f;
                    float ti = (avgI - static_cast<float>(intensity_min_)) / denom;
                    intensity_score = clampf(ti, 0.0f, 1.0f);
                }
            }

            // --- FINAL CONFIDENCE ---
            // early detection: strong weight on height & shape,
            // some weight on density, range boosts near bumps.
            float confidence =
                0.45f * height_score +
                0.25f * shape_score  +
                0.15f * density_score+
                0.15f * range_score;

            confidence = clampf(confidence, 0.0f, 1.0f);

            Bump b;
            b.start_bin       = start;
            b.end_bin         = end;
            b.x_center        = x_center;
            b.height          = height;
            b.width           = width;
            b.confidence      = confidence;
            b.height_score    = height_score;
            b.density_score   = density_score;
            b.shape_score     = shape_score;
            b.range_score     = range_score;
            b.intensity_score = intensity_score;
            b.coverage        = coverage;

            current_bumps.push_back(b);

            std::string emoji =
                (confidence >= 0.80f) ? "🟥" :
                (confidence >= 0.55f) ? "🟨" : "🟩";

            RCLCPP_INFO(get_logger(),
                "%s Candidate: X=%.1fm | H=%.2fm | W=%.2fm | C=%.2f "
                "(h=%.2f d=%.2f s=%.2f r=%.2f cov=%.2f int=%.2f)",
                emoji.c_str(),
                x_center, height, width, confidence,
                height_score, density_score, shape_score,
                range_score, coverage, intensity_score);
        }

        // ------------------------------------------------------
        // 6) TEMPORAL FILTER ON CURRENT FRAME CANDIDATES
        // ------------------------------------------------------
        auto confirmed_bumps = applyTemporalFilter(current_bumps);

        // ------------------------------------------------------
        // 7) PUBLISH ONLY CONFIRMED BUMPS TO FUSION
        // ------------------------------------------------------
        for (const auto& cb : confirmed_bumps)
        {
            std::string e =
                (cb.confidence >= 0.80f) ? "🟥" :
                (cb.confidence >= 0.55f) ? "🟨" : "🟩";

            RCLCPP_ERROR(get_logger(),
                "%s *** SPEED BUMP CONFIRMED *** Dist=%.1fm | Conf=%.2f | Height=%.2f",
                e.c_str(), cb.x_center, cb.confidence, cb.height);

            std_msgs::msg::Float32MultiArray out;
            out.data = {cb.x_center, cb.confidence, cb.height};
            lidar_pub_->publish(out);
        }

        // ------------------------------------------------------
        // 8) RVIZ VISUALIZATION
        // ------------------------------------------------------
        publishVisualization(current_bumps, confirmed_bumps);
    }

    // ---------------------------------------------------------
    //                TEMPORAL FILTER (multi-frame)
    // ---------------------------------------------------------
    std::vector<Bump> applyTemporalFilter(const std::vector<Bump>& current_bumps)
    {
        // add current frame to history
        bump_history_.push_back(current_bumps);
        if (static_cast<int>(bump_history_.size()) > temporal_history_frames_)
            bump_history_.pop_front();

        std::vector<Bump> confirmed;

        if (bump_history_.empty() || current_bumps.empty())
            return confirmed;

        // For each current bump, see in how many frames it appears
        for (const auto& curr : current_bumps)
        {
            int matches = 0;
            float sum_x = 0.0f;
            float sum_conf = 0.0f;
            float sum_h = 0.0f;

            for (const auto& frame_bumps : bump_history_)
            {
                bool found = false;
                for (const auto& b : frame_bumps)
                {
                    if (std::fabs(b.x_center - curr.x_center) <= temporal_match_tolerance_)
                    {
                        matches++;
                        sum_x += b.x_center;
                        sum_conf += b.confidence;
                        sum_h += b.height;
                        found = true;
                        break; // at most one match per frame
                    }
                }
                // For now, not enforcing strict "consecutive" frames; more forgiving
            }

            if (matches < temporal_min_matches_)
                continue;

            // average over matched frames
            Bump fused = curr;
            fused.x_center  = sum_x / matches;
            fused.confidence= sum_conf / matches;
            fused.height    = sum_h / matches;

            // Distance-based confidence requirement:
            //  - far (25–35m): allow low conf (≥0.35)
            //  - mid (15–25m): require moderate conf (≥0.45)
            //  - near (<15m): require strong conf (≥0.65)
            float min_conf_req = 0.35f;
            if (fused.x_center < 15.0f)
                min_conf_req = 0.65f;
            else if (fused.x_center < 25.0f)
                min_conf_req = 0.45f;

            if (fused.confidence >= min_conf_req)
            {
                confirmed.push_back(fused);

                std::string e =
                    (fused.confidence >= 0.80f) ? "🟥" :
                    (fused.confidence >= 0.55f) ? "🟨" : "🟩";

                RCLCPP_INFO(get_logger(),
                    "%s Temporal confirm: X=%.1fm | Conf=%.2f | H=%.2f | matches=%d (min_req=%.2f)",
                    e.c_str(), fused.x_center, fused.confidence,
                    fused.height, matches, min_conf_req);
            }
            else
            {
                RCLCPP_INFO(get_logger(),
                    "❌ Temporal candidate rejected: X=%.1fm | Conf=%.2f < min_req=%.2f (matches=%d)",
                    fused.x_center, fused.confidence, min_conf_req, matches);
            }
        }

        // Optionally merge very close confirmed bumps (within 1m)
        if (confirmed.size() > 1)
        {
            std::sort(confirmed.begin(), confirmed.end(),
                      [](const Bump& a, const Bump& b)
                      { return a.x_center < b.x_center; });

            std::vector<Bump> merged;
            merged.push_back(confirmed[0]);
            for (size_t i = 1; i < confirmed.size(); ++i)
            {
                Bump& last = merged.back();
                if (std::fabs(confirmed[i].x_center - last.x_center) < 1.0f)
                {
                    // keep the one with higher confidence
                    if (confirmed[i].confidence > last.confidence)
                        last = confirmed[i];
                }
                else
                {
                    merged.push_back(confirmed[i]);
                }
            }
            return merged;
        }

        return confirmed;
    }

    // ---------------------------------------------------------
    //                      RVIZ VISUALIZATION
    // ---------------------------------------------------------
    void publishVisualization(const std::vector<Bump>& all_candidates,
                              const std::vector<Bump>& confirmed_bumps)
    {
        visualization_msgs::msg::MarkerArray arr;

        // delete all previous markers
        visualization_msgs::msg::Marker del;
        del.header.frame_id = frame_id_;
        del.header.stamp    = now();
        del.ns              = "all";
        del.id              = 0;
        del.action          = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);

        // all candidates (semi-transparent)
        int id = 10;
        for (const auto& b : all_candidates)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp    = now();
            m.ns              = "candidates";
            m.id              = id++;
            m.type            = visualization_msgs::msg::Marker::SPHERE;

            m.pose.position.x = b.x_center;
            m.pose.position.y = 0.0;
            m.pose.position.z = b.height;

            m.scale.x = b.width;
            m.scale.y = b.width * 0.5f;
            m.scale.z = 0.2;

            float c = b.confidence;
            m.color.r = c;
            m.color.g = 1.0f - c;
            m.color.b = 0.0f;
            m.color.a = 0.4f;   // candidates = transparent

            arr.markers.push_back(m);
        }

        // confirmed bumps (solid)
        for (const auto& b : confirmed_bumps)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp    = now();
            m.ns              = "confirmed";
            m.id              = id++;
            m.type            = visualization_msgs::msg::Marker::SPHERE;

            m.pose.position.x = b.x_center;
            m.pose.position.y = 0.0;
            m.pose.position.z = b.height;

            m.scale.x = b.width;
            m.scale.y = b.width * 0.5f;
            m.scale.z = 0.25;   // a bit taller

            float c = b.confidence;
            m.color.r = c;
            m.color.g = 1.0f - c;
            m.color.b = 0.0f;
            m.color.a = 1.0f;   // solid

            arr.markers.push_back(m);
        }

        vis_pub_->publish(arr);
    }

    // ---------------------------------------------------------
    //                     MEMBER VARIABLES
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
    int    smooth_window_;
    int    num_bins_;

    double intensity_min_, intensity_max_;

    int    temporal_history_frames_;
    double temporal_match_tolerance_;
    int    temporal_min_matches_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   lidar_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundPlaneSpeedbumpNode>());
    rclcpp::shutdown();
    return 0;
}
