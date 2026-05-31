#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
double clampValue(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(value, maximum));
}

double quantile(std::vector<double> values, double q)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  q = clampValue(q, 0.0, 1.0);
  const double position = q * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<size_t>(std::floor(position));
  const auto upper = static_cast<size_t>(std::ceil(position));
  if (lower == upper) {
    return values[lower];
  }
  const double ratio = position - static_cast<double>(lower);
  return values[lower] * (1.0 - ratio) + values[upper] * ratio;
}

std::int64_t cellKey(int y_index, int z_index)
{
  const auto y = static_cast<std::uint64_t>(
    static_cast<std::uint32_t>(static_cast<std::int32_t>(y_index)));
  const auto z = static_cast<std::uint64_t>(
    static_cast<std::uint32_t>(static_cast<std::int32_t>(z_index)));
  return static_cast<std::int64_t>((y << 32) | z);
}

int cellYIndex(std::int64_t key)
{
  const auto packed = static_cast<std::uint64_t>(key);
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(packed >> 32));
}

int cellZIndex(std::int64_t key)
{
  const auto packed = static_cast<std::uint64_t>(key);
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(packed));
}
}  // namespace

class BoxGridLocatorNode : public rclcpp::Node
{
public:
  BoxGridLocatorNode() : Node("box_grid_locator_node")
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/odin1/cloud_render");
    debug_cloud_topic_ =
      declare_parameter<std::string>("debug_cloud_topic", "/navigation/box_cloud_debug");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/boxes/markers");
    target_id_topic_ = declare_parameter<std::string>("target_id_topic", "/vision/target_id");
    target_pose_topic_ =
      declare_parameter<std::string>("target_pose_topic", "/navigation/target_pose");
    status_topic_ = declare_parameter<std::string>("status_topic", "/boxes/status");
    target_row_.store(declare_parameter<int>("target_row", 0));
    target_col_.store(declare_parameter<int>("target_col", 0));

    x_min_ = declare_parameter<double>("x_min", 0.30);
    x_max_ = declare_parameter<double>("x_max", 4.50);
    y_min_ = declare_parameter<double>("y_min", -3.00);
    y_max_ = declare_parameter<double>("y_max", 3.00);
    z_min_ = declare_parameter<double>("z_min", -0.32);
    z_max_ = declare_parameter<double>("z_max", 0.05);
    self_filter_x_max_ = declare_parameter<double>("self_filter_x_max", 0.60);
    self_filter_y_abs_max_ = declare_parameter<double>("self_filter_y_abs_max", 0.60);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.015);

    candidate_extraction_mode_ =
      declare_parameter<std::string>("candidate_extraction_mode", "front_profile");
    yz_cell_y_size_ = declare_parameter<double>("yz_cell_y_size", 0.035);
    yz_cell_z_size_ = declare_parameter<double>("yz_cell_z_size", 0.035);
    yz_connect_y_gap_cells_ = declare_parameter<int>("yz_connect_y_gap_cells", 2);
    yz_connect_z_gap_cells_ = declare_parameter<int>("yz_connect_z_gap_cells", 2);
    yz_connect_x_tolerance_ = declare_parameter<double>("yz_connect_x_tolerance", 0.30);
    front_x_quantile_ = declare_parameter<double>("front_x_quantile", 0.20);
    front_face_depth_max_ = declare_parameter<double>("front_face_depth_max", 0.30);
    profile_z_min_ = declare_parameter<double>("profile_z_min", -0.24);
    profile_z_max_ = declare_parameter<double>("profile_z_max", 0.03);
    profile_y_bin_size_ = declare_parameter<double>("profile_y_bin_size", 0.05);
    profile_min_bin_points_ = declare_parameter<int>("profile_min_bin_points", 2);
    profile_connect_gap_bins_ = declare_parameter<int>("profile_connect_gap_bins", 2);
    profile_connect_x_tolerance_ =
      declare_parameter<double>("profile_connect_x_tolerance", 0.35);
    profile_min_segment_bins_ = declare_parameter<int>("profile_min_segment_bins", 2);
    profile_box_width_ = declare_parameter<double>("profile_box_width", 0.36);
    profile_candidate_x_tolerance_ =
      declare_parameter<double>("profile_candidate_x_tolerance", 0.35);
    profile_nms_y_tolerance_ = declare_parameter<double>("profile_nms_y_tolerance", 0.30);
    template_search_x_min_ = declare_parameter<double>("template_search_x_min", 0.80);
    template_search_x_max_ = declare_parameter<double>("template_search_x_max", 4.50);
    template_y_half_width_ = declare_parameter<double>("template_y_half_width", 0.22);
    template_front_margin_ = declare_parameter<double>("template_front_margin", 0.08);
    template_depth_ = declare_parameter<double>("template_depth", 0.42);
    template_x_tolerance_ = declare_parameter<double>("template_x_tolerance", 0.40);
    template_min_box_points_ = declare_parameter<int>("template_min_box_points", 5);
    template_min_total_points_ = declare_parameter<int>("template_min_total_points", 18);
    template_min_z_span_ = declare_parameter<double>("template_min_z_span", 0.12);
    template_upper_z_min_ = declare_parameter<double>("template_upper_z_min", -0.18);
    template_min_upper_points_ = declare_parameter<int>("template_min_upper_points", 2);

    min_face_points_ = declare_parameter<int>("min_face_points", 3);
    min_face_width_y_ = declare_parameter<double>("min_face_width_y", 0.03);
    max_face_width_y_ = declare_parameter<double>("max_face_width_y", 0.45);
    min_face_height_z_ = declare_parameter<double>("min_face_height_z", 0.03);
    max_face_height_z_ = declare_parameter<double>("max_face_height_z", 0.35);
    max_face_depth_x_ = declare_parameter<double>("max_face_depth_x", 0.45);
    near_row_depth_window_ = declare_parameter<double>("near_row_depth_window", 0.45);

    center_from_front_offset_ = declare_parameter<double>("center_from_front_offset", 0.125);
    publish_box_center_ = declare_parameter<bool>("publish_box_center", true);
    expected_cols_ = declare_parameter<int>("expected_cols", 4);
    sort_left_to_right_positive_y_ =
      declare_parameter<bool>("sort_left_to_right_positive_y", true);
    use_row_layout_filter_ = declare_parameter<bool>("use_row_layout_filter", true);
    row_col_spacing_ = declare_parameter<double>("row_col_spacing", 0.85);
    row_spacing_tolerance_ = declare_parameter<double>("row_spacing_tolerance", 0.25);
    row_x_tolerance_ = declare_parameter<double>("row_x_tolerance", 0.45);
    min_layout_matches_ = declare_parameter<int>("min_layout_matches", 3);
    infer_missing_boxes_ = declare_parameter<bool>("infer_missing_boxes", true);

    processing_period_ = declare_parameter<double>("processing_period", 0.20);
    marker_lifetime_ = declare_parameter<double>("marker_lifetime", 1.00);
    target_frame_override_ = declare_parameter<std::string>("target_frame_override", "");
    enable_diagnostics_ = declare_parameter<bool>("enable_diagnostics", true);
    status_include_faces_ = declare_parameter<bool>("status_include_faces", false);

    x_max_ = std::max(x_min_ + 0.1, x_max_);
    y_max_ = std::max(y_min_ + 0.1, y_max_);
    z_max_ = std::max(z_min_ + 0.02, z_max_);
    voxel_leaf_size_ = std::max(0.0, voxel_leaf_size_);
    yz_cell_y_size_ = std::max(0.01, yz_cell_y_size_);
    yz_cell_z_size_ = std::max(0.01, yz_cell_z_size_);
    yz_connect_y_gap_cells_ = std::max(1, yz_connect_y_gap_cells_);
    yz_connect_z_gap_cells_ = std::max(1, yz_connect_z_gap_cells_);
    yz_connect_x_tolerance_ = std::max(0.01, yz_connect_x_tolerance_);
    front_x_quantile_ = clampValue(front_x_quantile_, 0.0, 1.0);
    front_face_depth_max_ = std::max(0.02, front_face_depth_max_);
    profile_z_max_ = std::max(profile_z_min_ + 0.02, profile_z_max_);
    profile_y_bin_size_ = std::max(0.01, profile_y_bin_size_);
    profile_min_bin_points_ = std::max(1, profile_min_bin_points_);
    profile_connect_gap_bins_ = std::max(0, profile_connect_gap_bins_);
    profile_connect_x_tolerance_ = std::max(0.01, profile_connect_x_tolerance_);
    profile_min_segment_bins_ = std::max(1, profile_min_segment_bins_);
    profile_box_width_ = std::max(0.10, profile_box_width_);
    profile_candidate_x_tolerance_ = std::max(0.02, profile_candidate_x_tolerance_);
    profile_nms_y_tolerance_ = std::max(0.02, profile_nms_y_tolerance_);
    template_search_x_max_ = std::max(template_search_x_min_ + 0.1, template_search_x_max_);
    template_y_half_width_ = std::max(0.03, template_y_half_width_);
    template_front_margin_ = std::max(0.0, template_front_margin_);
    template_depth_ = std::max(0.05, template_depth_);
    template_x_tolerance_ = std::max(0.02, template_x_tolerance_);
    template_min_box_points_ = std::max(1, template_min_box_points_);
    template_min_total_points_ = std::max(1, template_min_total_points_);
    template_min_z_span_ = std::max(0.02, template_min_z_span_);
    template_min_upper_points_ = std::max(1, template_min_upper_points_);
    min_face_points_ = std::max(1, min_face_points_);
    max_face_width_y_ = std::max(min_face_width_y_, max_face_width_y_);
    max_face_height_z_ = std::max(min_face_height_z_, max_face_height_z_);
    max_face_depth_x_ = std::max(0.02, max_face_depth_x_);
    near_row_depth_window_ = std::max(0.05, near_row_depth_window_);
    expected_cols_ = std::clamp(expected_cols_, 1, 8);
    row_col_spacing_ = std::max(0.10, row_col_spacing_);
    row_spacing_tolerance_ = std::max(0.02, row_spacing_tolerance_);
    row_x_tolerance_ = std::max(0.05, row_x_tolerance_);
    min_layout_matches_ = std::clamp(min_layout_matches_, 1, expected_cols_);
    processing_period_ = std::max(0.05, processing_period_);
    center_from_front_offset_ = std::max(0.0, center_from_front_offset_);

    cloud_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    processing_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    target_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = cloud_callback_group_;
    auto cloud_qos = rclcpp::SensorDataQoS();
    cloud_qos.keep_last(1);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, cloud_qos,
      std::bind(&BoxGridLocatorNode::cloudCallback, this, std::placeholders::_1),
      cloud_options);

    rclcpp::SubscriptionOptions target_options;
    target_options.callback_group = target_callback_group_;
    target_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      target_id_topic_, 10,
      std::bind(&BoxGridLocatorNode::targetCallback, this, std::placeholders::_1),
      target_options);

    debug_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(debug_cloud_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(target_pose_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(processing_period_));
    processing_timer_ = create_wall_timer(
      period, std::bind(&BoxGridLocatorNode::processLatestCloud, this),
      processing_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "Box face locator started: cloud=%s ROI x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]",
      cloud_topic_.c_str(), x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
  }

private:
  struct FaceCell
  {
    std::vector<int> point_indices;
    double nearest_x = std::numeric_limits<double>::infinity();
  };

  struct ProfileBin
  {
    std::vector<int> point_indices;
    std::vector<double> xs;
    double front_x = 0.0;
    int count = 0;
  };

  struct FaceCandidate
  {
    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;
    double front_x = 0.0;
    double front_y = 0.0;
    double front_z = 0.0;
    double size_x = 0.0;
    double size_y = 0.0;
    double size_z = 0.0;
    int point_count = 0;
    int row = 0;
    int col = -1;
    bool inferred = false;
  };

  struct RowLayoutResult
  {
    bool valid = false;
    int matched_count = 0;
    double front_x = 0.0;
    double score = 0.0;
    std::vector<FaceCandidate> faces;
  };

  struct TemplateMatch
  {
    bool valid = false;
    double front_x = 0.0;
    double first_col_y = 0.0;
    double score = 0.0;
    int matched_boxes = 0;
    int total_points = 0;
    std::vector<FaceCandidate> faces;
  };

  enum class FaceRejectReason
  {
    kNone,
    kPoints,
    kWidth,
    kHeight,
    kDepth
  };

  struct ExtractionStats
  {
    size_t yz_cells = 0;
    size_t components = 0;
    size_t rejected_by_points = 0;
    size_t rejected_by_width = 0;
    size_t rejected_by_height = 0;
    size_t rejected_by_depth = 0;
    std::string mode;
  };

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
    latest_cloud_msg_ = msg;
    latest_cloud_sequence_++;
  }

  void targetCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 2) {
      RCLCPP_WARN(get_logger(), "Ignoring target id: expected [row, col].");
      return;
    }
    target_row_.store(msg->data[0]);
    target_col_.store(msg->data[1]);
  }

  void processLatestCloud()
  {
    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
      if (!latest_cloud_msg_ || latest_cloud_sequence_ == processed_cloud_sequence_) {
        return;
      }
      msg = latest_cloud_msg_;
      sequence = latest_cloud_sequence_;
      processed_cloud_sequence_ = latest_cloud_sequence_;
    }
    processCloud(msg, sequence);
  }

  void processCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    uint64_t sequence)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *raw_cloud);

    const auto roi_cloud = cropRoi(raw_cloud);
    const auto self_filtered_cloud = filterSelfPoints(roi_cloud);
    const auto working_cloud = downsampleCloud(self_filtered_cloud);
    publishDebugCloud(msg->header, working_cloud);

    ExtractionStats extraction_stats;
    auto all_faces = extractCandidates(working_cloud, &extraction_stats);
    RowLayoutResult row_layout;
    std::vector<FaceCandidate> first_row_faces;
    const auto template_match = fitTemplateRow(working_cloud);
    if (template_match.valid) {
      first_row_faces = template_match.faces;
      row_layout.valid = true;
      row_layout.matched_count = template_match.matched_boxes;
      row_layout.front_x = template_match.front_x;
      row_layout.score = template_match.score;
    } else if (use_row_layout_filter_) {
      row_layout = fitBestRowLayout(all_faces);
      first_row_faces = row_layout.faces;
    } else {
      first_row_faces = selectFirstRow(all_faces);
      assignColumns(first_row_faces);
      row_layout.valid = !first_row_faces.empty();
      row_layout.matched_count = static_cast<int>(first_row_faces.size());
      if (!first_row_faces.empty()) {
        row_layout.front_x = first_row_faces.front().front_x;
      }
    }

    publishMarkersAndTarget(msg->header, first_row_faces);
    publishStatus(
      sequence, msg->header.frame_id, raw_cloud->points.size(), roi_cloud->points.size(),
      self_filtered_cloud->points.size(), working_cloud->points.size(), all_faces.size(),
      extraction_stats, row_layout, first_row_faces);
  }

  std::vector<FaceCandidate> extractCandidates(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    ExtractionStats * stats) const
  {
    if (candidate_extraction_mode_ == "yz_faces") {
      if (stats) {
        stats->mode = "yz_faces";
      }
      return extractVerticalFaces(cloud, stats);
    }
    if (stats) {
      stats->mode = "front_profile";
    }
    return extractFrontProfileCandidates(cloud, stats);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cropRoi(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    filtered->reserve(cloud->points.size());
    for (const auto & point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (point.x < x_min_ || point.x > x_max_) {
        continue;
      }
      if (point.y < y_min_ || point.y > y_max_) {
        continue;
      }
      if (point.z < z_min_ || point.z > z_max_) {
        continue;
      }
      filtered->points.push_back(point);
    }
    filtered->width = static_cast<uint32_t>(filtered->points.size());
    filtered->height = 1;
    filtered->is_dense = true;
    return filtered;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filterSelfPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    filtered->reserve(cloud->points.size());
    for (const auto & point : cloud->points) {
      const bool near_self =
        point.x <= self_filter_x_max_ && std::abs(point.y) <= self_filter_y_abs_max_;
      if (!near_self) {
        filtered->points.push_back(point);
      }
    }
    filtered->width = static_cast<uint32_t>(filtered->points.size());
    filtered->height = 1;
    filtered->is_dense = cloud->is_dense;
    return filtered;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
  {
    if (voxel_leaf_size_ <= 1e-6 || cloud->points.empty()) {
      return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, std::max(0.01, voxel_leaf_size_));
    voxel.filter(*downsampled);
    return downsampled;
  }

  std::vector<FaceCandidate> extractVerticalFaces(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    ExtractionStats * stats) const
  {
    std::unordered_map<std::int64_t, FaceCell> cells;
    cells.reserve(cloud->points.size());

    for (size_t i = 0; i < cloud->points.size(); ++i) {
      const auto & point = cloud->points[i];
      const int y_index = static_cast<int>(std::floor(point.y / yz_cell_y_size_));
      const int z_index = static_cast<int>(std::floor(point.z / yz_cell_z_size_));
      auto & cell = cells[cellKey(y_index, z_index)];
      cell.point_indices.push_back(static_cast<int>(i));
      cell.nearest_x = std::min(cell.nearest_x, static_cast<double>(point.x));
    }
    if (stats) {
      stats->yz_cells = cells.size();
    }

    std::unordered_set<std::int64_t> visited;
    visited.reserve(cells.size());
    std::vector<FaceCandidate> candidates;

    for (const auto & item : cells) {
      const auto start_key = item.first;
      if (visited.find(start_key) != visited.end()) {
        continue;
      }

      std::vector<std::int64_t> component_cells;
      std::queue<std::int64_t> pending;
      pending.push(start_key);
      visited.insert(start_key);

      while (!pending.empty()) {
        const auto key = pending.front();
        pending.pop();
        component_cells.push_back(key);
        const int y_index = cellYIndex(key);
        const int z_index = cellZIndex(key);
        const auto & current_cell = cells.at(key);

        for (int dy = -yz_connect_y_gap_cells_; dy <= yz_connect_y_gap_cells_; ++dy) {
          for (int dz = -yz_connect_z_gap_cells_; dz <= yz_connect_z_gap_cells_; ++dz) {
            if (dy == 0 && dz == 0) {
              continue;
            }
            const auto neighbor_key = cellKey(y_index + dy, z_index + dz);
            if (visited.find(neighbor_key) != visited.end()) {
              continue;
            }
            const auto neighbor = cells.find(neighbor_key);
            if (neighbor == cells.end()) {
              continue;
            }
            if (std::abs(neighbor->second.nearest_x - current_cell.nearest_x) >
                yz_connect_x_tolerance_) {
              continue;
            }
            visited.insert(neighbor_key);
            pending.push(neighbor_key);
          }
        }
      }

      if (stats) {
        stats->components++;
      }
      auto candidate = makeFaceCandidate(cloud, cells, component_cells);
      const auto reject_reason = faceRejectReason(candidate);
      if (reject_reason == FaceRejectReason::kNone) {
        candidates.push_back(candidate);
      } else {
        countRejection(stats, reject_reason);
      }
    }

    std::sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      return a.front_x < b.front_x;
    });
    return candidates;
  }

  std::vector<FaceCandidate> extractFrontProfileCandidates(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    ExtractionStats * stats) const
  {
    std::unordered_map<int, ProfileBin> bins;
    bins.reserve(cloud->points.size());

    for (size_t i = 0; i < cloud->points.size(); ++i) {
      const auto & point = cloud->points[i];
      if (point.z < profile_z_min_ || point.z > profile_z_max_) {
        continue;
      }
      const int y_index = static_cast<int>(std::floor(point.y / profile_y_bin_size_));
      auto & bin = bins[y_index];
      bin.point_indices.push_back(static_cast<int>(i));
      bin.xs.push_back(point.x);
    }

    std::vector<int> ordered_indices;
    ordered_indices.reserve(bins.size());
    for (auto & item : bins) {
      auto & bin = item.second;
      if (static_cast<int>(bin.point_indices.size()) < profile_min_bin_points_) {
        continue;
      }
      bin.count = static_cast<int>(bin.point_indices.size());
      bin.front_x = quantile(bin.xs, front_x_quantile_);
      ordered_indices.push_back(item.first);
    }
    std::sort(ordered_indices.begin(), ordered_indices.end());

    if (stats) {
      stats->yz_cells = ordered_indices.size();
    }

    if (ordered_indices.empty()) {
      return {};
    }

    auto window_candidates =
      extractFrontProfileWindowCandidates(cloud, bins, ordered_indices, stats);
    if (!window_candidates.empty()) {
      return window_candidates;
    }

    std::vector<FaceCandidate> candidates;
    std::vector<int> segment_bins;
    double segment_front_x = 0.0;
    int previous_index = 0;
    bool has_segment = false;

    const auto flush_segment = [&]() {
      if (segment_bins.empty()) {
        return;
      }
      if (stats) {
        stats->components++;
      }
      auto candidate = makeProfileCandidate(cloud, bins, segment_bins);
      const auto reject_reason = faceRejectReason(candidate);
      if (
        reject_reason == FaceRejectReason::kNone &&
        static_cast<int>(segment_bins.size()) >= profile_min_segment_bins_) {
        candidates.push_back(candidate);
      } else if (static_cast<int>(segment_bins.size()) < profile_min_segment_bins_) {
        countRejection(stats, FaceRejectReason::kPoints);
      } else {
        countRejection(stats, reject_reason);
      }
      segment_bins.clear();
      has_segment = false;
      segment_front_x = 0.0;
    };

    for (const int index : ordered_indices) {
      const auto & bin = bins.at(index);
      if (!has_segment) {
        segment_bins.push_back(index);
        segment_front_x = bin.front_x;
        previous_index = index;
        has_segment = true;
        continue;
      }

      const bool close_in_y =
        index - previous_index <= profile_connect_gap_bins_ + 1;
      const bool close_in_x =
        std::abs(bin.front_x - segment_front_x) <= profile_connect_x_tolerance_;
      if (!close_in_y || !close_in_x) {
        flush_segment();
        segment_bins.push_back(index);
        segment_front_x = bin.front_x;
        previous_index = index;
        has_segment = true;
        continue;
      }

      segment_bins.push_back(index);
      segment_front_x =
        0.7 * segment_front_x + 0.3 * bin.front_x;
      previous_index = index;
    }
    flush_segment();

    std::sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      return a.front_x < b.front_x;
    });
    return candidates;
  }

  std::vector<FaceCandidate> extractFrontProfileWindowCandidates(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::unordered_map<int, ProfileBin> & bins,
    const std::vector<int> & ordered_indices,
    ExtractionStats * stats) const
  {
    std::vector<FaceCandidate> raw_candidates;
    const int half_window_bins = std::max(
      1, static_cast<int>(std::round((profile_box_width_ * 0.5) / profile_y_bin_size_)));

    for (const int center_index : ordered_indices) {
      const auto & center_bin = bins.at(center_index);
      std::vector<int> window_bins;
      window_bins.reserve(static_cast<size_t>(half_window_bins * 2 + 1));
      for (int offset = -half_window_bins; offset <= half_window_bins; ++offset) {
        const int bin_index = center_index + offset;
        const auto found = bins.find(bin_index);
        if (found == bins.end()) {
          continue;
        }
        if (found->second.count < profile_min_bin_points_) {
          continue;
        }
        if (std::abs(found->second.front_x - center_bin.front_x) >
            profile_candidate_x_tolerance_) {
          continue;
        }
        window_bins.push_back(bin_index);
      }

      if (static_cast<int>(window_bins.size()) < profile_min_segment_bins_) {
        continue;
      }

      auto candidate = makeProfileCandidate(cloud, bins, window_bins);
      const auto reject_reason = faceRejectReason(candidate);
      if (stats) {
        stats->components++;
      }
      if (reject_reason == FaceRejectReason::kNone) {
        raw_candidates.push_back(candidate);
      } else {
        countRejection(stats, reject_reason);
      }
    }

    std::sort(raw_candidates.begin(), raw_candidates.end(), [](const auto & a, const auto & b) {
      if (a.point_count != b.point_count) {
        return a.point_count > b.point_count;
      }
      return a.size_y > b.size_y;
    });

    std::vector<FaceCandidate> kept;
    for (const auto & candidate : raw_candidates) {
      bool overlaps_existing = false;
      for (const auto & existing : kept) {
        const bool close_y =
          std::abs(candidate.front_y - existing.front_y) <= profile_nms_y_tolerance_;
        const bool close_x =
          std::abs(candidate.front_x - existing.front_x) <= profile_candidate_x_tolerance_;
        if (close_y && close_x) {
          overlaps_existing = true;
          break;
        }
      }
      if (!overlaps_existing) {
        kept.push_back(candidate);
      }
    }

    std::sort(kept.begin(), kept.end(), [](const auto & a, const auto & b) {
      return a.front_x < b.front_x;
    });
    return kept;
  }

  FaceCandidate makeFaceCandidate(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::unordered_map<std::int64_t, FaceCell> & cells,
    const std::vector<std::int64_t> & component_cells) const
  {
    std::vector<double> xs;
    size_t point_count = 0;
    for (const auto key : component_cells) {
      point_count += cells.at(key).point_indices.size();
    }
    xs.reserve(point_count);
    for (const auto key : component_cells) {
      for (const int index : cells.at(key).point_indices) {
        xs.push_back(cloud->points[static_cast<size_t>(index)].x);
      }
    }

    const double front_x = quantile(xs, front_x_quantile_);
    const double max_kept_x = front_x + front_face_depth_max_;

    std::vector<int> indices;
    indices.reserve(point_count);
    for (const auto key : component_cells) {
      for (const int index : cells.at(key).point_indices) {
        if (cloud->points[static_cast<size_t>(index)].x <= max_kept_x) {
          indices.push_back(index);
        }
      }
    }
    if (indices.empty()) {
      for (const auto key : component_cells) {
        for (const int index : cells.at(key).point_indices) {
          indices.push_back(index);
        }
      }
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();

    for (const int index : indices) {
      const auto & point = cloud->points[static_cast<size_t>(index)];
      sum_x += point.x;
      sum_y += point.y;
      sum_z += point.z;
      min_x = std::min(min_x, static_cast<double>(point.x));
      max_x = std::max(max_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_y = std::max(max_y, static_cast<double>(point.y));
      min_z = std::min(min_z, static_cast<double>(point.z));
      max_z = std::max(max_z, static_cast<double>(point.z));
    }

    const double inv_count = indices.empty() ? 0.0 : 1.0 / static_cast<double>(indices.size());
    FaceCandidate candidate;
    candidate.front_x = front_x;
    candidate.front_y = sum_y * inv_count;
    candidate.front_z = sum_z * inv_count;
    candidate.center_x = publish_box_center_ ? front_x + center_from_front_offset_ : front_x;
    candidate.center_y = candidate.front_y;
    candidate.center_z = candidate.front_z;
    candidate.size_x = max_x - min_x;
    candidate.size_y = max_y - min_y;
    candidate.size_z = max_z - min_z;
    candidate.point_count = static_cast<int>(indices.size());
    return candidate;
  }

  FaceCandidate makeProfileCandidate(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::unordered_map<int, ProfileBin> & bins,
    const std::vector<int> & segment_bins) const
  {
    std::vector<int> all_indices;
    std::vector<double> front_xs;
    size_t point_count = 0;
    for (const int bin_index : segment_bins) {
      const auto & bin = bins.at(bin_index);
      point_count += bin.point_indices.size();
      front_xs.push_back(bin.front_x);
    }
    all_indices.reserve(point_count);
    for (const int bin_index : segment_bins) {
      const auto & bin = bins.at(bin_index);
      for (const int point_index : bin.point_indices) {
        all_indices.push_back(point_index);
      }
    }

    const double front_x = quantile(front_xs, 0.5);
    const double max_kept_x = front_x + front_face_depth_max_;

    std::vector<int> front_indices;
    front_indices.reserve(all_indices.size());
    for (const int point_index : all_indices) {
      const auto & point = cloud->points[static_cast<size_t>(point_index)];
      if (point.x <= max_kept_x) {
        front_indices.push_back(point_index);
      }
    }
    if (front_indices.empty()) {
      front_indices = all_indices;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();

    for (const int point_index : front_indices) {
      const auto & point = cloud->points[static_cast<size_t>(point_index)];
      sum_x += point.x;
      sum_y += point.y;
      sum_z += point.z;
      min_x = std::min(min_x, static_cast<double>(point.x));
      max_x = std::max(max_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_y = std::max(max_y, static_cast<double>(point.y));
      min_z = std::min(min_z, static_cast<double>(point.z));
      max_z = std::max(max_z, static_cast<double>(point.z));
    }

    const double inv_count =
      front_indices.empty() ? 0.0 : 1.0 / static_cast<double>(front_indices.size());
    FaceCandidate candidate;
    candidate.front_x = front_x;
    candidate.front_y = sum_y * inv_count;
    candidate.front_z = sum_z * inv_count;
    candidate.center_x = publish_box_center_ ? front_x + center_from_front_offset_ : front_x;
    candidate.center_y = candidate.front_y;
    candidate.center_z = candidate.front_z;
    candidate.size_x = max_x - min_x;
    candidate.size_y = max_y - min_y;
    candidate.size_z = max_z - min_z;
    candidate.point_count = static_cast<int>(front_indices.size());
    return candidate;
  }

  FaceRejectReason faceRejectReason(const FaceCandidate & candidate) const
  {
    if (candidate.point_count < min_face_points_) {
      return FaceRejectReason::kPoints;
    }
    if (candidate.size_y < min_face_width_y_ || candidate.size_y > max_face_width_y_) {
      return FaceRejectReason::kWidth;
    }
    if (candidate.size_z < min_face_height_z_ || candidate.size_z > max_face_height_z_) {
      return FaceRejectReason::kHeight;
    }
    if (candidate.size_x > max_face_depth_x_) {
      return FaceRejectReason::kDepth;
    }
    return FaceRejectReason::kNone;
  }

  static void countRejection(ExtractionStats * stats, FaceRejectReason reason)
  {
    if (!stats) {
      return;
    }
    switch (reason) {
      case FaceRejectReason::kPoints:
        stats->rejected_by_points++;
        break;
      case FaceRejectReason::kWidth:
        stats->rejected_by_width++;
        break;
      case FaceRejectReason::kHeight:
        stats->rejected_by_height++;
        break;
      case FaceRejectReason::kDepth:
        stats->rejected_by_depth++;
        break;
      case FaceRejectReason::kNone:
        break;
    }
  }

  std::vector<FaceCandidate> selectFirstRow(
    const std::vector<FaceCandidate> & candidates) const
  {
    if (candidates.empty()) {
      return {};
    }
    const double nearest_x = candidates.front().front_x;
    std::vector<FaceCandidate> first_row;
    for (const auto & candidate : candidates) {
      if (candidate.front_x <= nearest_x + near_row_depth_window_) {
        first_row.push_back(candidate);
      }
    }
    return first_row;
  }

  void assignColumns(std::vector<FaceCandidate> & candidates) const
  {
    std::sort(candidates.begin(), candidates.end(), [this](const auto & a, const auto & b) {
      if (sort_left_to_right_positive_y_) {
        return a.front_y > b.front_y;
      }
      return a.front_y < b.front_y;
    });
    if (static_cast<int>(candidates.size()) > expected_cols_) {
      candidates.resize(static_cast<size_t>(expected_cols_));
    }
    for (size_t i = 0; i < candidates.size(); ++i) {
      candidates[i].row = 0;
      candidates[i].col = static_cast<int>(i);
    }
  }

  double expectedColumnY(double first_col_y, int col) const
  {
    const double offset = static_cast<double>(col) * row_col_spacing_;
    return sort_left_to_right_positive_y_ ? first_col_y - offset : first_col_y + offset;
  }

  RowLayoutResult fitBestRowLayout(const std::vector<FaceCandidate> & candidates) const
  {
    RowLayoutResult best;
    if (candidates.empty()) {
      return best;
    }

    for (size_t anchor_index = 0; anchor_index < candidates.size(); ++anchor_index) {
      const auto & anchor = candidates[anchor_index];
      for (int anchor_col = 0; anchor_col < expected_cols_; ++anchor_col) {
        const double first_col_y = sort_left_to_right_positive_y_
          ? anchor.front_y + static_cast<double>(anchor_col) * row_col_spacing_
          : anchor.front_y - static_cast<double>(anchor_col) * row_col_spacing_;
        auto trial = evaluateRowLayout(candidates, first_col_y, anchor.front_x);
        if (!trial.valid) {
          continue;
        }
        if (!best.valid || isBetterRowLayout(trial, best)) {
          best = trial;
        }
      }
    }

    return best;
  }

  RowLayoutResult evaluateRowLayout(
    const std::vector<FaceCandidate> & candidates,
    double first_col_y,
    double anchor_x) const
  {
    std::vector<int> matched_indices(static_cast<size_t>(expected_cols_), -1);
    std::vector<bool> used(candidates.size(), false);
    double cost_sum = 0.0;
    int matched_count = 0;
    int total_points = 0;

    for (int col = 0; col < expected_cols_; ++col) {
      const double expected_y = expectedColumnY(first_col_y, col);
      int best_index = -1;
      double best_cost = std::numeric_limits<double>::infinity();

      for (size_t i = 0; i < candidates.size(); ++i) {
        if (used[i]) {
          continue;
        }
        const auto & candidate = candidates[i];
        const double y_error = std::abs(candidate.front_y - expected_y);
        if (y_error > row_spacing_tolerance_) {
          continue;
        }
        const double x_error = std::abs(candidate.front_x - anchor_x);
        if (x_error > row_x_tolerance_) {
          continue;
        }

        const double point_bonus =
          std::min(1.0, static_cast<double>(candidate.point_count) / 30.0) * 0.25;
        const double cost =
          y_error / row_spacing_tolerance_ + x_error / row_x_tolerance_ - point_bonus;
        if (cost < best_cost) {
          best_cost = cost;
          best_index = static_cast<int>(i);
        }
      }

      if (best_index >= 0) {
        matched_indices[static_cast<size_t>(col)] = best_index;
        used[static_cast<size_t>(best_index)] = true;
        cost_sum += best_cost;
        matched_count++;
        total_points += candidates[static_cast<size_t>(best_index)].point_count;
      }
    }

    RowLayoutResult result;
    if (matched_count < min_layout_matches_) {
      return result;
    }

    std::vector<double> matched_xs;
    double sum_z = 0.0;
    matched_xs.reserve(static_cast<size_t>(matched_count));
    for (const int index : matched_indices) {
      if (index < 0) {
        continue;
      }
      const auto & candidate = candidates[static_cast<size_t>(index)];
      matched_xs.push_back(candidate.front_x);
      sum_z += candidate.front_z;
    }

    const double row_front_x = quantile(matched_xs, 0.5);
    const double average_z = sum_z / static_cast<double>(matched_count);
    result.valid = true;
    result.matched_count = matched_count;
    result.front_x = row_front_x;
    result.score =
      static_cast<double>(matched_count) * 100.0 -
      (cost_sum / static_cast<double>(matched_count)) * 10.0 +
      std::min(50.0, static_cast<double>(total_points)) * 0.2;

    for (int col = 0; col < expected_cols_; ++col) {
      const int index = matched_indices[static_cast<size_t>(col)];
      if (index >= 0) {
        auto face = candidates[static_cast<size_t>(index)];
        face.row = 0;
        face.col = col;
        face.inferred = false;
        result.faces.push_back(face);
        continue;
      }
      if (!infer_missing_boxes_) {
        continue;
      }

      FaceCandidate inferred;
      inferred.row = 0;
      inferred.col = col;
      inferred.inferred = true;
      inferred.front_x = row_front_x;
      inferred.front_y = expectedColumnY(first_col_y, col);
      inferred.front_z = average_z;
      inferred.center_x = publish_box_center_ ? row_front_x + center_from_front_offset_ : row_front_x;
      inferred.center_y = inferred.front_y;
      inferred.center_z = average_z;
      inferred.size_x = 0.06;
      inferred.size_y = 0.25;
      inferred.size_z = 0.25;
      inferred.point_count = 0;
      result.faces.push_back(inferred);
    }

    std::sort(result.faces.begin(), result.faces.end(), [](const auto & a, const auto & b) {
      return a.col < b.col;
    });
    return result;
  }

  bool isBetterRowLayout(
    const RowLayoutResult & candidate,
    const RowLayoutResult & current_best) const
  {
    const double same_row_x_margin = std::max(0.12, row_x_tolerance_ * 0.5);
    if (candidate.front_x < current_best.front_x - same_row_x_margin) {
      return true;
    }
    if (candidate.front_x > current_best.front_x + same_row_x_margin) {
      return false;
    }
    if (candidate.matched_count != current_best.matched_count) {
      return candidate.matched_count > current_best.matched_count;
    }
    return candidate.score > current_best.score;
  }

  TemplateMatch fitTemplateRow(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
  {
    std::vector<double> front_x_samples;
    front_x_samples.reserve(cloud->points.size());
    for (const auto & point : cloud->points) {
      if (point.z < profile_z_min_ || point.z > profile_z_max_) {
        continue;
      }
      if (point.x < template_search_x_min_ || point.x > template_search_x_max_) {
        continue;
      }
      front_x_samples.push_back(point.x);
    }
    if (front_x_samples.empty()) {
      return {};
    }

    std::sort(front_x_samples.begin(), front_x_samples.end());
    const double front_x_start =
      std::max(template_search_x_min_, front_x_samples.front() - template_front_margin_);
    const double front_x_end =
      std::min(template_search_x_max_, front_x_samples.back() + template_front_margin_);

    TemplateMatch best;
    const double x_step = 0.05;
    const double y_step = std::max(0.05, row_col_spacing_ * 0.1);
    const double max_y_span = row_col_spacing_ * static_cast<double>(expected_cols_ - 1);

    for (double front_x = front_x_start; front_x <= front_x_end; front_x += x_step) {
      const double min_first_col_y = y_min_ + max_y_span;
      const double max_first_col_y = y_max_;
      for (double first_col_y = min_first_col_y; first_col_y <= max_first_col_y;
           first_col_y += y_step) {
        auto match = evaluateTemplateRow(cloud, front_x, first_col_y);
        if (!match.valid) {
          continue;
        }
        if (!best.valid || isBetterTemplateMatch(match, best)) {
          best = match;
        }
      }
    }

    return best;
  }

  TemplateMatch evaluateTemplateRow(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    double front_x,
    double first_col_y) const
  {
    TemplateMatch result;
    result.front_x = front_x;
    result.first_col_y = first_col_y;

    std::vector<std::vector<int>> box_indices(static_cast<size_t>(expected_cols_));
    std::vector<double> nearest_x(static_cast<size_t>(expected_cols_), std::numeric_limits<double>::infinity());
    std::vector<double> min_z(static_cast<size_t>(expected_cols_), std::numeric_limits<double>::infinity());
    std::vector<double> max_z(static_cast<size_t>(expected_cols_), -std::numeric_limits<double>::infinity());
    std::vector<int> upper_points(static_cast<size_t>(expected_cols_), 0);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
      const auto & point = cloud->points[i];
      if (point.z < profile_z_min_ || point.z > profile_z_max_) {
        continue;
      }
      if (point.x < front_x - template_front_margin_ || point.x > front_x + template_depth_) {
        continue;
      }

      for (int col = 0; col < expected_cols_; ++col) {
        const double expected_y = expectedColumnY(first_col_y, col);
        if (std::abs(point.y - expected_y) > template_y_half_width_) {
          continue;
        }
        box_indices[static_cast<size_t>(col)].push_back(static_cast<int>(i));
        nearest_x[static_cast<size_t>(col)] =
          std::min(nearest_x[static_cast<size_t>(col)], static_cast<double>(point.x));
        min_z[static_cast<size_t>(col)] =
          std::min(min_z[static_cast<size_t>(col)], static_cast<double>(point.z));
        max_z[static_cast<size_t>(col)] =
          std::max(max_z[static_cast<size_t>(col)], static_cast<double>(point.z));
        if (point.z >= template_upper_z_min_) {
          upper_points[static_cast<size_t>(col)]++;
        }
        break;
      }
    }

    int matched_boxes = 0;
    int total_points = 0;
    double x_error_sum = 0.0;
    for (int col = 0; col < expected_cols_; ++col) {
      const int points = static_cast<int>(box_indices[static_cast<size_t>(col)].size());
      const double z_span =
        std::isfinite(min_z[static_cast<size_t>(col)]) ?
        max_z[static_cast<size_t>(col)] - min_z[static_cast<size_t>(col)] : 0.0;
      total_points += points;
      if (points >= template_min_box_points_ &&
          z_span >= template_min_z_span_ &&
          upper_points[static_cast<size_t>(col)] >= template_min_upper_points_ &&
          std::abs(nearest_x[static_cast<size_t>(col)] - front_x) <= template_x_tolerance_) {
        matched_boxes++;
        x_error_sum += std::abs(nearest_x[static_cast<size_t>(col)] - front_x);
      }
    }

    if (matched_boxes < min_layout_matches_ || total_points < template_min_total_points_) {
      return result;
    }

    result.valid = true;
    result.matched_boxes = matched_boxes;
    result.total_points = total_points;
    result.score =
      static_cast<double>(matched_boxes) * 1000.0 +
      static_cast<double>(total_points) -
      x_error_sum * 20.0 -
      front_x * 2.0;

    for (int col = 0; col < expected_cols_; ++col) {
      FaceCandidate face = makeTemplateFace(
        cloud, box_indices[static_cast<size_t>(col)], front_x, expectedColumnY(first_col_y, col),
        col);
      result.faces.push_back(face);
    }
    return result;
  }

  FaceCandidate makeTemplateFace(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::vector<int> & indices,
    double template_front_x,
    double template_y,
    int col) const
  {
    FaceCandidate candidate;
    candidate.row = 0;
    candidate.col = col;
    candidate.front_x = template_front_x;
    candidate.front_y = template_y;
    candidate.front_z = (profile_z_min_ + profile_z_max_) * 0.5;
    candidate.center_x =
      publish_box_center_ ? template_front_x + center_from_front_offset_ : template_front_x;
    candidate.center_y = template_y;
    candidate.center_z = candidate.front_z;
    candidate.size_x = 0.08;
    candidate.size_y = std::min(0.45, template_y_half_width_ * 2.0);
    candidate.size_z = std::max(0.08, profile_z_max_ - profile_z_min_);
    candidate.point_count = static_cast<int>(indices.size());
    candidate.inferred = candidate.point_count < template_min_box_points_;

    if (indices.empty()) {
      return candidate;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    std::vector<double> xs;
    xs.reserve(indices.size());

    for (const int index : indices) {
      const auto & point = cloud->points[static_cast<size_t>(index)];
      xs.push_back(point.x);
      sum_x += point.x;
      sum_y += point.y;
      sum_z += point.z;
      min_x = std::min(min_x, static_cast<double>(point.x));
      max_x = std::max(max_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_y = std::max(max_y, static_cast<double>(point.y));
      min_z = std::min(min_z, static_cast<double>(point.z));
      max_z = std::max(max_z, static_cast<double>(point.z));
    }

    const double inv_count = 1.0 / static_cast<double>(indices.size());
    candidate.front_x = quantile(xs, front_x_quantile_);
    candidate.front_y = sum_y * inv_count;
    candidate.front_z = sum_z * inv_count;
    candidate.center_x =
      publish_box_center_ ? candidate.front_x + center_from_front_offset_ : candidate.front_x;
    candidate.center_y = candidate.front_y;
    candidate.center_z = candidate.front_z;
    candidate.size_x = std::max(0.04, std::min(0.20, max_x - min_x));
    candidate.size_y = std::max(0.08, std::min(0.45, max_y - min_y));
    candidate.size_z = std::max(0.08, std::min(0.35, max_z - min_z));
    return candidate;
  }

  bool isBetterTemplateMatch(
    const TemplateMatch & candidate,
    const TemplateMatch & current_best) const
  {
    const double x_margin = 0.20;
    if (candidate.matched_boxes != current_best.matched_boxes) {
      return candidate.matched_boxes > current_best.matched_boxes;
    }
    if (candidate.front_x < current_best.front_x - x_margin) {
      return true;
    }
    if (candidate.front_x > current_best.front_x + x_margin) {
      return false;
    }
    return candidate.score > current_best.score;
  }

  void publishDebugCloud(
    const std_msgs::msg::Header & header,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
  {
    sensor_msgs::msg::PointCloud2 debug_msg;
    pcl::toROSMsg(*cloud, debug_msg);
    debug_msg.header = header;
    debug_cloud_pub_->publish(debug_msg);
  }

  void publishMarkersAndTarget(
    const std_msgs::msg::Header & cloud_header,
    const std::vector<FaceCandidate> & candidates)
  {
    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = now();
    const std::string frame_id =
      target_frame_override_.empty() ? cloud_header.frame_id : target_frame_override_;
    const int target_row = target_row_.load();
    const int target_col = target_col_.load();

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = frame_id;
    clear.header.stamp = stamp;
    clear.ns = "box_faces";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int marker_id = 1;
    for (const auto & candidate : candidates) {
      const bool is_target = candidate.row == target_row && candidate.col == target_col;
      markers.markers.push_back(makeFaceMarker(frame_id, stamp, marker_id++, candidate, is_target));
      markers.markers.push_back(makeTextMarker(frame_id, stamp, marker_id++, candidate));
      if (is_target) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = stamp;
        pose.pose.position.x = candidate.center_x;
        pose.pose.position.y = candidate.center_y;
        pose.pose.position.z = candidate.center_z;
        pose.pose.orientation.w = 1.0;
        target_pose_pub_->publish(pose);
      }
    }
    marker_pub_->publish(markers);
  }

  visualization_msgs::msg::Marker makeFaceMarker(
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    int marker_id,
    const FaceCandidate & candidate,
    bool is_target) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "box_faces";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = candidate.center_x;
    marker.pose.position.y = candidate.center_y;
    marker.pose.position.z = candidate.center_z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.04, std::min(0.10, candidate.size_x));
    marker.scale.y = std::max(0.08, std::min(0.45, candidate.size_y));
    marker.scale.z = std::max(0.08, std::min(0.35, candidate.size_z));
    if (is_target) {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = candidate.inferred ? 0.55 : 0.9;
    } else if (candidate.inferred) {
      marker.color.r = 0.2;
      marker.color.g = 0.35;
      marker.color.b = 1.0;
      marker.color.a = 0.35;
    } else {
      marker.color.r = 0.1;
      marker.color.g = 0.8;
      marker.color.b = 0.7;
      marker.color.a = 0.65;
    }
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
    return marker;
  }

  visualization_msgs::msg::Marker makeTextMarker(
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    int marker_id,
    const FaceCandidate & candidate) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "box_face_labels";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = candidate.center_x;
    marker.pose.position.y = candidate.center_y;
    marker.pose.position.z = candidate.center_z + 0.18;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.14;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.95;
    std::ostringstream label;
    label << "(" << candidate.row << "," << candidate.col << ")";
    if (candidate.inferred) {
      label << "*";
    }
    marker.text = label.str();
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
    return marker;
  }

  void publishStatus(
    uint64_t sequence,
    const std::string & frame_id,
    size_t raw_points,
    size_t roi_points,
    size_t self_filtered_points,
    size_t working_points,
    size_t accepted_face_count,
    const ExtractionStats & extraction_stats,
    const RowLayoutResult & row_layout,
    const std::vector<FaceCandidate> & first_row_faces)
  {
    std::ostringstream stream;
    stream
      << "seq=" << sequence
      << ", frame=" << frame_id
      << ", raw=" << raw_points
      << ", roi=" << roi_points
      << ", self_filtered=" << self_filtered_points
      << ", working=" << working_points
      << ", mode=" << extraction_stats.mode
      << ", yz_cells=" << extraction_stats.yz_cells
      << ", components=" << extraction_stats.components
      << ", accepted_faces=" << accepted_face_count
      << ", rejected(points,width,height,depth)=("
      << extraction_stats.rejected_by_points << ","
      << extraction_stats.rejected_by_width << ","
      << extraction_stats.rejected_by_height << ","
      << extraction_stats.rejected_by_depth << ")"
      << ", layout_valid=" << (row_layout.valid ? "true" : "false")
      << ", layout_matches=" << row_layout.matched_count
      << ", layout_front_x=" << row_layout.front_x
      << ", layout_score=" << row_layout.score
      << ", first_row_faces=" << first_row_faces.size()
      << ", target=(" << target_row_.load() << "," << target_col_.load() << ")";
    if (status_include_faces_ && !first_row_faces.empty()) {
      stream << ", faces=[";
      for (size_t i = 0; i < first_row_faces.size(); ++i) {
        const auto & face = first_row_faces[i];
        if (i > 0) {
          stream << "; ";
        }
        stream
          << "(" << face.row << "," << face.col << ")"
          << " front=(" << face.front_x << "," << face.front_y << "," << face.front_z << ")"
          << " center=(" << face.center_x << "," << face.center_y << "," << face.center_z << ")"
          << " size=(" << face.size_x << "," << face.size_y << "," << face.size_z << ")"
          << " pts=" << face.point_count;
      }
      stream << "]";
    }

    std_msgs::msg::String status;
    status.data = stream.str();
    status_pub_->publish(status);
    if (enable_diagnostics_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", status.data.c_str());
    }
  }

  std::string cloud_topic_;
  std::string debug_cloud_topic_;
  std::string marker_topic_;
  std::string target_id_topic_;
  std::string target_pose_topic_;
  std::string status_topic_;
  std::atomic<int> target_row_{0};
  std::atomic<int> target_col_{0};

  double x_min_ = 0.30;
  double x_max_ = 4.50;
  double y_min_ = -3.00;
  double y_max_ = 3.00;
  double z_min_ = -0.32;
  double z_max_ = 0.05;
  double self_filter_x_max_ = 0.60;
  double self_filter_y_abs_max_ = 0.60;
  double voxel_leaf_size_ = 0.015;

  std::string candidate_extraction_mode_ = "front_profile";
  double yz_cell_y_size_ = 0.035;
  double yz_cell_z_size_ = 0.035;
  int yz_connect_y_gap_cells_ = 2;
  int yz_connect_z_gap_cells_ = 2;
  double yz_connect_x_tolerance_ = 0.30;
  double front_x_quantile_ = 0.20;
  double front_face_depth_max_ = 0.30;
  double profile_z_min_ = -0.24;
  double profile_z_max_ = 0.03;
  double profile_y_bin_size_ = 0.05;
  int profile_min_bin_points_ = 2;
  int profile_connect_gap_bins_ = 2;
  double profile_connect_x_tolerance_ = 0.35;
  int profile_min_segment_bins_ = 2;
  double profile_box_width_ = 0.36;
  double profile_candidate_x_tolerance_ = 0.35;
  double profile_nms_y_tolerance_ = 0.30;
  double template_search_x_min_ = 0.80;
  double template_search_x_max_ = 4.50;
  double template_y_half_width_ = 0.22;
  double template_front_margin_ = 0.08;
  double template_depth_ = 0.42;
  double template_x_tolerance_ = 0.40;
  int template_min_box_points_ = 5;
  int template_min_total_points_ = 18;
  double template_min_z_span_ = 0.12;
  double template_upper_z_min_ = -0.18;
  int template_min_upper_points_ = 2;

  int min_face_points_ = 3;
  double min_face_width_y_ = 0.03;
  double max_face_width_y_ = 0.45;
  double min_face_height_z_ = 0.03;
  double max_face_height_z_ = 0.35;
  double max_face_depth_x_ = 0.45;
  double near_row_depth_window_ = 0.45;

  double center_from_front_offset_ = 0.125;
  bool publish_box_center_ = true;
  int expected_cols_ = 4;
  bool sort_left_to_right_positive_y_ = true;
  bool use_row_layout_filter_ = true;
  double row_col_spacing_ = 0.85;
  double row_spacing_tolerance_ = 0.25;
  double row_x_tolerance_ = 0.45;
  int min_layout_matches_ = 3;
  bool infer_missing_boxes_ = true;

  double processing_period_ = 0.20;
  double marker_lifetime_ = 1.00;
  std::string target_frame_override_;
  bool enable_diagnostics_ = true;
  bool status_include_faces_ = false;

  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr processing_callback_group_;
  rclcpp::CallbackGroup::SharedPtr target_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr target_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr processing_timer_;

  std::mutex latest_cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_msg_;
  uint64_t latest_cloud_sequence_ = 0;
  uint64_t processed_cloud_sequence_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<BoxGridLocatorNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
