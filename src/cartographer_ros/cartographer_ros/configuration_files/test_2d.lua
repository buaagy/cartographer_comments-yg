include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  -- 是否提供里程计数据
  provide_odom_frame = false,
  -- false改为true,仅发布2D位姿
  publish_frame_projected_to_2d = true,
  -- 是否发布位姿外推器计算的位姿,还是发布前端计算的位姿
  use_pose_extrapolator = false,
  -- 是否使用里程计数据
  use_odometry = false,
  -- 是否使用GPS数据
  use_nav_sat = false,
  -- 是否使用路标
  use_landmarks = false,
  -- 使用雷达的个数
  num_laser_scans = 0,
  -- 使用多波雷达的个数
  num_multi_echo_laser_scans = 0,
  -- 10改为1,1/1=1等于不对点云数据进行分割处理
  num_subdivisions_per_laser_scan = 1,
  -- 使用3D点云的个数
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 是否启动2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- trajectory_builder_2d相关参数
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0
-- 5改成3,激光雷达缺失数据的填充值
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- 是否使用实时相关性扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
-- 1.0改成0.1,提高对运动的敏感度
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- 0.55改成0.65,Fast csm的最低分数,高于此分数才进行优化
POSE_GRAPH.constraint_builder.min_score = 0.65
-- 0.6改成0.7,全局定位最小分数,低于此分数则认为目前全局定位不准确
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.optimization_problem.huber_scale = 1e2
-- 默认值为50,设置为0可关闭全局SLAM
POSE_GRAPH.optimize_every_n_nodes = 50

return options
