include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- 跟踪和发布的frameID都改成激光雷达的frameID
  tracking_frame = "footprint",
  published_frame = "footprint",
  odom_frame = "odom",
  -- true改为false,不用提供里程计数据
  provide_odom_frame = false,
  -- false改为true,仅发布2D位姿
  publish_frame_projected_to_2d = true,
  -- 是否使用位姿外推器
  use_pose_extrapolator = true,
  -- 是否使用里程计数据
  use_odometry = false,
  use_nav_sat = false,
  -- 是否使用路标
  use_landmarks = false,
  -- 0改为1,使用一个雷达
  num_laser_scans = 1,
  -- 1改为0,不使用多波雷达
  num_multi_echo_laser_scans = 0,
  -- 10改为1,1/1=1等于不分割
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
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

-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 3,
-- }
-- 0改成0.10,比机器人半径小的都可以忽略
TRAJECTORY_BUILDER_2D.min_range = 0.10
-- 30改成10.0,限制在激光雷达最大扫描范围内,越小一般越精确些
TRAJECTORY_BUILDER_2D.max_range = 10.0
-- 5改成3,激光雷达缺失数据的填充值
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- 是否使用IMU数据
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 是否使用实时相关性扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
-- 1.0改成0.1,提高对运动的敏感度
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- 0.55改成0.65,Fast csm的最低分数,高于此分数才进行优化
POSE_GRAPH.constraint_builder.min_score = 0.65
-- 0.6改成0.7,全局定位最小分数,低于此分数则认为目前全局定位不准确
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.optimization_problem.huber_scale = 1e2
-- 默认值为50,设置为0可关闭全局SLAM
POSE_GRAPH.optimize_every_n_nodes = 50
return options
