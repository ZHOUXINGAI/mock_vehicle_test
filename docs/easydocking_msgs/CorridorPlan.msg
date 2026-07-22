# CorridorPlan – 双向走廊调度计划
# 由 docking_controller 计算，carrier 和 mini 各取所需

std_msgs/Header header

# ── rendezvous 点（mini 到达此相位时切 glide）──
float64 rendezvous_x
float64 rendezvous_y
float64 rendezvous_z

# ── 切线方向（单位向量，与 mini 轨道速度同向）──
float64 tangent_dir_x
float64 tangent_dir_y

# ── 走廊尺寸 ──
float64 corridor_length     # 总长，默认 60m
float64 ahead_distance      # rendezvous 前方的 carrier 等候距离，默认 15m

# ── 时间 ──
float64 mini_arrival_time   # mini 预计到达 rendezvous 相位的绝对时间 (ROS time, sec)

# ── carrier 目标 ──
float64 carrier_start_x     # = rendezvous - tangent × ahead_distance（carrier 先飞这里）
float64 carrier_start_y
float64 carrier_target_x    # = rendezvous + tangent × corridor_length（走廊远端）
float64 carrier_target_y

# ── mini 触发门限 ──
float64 mini_orbit_phase_trigger_rad  # mini 轨道到达此相位时切 glide

# ── 有效性 ──
bool corridor_valid          # false 表示未找到有效走廊，需 fallback
int32 plan_id               # 计划 ID（每次走廊更新递增，用于去重）
