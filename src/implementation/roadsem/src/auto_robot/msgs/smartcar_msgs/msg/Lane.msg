Header header
int32 lane_id
Waypoint[] waypoints

float64 cost
float64 center_cost
float64 transition_cost
float64 lateral_cost
float64 long_cost

float64 closest_obj_dis
bool is_blocked
int32 best_rollout_index

bool  is_dual_control_enabled  # 是否开启双阿克曼车型的双向控制
bool  is_dual_control_front    # true表示向前控制
