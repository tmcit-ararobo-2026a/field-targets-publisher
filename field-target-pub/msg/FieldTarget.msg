uint8 START_ZONE=0
uint8 FIXED_BUCKET1=1
uint8 FIXED_BUCKET2=2
uint8 FIXED_BUCKET3=3
uint8 DESK1=4
uint8 DESK2=5
uint8 FLAG=6
uint8 MOVING_BUCKET=7
uint8 target_id
string label
geometry_msgs/PointStamped aim_point
geometry_msgs/PoseStamped stop_pose
bool aim_valid
bool stop_pose_valid
# Only approved integer degree values. No invalid sentinel is defined.
bool angle_valid
int8 angle_deg
