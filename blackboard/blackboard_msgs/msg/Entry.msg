uint8 BOOL_TYPE=0
uint8 STRING_TYPE=1
uint8 FLOAT_TYPE=2
uint8 TRANSFORM_TYPE=3
uint8 OCTOMAP_TYPE=4
uint8 POSE_TYPE=5

uint8 type
string key

bool bool_entry
string string_entry
float32 float_entry
geometry_msgs/TransformStamped transform_entry
octomap_msgs/Octomap octomap_entry
geometry_msgs/PoseStamped pose_entry
