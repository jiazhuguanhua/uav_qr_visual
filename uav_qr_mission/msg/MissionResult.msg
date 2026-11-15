string mission_id
string status
string[] target_ids
# Aligned with target_ids. A recorded detection stays found for this mission.
bool[] found
string position_target_id
# The first valid position transformed to the configured result frame.
bool position_valid
geometry_msgs/PointStamped position
# Current observation of the selected QR, using the fixed TF child name.
bool report_tf_valid
geometry_msgs/TransformStamped report_tf
bool complete
