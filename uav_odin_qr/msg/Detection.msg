# UTF-8 QR content and mean of four corners in the original undistorted image.
string content
float64 center_u
float64 center_v

# OYO camera forward Z in meters. When invalid, depth and all XYZ are NaN.
bool depth_valid
float64 depth_m
geometry_msgs/PointStamped lidar_point
string status
# OYO image timestamp minus depth timestamp, in seconds. NaN when invalid.
float64 sync_dt
