# Source image header. Each transform has its own camera frame and depth stamp.
std_msgs/Header header
# Valid camera-to-QR transforms from this processed frame only.
uav_odin_qr/QrTransform[] targets
