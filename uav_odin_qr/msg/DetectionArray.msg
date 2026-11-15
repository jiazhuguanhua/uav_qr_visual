# stamp and frame_id describe the source image; frame_id may be empty.
std_msgs/Header header
uint32 image_width
uint32 image_height
# Empty if the processed image has no decodable QR code.
uav_odin_qr/Detection[] detections
