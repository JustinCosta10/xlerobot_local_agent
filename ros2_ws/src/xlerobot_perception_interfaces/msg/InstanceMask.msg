# ID is local to this image, not a persistent object or class ID.
uint32 instance_id
string class_name
float32 confidence
# Original RGB pixel coordinates of this cropped, lossless mono8 mask.
uint32 x_offset
uint32 y_offset
# 0 = outside, 255 = inside. Header matches the source RGB capture.
sensor_msgs/Image mask
