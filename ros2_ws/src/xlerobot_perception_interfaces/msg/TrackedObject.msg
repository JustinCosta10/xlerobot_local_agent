# UUIDs are distinct from YOLO class IDs and per-image instance IDs.
string object_id
string session_id
string label
# Frame is the session's map frame; stamp is the last observation's capture time.
std_msgs/Header header
geometry_msgs/Point position
# Extent and spread of observed surface points, not a full object model or
# calibrated pose uncertainty. Position is not a grasp pose or true object center.
geometry_msgs/Vector3 extent
geometry_msgs/Vector3 surface_spread
float32 confidence
float32 valid_depth_fraction
uint32 point_count
uint32 observation_count
builtin_interfaces/Time first_seen
bool stale
# Populated only when requested through QueryObjects.
sensor_msgs/CompressedImage crop
