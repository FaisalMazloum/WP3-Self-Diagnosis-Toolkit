# Status broadcast by every robot on the shared /echi_status topic.
# Every robot publishes its own and subscribes to everyone's.

std_msgs/Header header   # stamp = time of the GNSS fix
string id                # robot id / namespace, e.g. "remora3"

# WGS84 position straight from the RTK GNSS (sensor_msgs/NavSatFix)
float64 latitude
float64 longitude
float64 altitude
