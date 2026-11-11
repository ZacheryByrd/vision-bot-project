# The fused drone track from drone_tracker (spec section 3).
uint8 TENTATIVE=1
uint8 CONFIRMED=2
uint8 COASTING=3
uint8 LOST=4

std_msgs/Header header          # frame_id is "map"
uint8 status
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64[36] covariance          # 6x6 of (x, y, z, vx, vy, vz), row-major
string[] robots                 # robots whose bearings updated the track in the last second
float64 seconds_since_update    # time since the last accepted bearing
uint32 rejected                 # bearings rejected by the gate since the track started
