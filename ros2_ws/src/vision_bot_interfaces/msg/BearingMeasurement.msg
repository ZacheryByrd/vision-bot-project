# One sighting of the drone by one robot's sky camera: the direction from the
# camera toward the drone, in the map frame (spec section 3).
std_msgs/Header header          # image stamp; frame_id is "map"
string robot                    # robot1 or robot2
geometry_msgs/Point origin      # camera position in the map frame at the image stamp
geometry_msgs/Vector3 direction # unit vector from the camera toward the drone
float64 sigma_rad               # one-sigma angular noise (pixel noise + robot localization)
float64 blob_area_px            # detected blob size, for diagnostics
