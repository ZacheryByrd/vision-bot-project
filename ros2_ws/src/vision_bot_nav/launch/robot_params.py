"""Per-robot Nav2 parameters, derived from config/nav2_params.yaml.

Used by nav2.launch.py; pure (no ROS) so it is unit tested in
test/test_robot_params.py. Does what the installed nav2_bringup does with
RewrittenYaml(root_key=namespace) and its per-robot params files, but in one
place:

- nests every node's parameters under the namespace, so /robot1/amcl etc.
  match;
- makes the costmaps' scan topics absolute per robot (/robot1/scan): a
  relative topic would resolve under the costmap's own namespace
  (/robot1/local_costmap/scan), which is why the installed multi-robot
  params also spell out /robot1/scan;
- sets this robot's AMCL initial pose and the pose topics its
  RobotAvoidanceLayer watches (the other robots').
"""

import copy

COSTMAPS = ("local_costmap", "global_costmap")


def robot_params(base, namespace, initial_pose, other_pose_topics):
    """New params dict for one robot; base is the loaded nav2_params.yaml.

    initial_pose is (x, y, yaw) in the map frame; namespace "" means none.
    """
    params = copy.deepcopy(base)
    x, y, yaw = initial_pose
    params["amcl"]["ros__parameters"]["initial_pose"] = {
        "x": float(x), "y": float(y), "z": 0.0, "yaw": float(yaw)}
    for name in COSTMAPS:
        node = params[name][name]["ros__parameters"]
        node["robot_avoidance_layer"]["robot_pose_topics"] = list(other_pose_topics)
        if namespace:
            node["obstacle_layer"]["scan"]["topic"] = f"/{namespace}/scan"
    return {namespace: params} if namespace else params
