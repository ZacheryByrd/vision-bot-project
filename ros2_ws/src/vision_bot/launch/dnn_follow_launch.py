"""
dnn_follow_launch.py
=====================
The DNN-detection variant of sim_launch.py: same nodes, same topics,
same approach-and-stop control behavior as the red-box demo -- the only
thing that changes is perception_node's detector_backend, from HSV color
threshold to a pretrained MobileNet-SSD (OpenCV's cv2.dnn) recognizing
an actual "person" class rather than a hand-picked color range. See
perception_node.py's dnn_detect_target() and module docstring.

Run with:
    ros2 launch vision_bot dnn_follow_launch.py

Why the target is a flat photo, not a 3D model
-----------------------------------------------
The obvious approach -- spawn a humanoid-shaped Gazebo model built from
primitives (box torso, sphere head, cylinder limbs) -- does NOT work.
Tested it: a real-world-trained detector gets 0.0 confidence against
flat-shaded synthetic geometry, for every one of its 20 known classes.
It has never seen anything like it; there's no texture, no lighting
detail, none of the pixel statistics a CNN trained on photographs
actually keys on. Shape alone isn't enough.

What does work: crop a real person out of a real photograph and texture
that image onto a flat plane in the world (worlds/models/person_photo_model).
Same detector, same synthetic render pipeline, same lighting -- just real
photo pixels instead of flat colors -- and confidence jumps to ~0.8.
That model file is checked in at worlds/models/person_photo_model/materials/.

Why the world is generated at launch time, not a static .world file
----------------------------------------------------------------------
worlds/person_track.world.in is a template with a {MODEL_DIR} placeholder,
filled in here with the actual installed path to
worlds/models/person_photo_model. Two things forced this instead of a
plain checked-in .world file:
    1. That path varies by machine/install location, so it can't be a
       fixed string in a file committed to git.
    2. Gazebo Classic's <include><uri>model://name</uri></include> tries
       an online model database (http://models.gazebosim.org) as part of
       resolving it -- confirmed hanging indefinitely (90s+, no response)
       during development, presumably because that legacy service no
       longer works properly. Using a direct absolute path in <uri> (no
       model:// scheme) skips that lookup and resolves instantly.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory("vision_bot")
    model_dir = os.path.join(pkg_share, "worlds", "models", "person_photo_model")

    with open(os.path.join(pkg_share, "worlds", "person_track.world.in")) as f:
        world_content = f.read().replace("{MODEL_DIR}", model_dir)

    generated_world_fd, generated_world_path = tempfile.mkstemp(
        prefix="vision_bot_person_track_", suffix=".world"
    )
    with os.fdopen(generated_world_fd, "w") as f:
        f.write(world_content)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sim_launch.py")
        ),
        launch_arguments={
            "world": generated_world_path,
            "detector_backend": "dnn",
            "dnn_prototxt_path": os.path.join(pkg_share, "models", "dnn", "MobileNetSSD_deploy.prototxt"),
            "dnn_model_path": os.path.join(pkg_share, "models", "dnn", "MobileNetSSD_deploy.caffemodel"),
            "dnn_target_class_id": "15",   # "person" in the VOC 20-class set
            "dnn_confidence_threshold": "0.5",
            # Same approach-and-stop control law as the red-box demo
            # (sim_launch.py's stop_area_fraction/angular_gain/base_linear_speed
            # defaults) -- a person is a discrete target, same as the box,
            # not a continuous path like the line-following demo.
            "spawn_x": "-2.0",
            "spawn_y": "0.0",
            "spawn_yaw": "0.0",
        }.items(),
    )

    return LaunchDescription([sim])
