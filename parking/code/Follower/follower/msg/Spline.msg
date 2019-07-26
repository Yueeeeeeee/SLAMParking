# A single bezier spline.

geometry_msgs/Point start       # Start point of this spline.
geometry_msgs/Point waypoint1   # Waypoint 1. The spline starts in its direction but won't pass through it.
geometry_msgs/Point waypoint2   # Waypoint 2. The spline arrives a the end from its direction but won't pass through it.
geometry_msgs/Point end         # The spline ends here.
bool                backward    # true if this spline should be travelled backward.
                                # The start is still the starting point and the end still the final goal but the robot will drive backward instead of forward.