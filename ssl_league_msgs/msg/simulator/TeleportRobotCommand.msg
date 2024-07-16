# Teleport a robot to some location and give it a velocity
ssl_league_msgs/RobotId id

geometry_msgs/Pose pose
geometry_msgs/Twist twist

# Robot should be present on the field?
# true -> robot will be added, if it does not exist yet
# false -> robot will be removed, if it is present
bool present

# Instead of teleporting, apply some force to make sure
# the robot reaches the required position soon (velocity is ignored if true)
# WARNING: A command with by_force stays active (the move will take some time)
# until cancled by another TeleportRobot command for the same bot with by_force = false.
# To avoid teleporting at the end,
# do not set any of the optional fields in this message
# to end the force without triggering
# an additional teleportation
bool by_force