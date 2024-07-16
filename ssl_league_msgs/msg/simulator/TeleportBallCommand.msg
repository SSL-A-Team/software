# Teleport the ball to a new location and optionally set it to some velocity
geometry_msgs/Pose pose
geometry_msgs/Twist twist

# Teleport the ball safely to the target, for example by
# moving robots out of the way in case of collision and set speed of robots close-by to zero
bool teleport_safely

# Adapt the angular ball velocity such that the ball is rolling
bool roll

# Instead of teleporting the ball, apply some force to make sure
# the ball reaches the required position soon (velocity is ignored if true)
# WARNING: A command with by_force stays active (the move will take some time)
# until cancled by another TeleportBall command with by_force = false.
# To avoid teleporting the ball at the end and resetting its current spin,
# do not set any of the optional fields in this message to end the force without triggering
# an additional teleportation
bool by_force