// Types based on ros message types

interface Point {
    x: number
    y: number
    z: number
}

interface Pose {
    position: ROSLIB.Vector3
    orientation: ROSLIB.Quaternion
}

interface Twist {
    linear: ROSLIB.Vector3
    angular: ROSLIB.Vector3
}

interface Accel {
    linear: ROSLIB.Vector3
    angular: ROSLIB.Vector3
}