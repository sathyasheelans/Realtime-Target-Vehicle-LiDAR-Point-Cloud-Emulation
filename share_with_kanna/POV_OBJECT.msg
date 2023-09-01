# msg definition for a single POV object

Header header

# POV id
int32 id

# POV dimensions
float64 scale_x
float64 scale_y
float64 scale_z

# the grid-based zone
#   0-7 left-to-right, front-to-rear
uint8 zone

# dynamical states
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
sensor_msgs/NavSatFix coordinate
float64 speed