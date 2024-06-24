# All units in meters

# Type of overlay
uint8 POINT=0
uint8 LINE=1
uint8 RECTANGLE=2
uint8 ELLIPSE=3
uint8 POLYGON=4
uint8 TEXT=5
uint8 MESH=6
uint8 CUSTOM=7
uint8 ARC=8

# Type of Command
uint8 REPLACE=0 # Replaces overlay with matching name or creates it if none exist
uint8 EDIT=1 # Edits the overlay with matching name
uint8 REMOVE=2 # Removes the overlay with matching name

# Namespace for the overlay, preferably sending node's name prepended with its namespace to help prevent collisions
string ns

# All overlays are named so we can keep track of them
string name

# Determines if the overlay is visible in the ui
bool visible

# Specifies the type of overlay to use
uint8 type

# Specifies the command to be used
uint8 command

# Center position specified as (X, Y, rotation around center in degrees)
geometry_msgs/Point position

# Scale specified as (Width, Height, _)
geometry_msgs/Point scale

# Colors are specified using "#RRGGBBAA" hexadecimal RGBA format. The # is required
string stroke_color
# Fill color is used for text color when using the TEXT type
string fill_color

# Also used as font size when using the TEXT type
uint8 stroke_width 1

# lifetime of overlay in milliseconds, 0 for infinite
uint32 lifetime 0

# List of points for the LINE and POLYGON types (relative to the center position point)
geometry_msgs/Point[] points

# when using the MESH type specify the boundaries of the mesh as a rectangle using the position and scale values
ateam_msgs/Mesh1d[] mesh
ateam_msgs/Mesh1d[] mesh_alpha

# When using the TEXT type place the text here, when using the CUSTOM type put arbitrary javascript in this variable
string text

# 0 is background, upwards
uint32 depth

# arc angles
float64 start_angle
float64 end_angle
