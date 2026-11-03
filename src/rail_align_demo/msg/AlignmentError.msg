# Camera-derived vehicle pose error in the rail frame. See README for sign rules.
Header header                  # stamp of the source image

bool valid                     # true only when both markers were detected and the pose passed all gates
string reason                  # why the estimate is invalid; empty when valid
int32[] detected_ids

float64 lateral_error_m        # + : vehicle is left of the rail center line
float64 heading_error_deg      # + : vehicle is rotated CCW (nose left) from the rail direction
float64 longitudinal_error_m   # + : stop target is still ahead of the vehicle

float64 reprojection_rms_px
geometry_msgs/Pose left_marker_in_camera    # per-marker pose in the optical frame
geometry_msgs/Pose right_marker_in_camera
