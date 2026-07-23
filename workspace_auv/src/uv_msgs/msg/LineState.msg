# Compact result of a YOLO-Seg line estimate.

builtin_interfaces/Time stamp
string camera_name
bool detected
float32 center_error       # scanline center error relative to image center, [-1, 1]
float32 heading_error_deg  # image-only line angle relative to vertical; debug only
float32 area_ratio         # segmentation mask area divided by image area
