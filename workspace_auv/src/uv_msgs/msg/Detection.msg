# Single object detection result. Pixel coordinates refer to the image_space
# and calibration_id in its enclosing DetectionArray; they are eye-local.

uint16 class_id
float32 confidence
float32 pixel_x       # Center pixel X
float32 pixel_y       # Center pixel Y
float32 bbox_x1       # Bounding box top-left X
float32 bbox_y1       # Bounding box top-left Y
float32 bbox_x2       # Bounding box bottom-right X
float32 bbox_y2       # Bounding box bottom-right Y

# Optional stable image feature used by the front geometric estimator.  The
# bbox center remains in pixel_x/pixel_y for compatibility; front localization
# uses feature_pixel_x/feature_pixel_y when feature_type is non-zero.
uint8 FEATURE_BBOX_CENTER=0
uint8 FEATURE_GATE_CENTERLINE=1
uint8 FEATURE_GATE_SEGMENTATION=2
uint8 feature_type
float32 feature_pixel_x
float32 feature_pixel_y

# Optional projected red-ring axis, computed only for down-view detections.
# Detector eye-image coordinates: right=0 deg, down=90 deg, modulo 180 deg.
# This is an undirected 2D axis, not a body yaw or complete 3D normal.
# Quality is PCA anisotropy in [0,1], not a detection probability.
# Invalid orientation: all three fields are false/0/0.
bool orientation_valid
float32 orientation_axis_deg
float32 orientation_quality
