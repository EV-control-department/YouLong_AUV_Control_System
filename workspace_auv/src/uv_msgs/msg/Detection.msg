# Single object detection result

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
