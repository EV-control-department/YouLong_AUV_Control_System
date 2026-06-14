# Motion command from control layer to sim_bridge
# Replaces ZitSetpoint with a cleaner interface

uint8 MODE_POS_WORLD      = 0   # Absolute world NED position
uint8 MODE_POS_BODY       = 1   # Absolute body-frame position
uint8 MODE_VEL_WORLD      = 2   # World-frame velocity
uint8 MODE_VEL_BODY       = 3   # Body-frame velocity
uint8 MODE_FORCE_BODY     = 4   # Direct body-frame force
uint8 MODE_POS_WORLD_STEP = 5   # Incremental world position step
uint8 MODE_POS_BODY_STEP  = 6   # Incremental body position step
uint8 MODE_VEL_WORLD_STEP = 7   # Incremental world velocity step
uint8 MODE_VEL_BODY_STEP  = 8   # Incremental body velocity step

uint8 mode            # One of the MODE_* constants
uint8 type_mask       # Bitmask: bit0=x, bit1=y, bit2=z, bit3=yaw (1=ignore axis)

float32 x
float32 y
float32 z
float32 yaw           # Radians
