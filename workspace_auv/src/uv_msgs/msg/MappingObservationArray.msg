# One processed stereo pair, including empty pairs for visibility diagnostics.
std_msgs/Header header
bool processed
string reason
uint32 tag_candidates
uint32 tag_depth_rejected
uint32 cone_candidates
uint32 cone_depth_rejected
uint32 cone_confidence_rejected
uint32 cone_mask_rejected
MappingObservation[] observations
