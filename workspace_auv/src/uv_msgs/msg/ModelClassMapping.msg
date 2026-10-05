# Latched detector class mapping shared by perception and task nodes.
# All arrays have the same length and are indexed by ids[].
uint32 schema_version
string model
uint32[] ids
string[] names
string[] objects
string[] cameras
bool[] multi_instance
