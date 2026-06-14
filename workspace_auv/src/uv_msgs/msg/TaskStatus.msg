# Task execution status

uint8 STATUS_IDLE     = 0
uint8 STATUS_RUNNING  = 1
uint8 STATUS_PAUSED   = 2
uint8 STATUS_DONE     = 3
uint8 STATUS_ERROR    = 4

uint8 status
uint32 current_task_index
uint32 total_tasks
string current_task_name
string error_message
