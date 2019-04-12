# return codes
# >=0 - success
# < 0 - failure

# return code
int16 value

# return message
string message

#  Default success without additional information
int16 SUCCESS=0

# Reserved error codes
int16 INVALID_ARGUMENT=-1
int16 UNINITIALIZED=-2
int16 INTERNAL_TIMEOUT=-3
int16 SENSOR_TIMEOUT=-4
int16 REQUEST_TO_DATABASE_FAILED=-5
int16 REQUEST_TO_TF_MODULE_FAILED=-6
int16 IO_ERROR=-7

# each module defines own msg-file with consts for the module specific return codes