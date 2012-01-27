Header header

geometry_msgs/Point32           current_setpoint_pos
float32                         current_setpoint_yaw

bool                            hl_interface_enabled

int32                           mav_controller_mode
int32                           MAV_CONTROLLER_MANUAL = 0
int32                           MAV_CONTROLLER_HL_HEIGHT = 1   # height control on the HLP
int32                           MAV_CONTROLLER_HL_POS = 2      # position control on the HLP
int32                           MAV_CONTROLLER_LL_HEIGHT = 3   # height control on the LLP
int32                           MAV_CONTROLLER_LL_GPS = 4      # position control based on GPS on the LLP

bool                            gps_lock
int32                           gps_num_satellites

bool                            fcu_alive
bool                            vismaggps_alive

int32                           filter_status                  # consts are defined in vismaggps_fusion/Status
