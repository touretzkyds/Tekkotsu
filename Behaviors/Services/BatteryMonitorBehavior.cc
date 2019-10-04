#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_POWER_STATUS

#include "BatteryMonitorBehavior.h"

REGISTER_BEHAVIOR_MENU_OPT(BatteryMonitorBehavior,"Background Behaviors/System Daemons",BEH_NONEXCLUSIVE|BEH_START);

const double BatteryMonitorBehavior::low_voltage_threshold=11.0;

#endif
