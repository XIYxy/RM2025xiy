
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

extern sp::DBus remote;
extern sp::RM_Motor motor_lf;
extern sp::PID pid_lf;
extern sp::Mecanum mecanum;
sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    plotter.plot(remote.ch_lh, motor_lf.speed, pid_lf.out, mecanum.speed_lf);
    osDelay(10);  // 100Hz
  }
}