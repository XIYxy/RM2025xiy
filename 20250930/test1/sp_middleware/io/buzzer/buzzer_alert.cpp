#include "buzzer_alert.hpp"

#include "buzzer.hpp"
#include "cmsis_os.h"

// buzzer 是在别的地方初始化过的，这里只用 extern 声明
extern sp::Buzzer buzzer;

void buzzer_alert1()
{
  buzzer.set(1000, 0.5f);  // 1000Hz, 占空比50%
  buzzer.start();
  osDelay(500);  // 延时 0.5s
  buzzer.stop();
}

void buzzer_alert2()
{
  buzzer.set(3000, 0.5f);  // 3000Hz, 占空比50%
  buzzer.start();
  osDelay(500);
  buzzer.stop();
}

void buzzer_alert_custom(float hz, int duration_ms, float duty)
{
  buzzer.set(hz, duty);
  buzzer.start();
  osDelay(duration_ms);
  buzzer.stop();
}
