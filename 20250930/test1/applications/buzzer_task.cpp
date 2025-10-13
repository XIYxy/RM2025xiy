#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

// C板
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

// 达妙
// sp::Buzzer buzzer(&htim12, TIM_CHANNEL_2, 240e6);

extern "C" void buzzer_task()
{
  // C大调音阶频率表 (Hz)
  // int notes[] = {330, 294, 262, 294, 330, 349, 330, 294};
  int notes[] = {294, 294};
  int length = sizeof(notes) / sizeof(notes[0]);

  for (int i = 0; i < length; i++) {
    buzzer.set(notes[i], 0.1);  // 设置音符频率，占空比 10%
    buzzer.start();
    osDelay(300);  // 每个音响 300ms
    buzzer.stop();
    osDelay(100);  // 间隔 100ms
  }

  while (true) {
    osDelay(100);
  }
}