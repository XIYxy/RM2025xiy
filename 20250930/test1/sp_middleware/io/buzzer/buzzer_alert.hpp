#ifndef BUZZER_ALERT_HPP
#define BUZZER_ALERT_HPP

#ifdef __cplusplus
extern "C" {
#endif

// 预定义两种报警
void buzzer_alert1();
void buzzer_alert2();

// 通用报警接口：传入频率和持续时间
void buzzer_alert_custom(float hz, int duration_ms, float duty = 0.5f);

#ifdef __cplusplus
}
#endif

#endif
