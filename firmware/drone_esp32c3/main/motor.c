#include "motor.h"
#include "board_config.h"
#include "driver/ledc.h"
#include "esp_err.h"
static const int pins[4]={PIN_MOTOR_1,PIN_MOTOR_2,PIN_MOTOR_3,PIN_MOTOR_4};
void motors_init(void){
 ledc_timer_config_t t={.speed_mode=LEDC_LOW_SPEED_MODE,.duty_resolution=LEDC_TIMER_10_BIT,.timer_num=LEDC_TIMER_0,.freq_hz=20000,.clk_cfg=LEDC_AUTO_CLK}; ledc_timer_config(&t);
 for(int i=0;i<4;i++){ledc_channel_config_t c={.gpio_num=pins[i],.speed_mode=LEDC_LOW_SPEED_MODE,.channel=(ledc_channel_t)i,.intr_type=LEDC_INTR_DISABLE,.timer_sel=LEDC_TIMER_0,.duty=0,.hpoint=0};ledc_channel_config(&c);} motors_stop();
}
void motors_write(const float d[4]){for(int i=0;i<4;i++){float x=d[i];if(x<0)x=0;if(x>1)x=1;ledc_set_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)i,(uint32_t)(x*1023));ledc_update_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)i);}}
void motors_stop(void){const float z[4]={0};motors_write(z);}
