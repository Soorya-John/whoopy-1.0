#include "board_config.h"
#include "hid_input.h"
#include "radio_tx.h"
#include "whoopy_radio_protocol.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char*TAG="tx";
void app_main(void){ESP_ERROR_CHECK(radio_tx_init());hid_input_init();uint16_t seq=0;bool arm_released_seen=false;int64_t log_t=0;while(1){tx_input_t in={.roll=500,.pitch=500,.yaw=500,.throttle=0,.valid=false};bool valid=hid_input_get(&in);static bool was_valid=false;if(!valid){arm_released_seen=false;was_valid=false;in.roll=in.pitch=in.yaw=500;in.throttle=0;in.arm=false;}else if(!was_valid){arm_released_seen=false;was_valid=true;}if(valid&&!in.arm)arm_released_seen=true;whoopy_control_t p={.type=WHOOPY_PACKET_CONTROL,.version=WHOOPY_PROTO_VERSION,.sequence=seq++,.roll=in.roll,.pitch=in.pitch,.yaw=in.yaw,.throttle=in.throttle,.flags=(valid&&arm_released_seen&&in.arm)?WHOOPY_FLAG_ARM:WHOOPY_FLAG_DISARM,.sender_ms=(uint32_t)(esp_timer_get_time()/1000)};bool sent=radio_tx_send(&p);int64_t now=esp_timer_get_time();if(now-log_t>1000000){ESP_LOGI(TAG,"HID valid=%d age=%lu axes=%u,%u,%u,%u arm=%d TX=%d ack=%lu fail=%lu",valid,(unsigned long)in.age_ms,in.roll,in.pitch,in.yaw,in.throttle,in.arm,sent,(unsigned long)radio_tx_successes(),(unsigned long)radio_tx_failures());log_t=now;}vTaskDelay(pdMS_TO_TICKS(1000/TX_RATE_HZ));}}
