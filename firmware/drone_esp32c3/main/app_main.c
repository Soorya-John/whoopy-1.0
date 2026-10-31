#include "board_config.h"
#include "drone_config.h"
#include "imu.h"
#include "motor.h"
#include "radio.h"
#include "web_config.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>
static const char*TAG="flight";static drone_config_t cfg;static live_status_t live={.state="BOOT"};static volatile bool calibrate_req,armed,calibrating;static whoopy_flight_state_t state=WHOOPY_BOOT;static bool init_ok;static whoopy_control_t last;static bool have_packet,arm_seen_low=true;static int64_t packet_us;
static whoopy_pid_state_t pr,pp,py;
static const char*state_name(whoopy_flight_state_t s){switch(s){case WHOOPY_BOOT:return"BOOT";case WHOOPY_INITIALIZING:return"INITIALIZING";case WHOOPY_DISARMED:return"DISARMED";case WHOOPY_ARMED:return"ARMED";default:return"FAILSAFE";}}
static float norm_axis(uint16_t x){return ((float)x-500.0f)/500.0f;}
static void calibration_task(void*arg){for(;;){if(calibrate_req){calibrate_req=false;if(state==WHOOPY_DISARMED){calibrating=true;live.calibration_status="running";ESP_LOGI(TAG,"gyro calibration started; keep stationary");esp_err_t e=imu_calibrate();live.calibration_status=e==ESP_OK?"complete":"failed: move less and retry";ESP_LOGI(TAG,"gyro calibration %s",e==ESP_OK?"complete":"failed stationary/IMU check");calibrating=false;}}vTaskDelay(pdMS_TO_TICKS(20));}}
static void flight_task(void*arg){TickType_t wake=xTaskGetTickCount();const TickType_t period=pdMS_TO_TICKS(1000/CONTROL_HZ);imu_sample_t im={0};drone_config_t active;uint16_t prev_seq=0;bool seq_ok=false,prev_arm=false;int64_t last_t=esp_timer_get_time();config_snapshot(&cfg,&active);imu_set_filter_alpha(active.complementary_alpha);
 receiver_mode_t previous_mode=radio_get_mode();for(;;){vTaskDelayUntil(&wake,period);config_snapshot(&cfg,&active);int64_t start=esp_timer_get_time();float dt=(start-last_t)/1000000.0f;last_t=start;receiver_mode_t current_mode=radio_get_mode();if(current_mode!=previous_mode){have_packet=false;seq_ok=false;memset(&last,0,sizeof last);arm_seen_low=true;previous_mode=current_mode;}
  whoopy_control_t p;if(radio_receive(&p)){if(!seq_ok||whoopy_seq_newer(p.sequence,prev_seq)){if(seq_ok){uint16_t gap=(uint16_t)(p.sequence-prev_seq);if(gap>1)live.lost_packets+=(uint32_t)gap-1;}prev_seq=p.sequence;seq_ok=true;live.accepted_packets++;last=p;have_packet=true;packet_us=start;arm_seen_low=(p.flags&WHOOPY_FLAG_ARM)==0;live.sequence=p.sequence;}}
  bool link=have_packet&&((start-packet_us)/1000)<active.failsafe_ms;bool imuok=imu_read(&im,dt)==ESP_OK;bool arm=(last.flags&WHOOPY_FLAG_ARM)!=0;bool edge=arm&&!prev_arm&&arm_seen_low;bool dis=(last.flags&WHOOPY_FLAG_DISARM)!=0;bool low=last.throttle<80;prev_arm=arm;
  whoopy_flight_state_t before=state;
  if(state==WHOOPY_BOOT)state=WHOOPY_INITIALIZING;
  state=whoopy_safety_step(state,init_ok,imuok,link,edge&&!calibrating,dis,low);
  if(before==WHOOPY_FAILSAFE&&state==WHOOPY_DISARMED){prev_arm=arm;arm_seen_low=!arm;}
  if(state!=WHOOPY_ARMED){motors_stop();memset(live.motors,0,sizeof live.motors);memset(&pr,0,sizeof pr);memset(&pp,0,sizeof pp);memset(&py,0,sizeof py);}
  else {float r=norm_axis(last.roll),pval=norm_axis(last.pitch),y=norm_axis(last.yaw),thr=(float)last.throttle/1000.0f;float rrate,prate;
   if((last.flags&WHOOPY_FLAG_ACRO)||active.acro){rrate=r*active.max_rate_dps;prate=pval*active.max_rate_dps;}
   else {rrate=(r*active.max_angle_deg-im.roll)*4.0f;prate=(pval*active.max_angle_deg-im.pitch)*4.0f;}
   float ro=whoopy_pid_step(&pr,&active.roll,rrate,im.gx,dt),po=whoopy_pid_step(&pp,&active.pitch,prate,im.gy,dt),yo=whoopy_pid_step(&py,&active.yaw,y*active.max_rate_dps,im.gz,dt);whoopy_mix_t mix;whoopy_mix_x(thr,ro,po,yo,&mix);float outputs[4]={0};for(int i=0;i<4;i++){mix.motor[i]*=active.max_motor_output;outputs[active.motor_map[i]]=mix.motor[i];}memcpy(live.motors,outputs,sizeof outputs);motors_write(outputs);
  }
  armed=state==WHOOPY_ARMED;live.state=state_name(state);live.calibrating=calibrating;live.imu_ok=imuok;live.radio_ok=link;live.controls_available=have_packet;live.packet_age_ms=link?(uint32_t)((start-packet_us)/1000):UINT32_MAX;live.loop_us=(uint32_t)(esp_timer_get_time()-start);live.ax=im.ax;live.ay=im.ay;live.az=im.az;live.roll=im.roll;live.pitch=im.pitch;live.gx=im.gx;live.gy=im.gy;live.gz=im.gz;live.controls[0]=norm_axis(last.roll);live.controls[1]=norm_axis(last.pitch);live.controls[2]=norm_axis(last.yaw);live.controls[3]=last.throttle/1000.0f;live.control_mode=((last.flags&WHOOPY_FLAG_ACRO)||active.acro)?"ACRO / RATE":"STABILIZED";radio_stats_t rs;radio_get_stats(&rs);live.valid_packets=rs.valid_packets;live.invalid_packets=rs.invalid_packets;
  if(state!=before)ESP_LOGW(TAG,"state %s -> %s link=%d imu=%d",state_name(before),state_name(state),link,imuok);
  if(live.loop_us>3500){motors_stop();memset(live.motors,0,sizeof live.motors);state=WHOOPY_FAILSAFE;armed=false;live.state=state_name(state);ESP_LOGE(TAG,"loop overrun (%lu us), motor shutdown",(unsigned long)live.loop_us);}
 }
}
void app_main(void){motors_init();motors_stop();ESP_ERROR_CHECK(nvs_flash_init());esp_err_t ce=config_load(&cfg);if(!config_valid(&cfg))config_defaults(&cfg);live.config_status=ce==ESP_OK?"loaded from NVS":"defaults in RAM";live.save_status="unchanged this session";uint8_t saved_mode=0;esp_err_t me=config_load_receiver_mode(&saved_mode);radio_set_mode(saved_mode==1?RECEIVER_ELRS:RECEIVER_NRF24);ESP_LOGI(TAG,"disarmed startup; cfg v%lu receiver=%s mode_storage=%s",(unsigned long)cfg.version,saved_mode==1?"ELRS/CRSF":"nRF24",me==ESP_OK?"loaded":"default");esp_err_t ie=imu_init();if(ie==ESP_OK){ESP_LOGI(TAG,"keep stationary: gyro calibration");ie=imu_calibrate();live.calibration_status=ie==ESP_OK?"startup complete":"startup calibration failed";}else {ESP_LOGE(TAG,"IMU init failed; motor outputs remain disabled");live.calibration_status="IMU unavailable";}esp_err_t re=radio_init();if(re!=ESP_OK)ESP_LOGE(TAG,"radio initialization failed");init_ok=ie==ESP_OK&&re==ESP_OK;web_config_start(&cfg,&live,&calibrate_req,&armed);state=init_ok?WHOOPY_DISARMED:WHOOPY_FAILSAFE;live.state=state_name(state);xTaskCreatePinnedToCore(calibration_task,"calibration",3072,NULL,configMAX_PRIORITIES-5,NULL,0);xTaskCreatePinnedToCore(flight_task,"flight",6144,NULL,configMAX_PRIORITIES-3,NULL,0);}
