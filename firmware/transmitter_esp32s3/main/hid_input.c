#include "hid_input.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include <string.h>
#include <wchar.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
static const char*TAG="hid";static hid_host_device_handle_t handle;static SemaphoreHandle_t lock;static uint8_t report[128];static size_t report_len;static int64_t report_time;static bool connected;
#ifdef CONFIG_WHOOPY_HID_ROLL_INVERT
#define INV_ROLL true
#else
#define INV_ROLL false
#endif
#ifdef CONFIG_WHOOPY_HID_PITCH_INVERT
#define INV_PITCH true
#else
#define INV_PITCH false
#endif
#ifdef CONFIG_WHOOPY_HID_YAW_INVERT
#define INV_YAW true
#else
#define INV_YAW false
#endif
#ifdef CONFIG_WHOOPY_HID_THROTTLE_INVERT
#define INV_THROTTLE true
#else
#define INV_THROTTLE false
#endif
static uint32_t bit_value(const uint8_t*p,size_t len,unsigned bit,unsigned width){if(bit+width>len*8||width>16)return 0;uint32_t v=0;for(unsigned i=0;i<width;i++)v|=((p[(bit+i)/8]>>((bit+i)%8))&1u)<<i;return v;}
static int32_t axis_raw(uint32_t bits,unsigned width){if(CONFIG_WHOOPY_HID_AXIS_MIN<0&&width<32&&(bits&(1u<<(width-1))))return (int32_t)(bits|(~0u<<width));return (int32_t)bits;}
static uint16_t axis(int32_t raw,bool centered,bool invert){int lo=CONFIG_WHOOPY_HID_AXIS_MIN,hi=CONFIG_WHOOPY_HID_AXIS_MAX;if(hi<=lo)return centered?500:0;if(centered){int mid=lo+(hi-lo)/2;float x=(float)(raw-mid)/(float)((hi-lo)/2);float dz=CONFIG_WHOOPY_HID_DEADZONE/1000.0f;if(x>-dz&&x<dz)x=0;else x=x>0?(x-dz)/(1-dz):(x+dz)/(1-dz);if(x>1)x=1;if(x< -1)x=-1;if(invert)x=-x;return (uint16_t)(500+x*500);}float x=(float)(raw-lo)/(hi-lo);if(x<0)x=0;if(x>1)x=1;if(invert)x=1-x;return (uint16_t)(x*1000);}
bool hid_input_get(tx_input_t*out){if(!out||!lock)return false;xSemaphoreTake(lock,portMAX_DELAY);bool ok=connected&&report_len>0;size_t len=report_len;uint8_t r[128];memcpy(r,report,len);int64_t tm=report_time;xSemaphoreGive(lock);if(!ok)return false;
 const uint8_t*fields=r;size_t field_len=len;
#ifdef CONFIG_WHOOPY_HID_REPORT_ID_PRESENT
 if(field_len>0){fields++;field_len--;}
#endif
 unsigned w=CONFIG_WHOOPY_HID_AXIS_WIDTH;unsigned end0=CONFIG_WHOOPY_HID_AXIS0_BIT+w,end1=CONFIG_WHOOPY_HID_AXIS1_BIT+w,end2=CONFIG_WHOOPY_HID_AXIS2_BIT+w,end3=CONFIG_WHOOPY_HID_AXIS3_BIT+w,arm_end=CONFIG_WHOOPY_ARM_BUTTON_BIT+1;unsigned required=end0;if(end1>required)required=end1;if(end2>required)required=end2;if(end3>required)required=end3;if(arm_end>required)required=arm_end;if(required>field_len*8){out->valid=false;return false;}uint32_t a0=bit_value(fields,field_len,CONFIG_WHOOPY_HID_AXIS0_BIT,w),a1=bit_value(fields,field_len,CONFIG_WHOOPY_HID_AXIS1_BIT,w),a2=bit_value(fields,field_len,CONFIG_WHOOPY_HID_AXIS2_BIT,w),a3=bit_value(fields,field_len,CONFIG_WHOOPY_HID_AXIS3_BIT,w);out->roll=axis(axis_raw(a0,w),true,INV_ROLL);out->pitch=axis(axis_raw(a1,w),true,INV_PITCH);out->yaw=axis(axis_raw(a2,w),true,INV_YAW);out->throttle=axis(axis_raw(a3,w),false,INV_THROTTLE);bool arm=bit_value(fields,field_len,CONFIG_WHOOPY_ARM_BUTTON_BIT,1)!=0;
#ifdef CONFIG_WHOOPY_ARM_ACTIVE_HIGH
 out->arm=arm;
#else
 out->arm=!arm;
#endif
 out->age_ms=(uint32_t)((esp_timer_get_time()-tm)/1000);out->valid=out->age_ms<150;return out->valid;}
static void interface_cb(hid_host_device_handle_t h,const hid_host_interface_event_t e,void*arg){if(e==HID_HOST_INTERFACE_EVENT_INPUT_REPORT){uint8_t d[128];size_t n=0;if(hid_host_device_get_raw_input_report_data(h,d,sizeof d,&n)==ESP_OK){xSemaphoreTake(lock,0);if(n<=sizeof report){memcpy(report,d,n);report_len=n;report_time=esp_timer_get_time();connected=true;}xSemaphoreGive(lock);}}else if(e==HID_HOST_INTERFACE_EVENT_DISCONNECTED||e==HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR){xSemaphoreTake(lock,0);connected=false;report_len=0;xSemaphoreGive(lock);ESP_LOGW(TAG,"HID interface disconnected or transfer error");}}
static void driver_cb(hid_host_device_handle_t h,const hid_host_driver_event_t e,void*arg){if(e!=HID_HOST_DRIVER_EVENT_CONNECTED)return;hid_host_device_config_t c={.callback=interface_cb,.callback_arg=NULL};if(hid_host_device_open(h,&c)!=ESP_OK){ESP_LOGE(TAG,"HID open failed");return;}handle=h;hid_host_dev_params_t p={0};hid_host_device_get_params(h,&p);hid_host_dev_info_t info={0};hid_host_get_device_info(h,&info);size_t n=0;uint8_t*d=hid_host_get_report_descriptor(h,&n);ESP_LOGI(TAG,"HID connected addr=%u iface=%u VID=%04x PID=%04x product=%ls report descriptor bytes=%u",p.addr,p.iface_num,info.VID,info.PID,info.iProduct,(unsigned)n);if(d){for(size_t i=0;i<n;i++)ESP_LOGD(TAG,"descriptor[%u]=%02x",(unsigned)i,d[i]);}if(hid_host_device_start(h)!=ESP_OK)ESP_LOGE(TAG,"HID input start failed");}
static void usb_daemon(void*arg){while(1){uint32_t f;esp_err_t e=usb_host_lib_handle_events(portMAX_DELAY,&f);if(e!=ESP_OK)ESP_LOGW(TAG,"USB host daemon %s",esp_err_to_name(e));}}
static void hid_task(void*arg){usb_host_config_t u={.skip_phy_setup=false,.intr_flags=ESP_INTR_FLAG_LEVEL1};ESP_ERROR_CHECK(usb_host_install(&u));xTaskCreate(usb_daemon,"usb_daemon",4096,NULL,5,NULL);hid_host_driver_config_t h={.create_background_task=true,.task_priority=5,.stack_size=4096,.core_id=tskNO_AFFINITY,.callback=driver_cb};ESP_ERROR_CHECK(hid_host_install(&h));ESP_LOGI(TAG,"USB HID host ready; connect Boxer in USB joystick mode");vTaskDelete(NULL);}
void hid_input_init(void){lock=xSemaphoreCreateMutex();xTaskCreate(hid_task,"hid_host",4096,NULL,6,NULL);}
