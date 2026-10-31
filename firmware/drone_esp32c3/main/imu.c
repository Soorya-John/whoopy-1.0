#include "imu.h"
#include "board_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
static const char *TAG="imu"; static float bias[3],att_r,att_p,filter_alpha=.98f;static bool healthy;
static esp_err_t wr(uint8_t reg,uint8_t v){uint8_t b[2]={reg,v};return i2c_master_write_to_device(I2C_NUM_0,MPU6050_ADDRESS,b,2,pdMS_TO_TICKS(50));}
static esp_err_t rd(uint8_t reg,uint8_t *b,size_t n){return i2c_master_write_read_device(I2C_NUM_0,MPU6050_ADDRESS,&reg,1,b,n,pdMS_TO_TICKS(50));}
esp_err_t imu_init(void){i2c_config_t c={.mode=I2C_MODE_MASTER,.sda_io_num=PIN_I2C_SDA,.scl_io_num=PIN_I2C_SCL,.sda_pullup_en=GPIO_PULLUP_ENABLE,.scl_pullup_en=GPIO_PULLUP_ENABLE,.master.clk_speed=400000};ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0,&c));ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0,c.mode,0,0,0));uint8_t id=0;if(rd(0x75,&id,1)!=ESP_OK||id!=0x68){ESP_LOGE(TAG,"MPU6050 WHO_AM_I invalid: 0x%02x",id);return ESP_FAIL;}if(wr(0x6b,0)||wr(0x1b,0x08)||wr(0x1c,0x08)||wr(0x1a,0x03))return ESP_FAIL;vTaskDelay(pdMS_TO_TICKS(100));healthy=true;return ESP_OK;}
esp_err_t imu_calibrate(void){int32_t sums[3]={0};uint8_t b[14];for(int i=0;i<50;i++){if(rd(0x3b,b,14)!=ESP_OK)return ESP_FAIL;float ax=(int16_t)((b[0]<<8)|b[1])/8192.0f,ay=(int16_t)((b[2]<<8)|b[3])/8192.0f,az=(int16_t)((b[4]<<8)|b[5])/8192.0f;float gx=(int16_t)((b[8]<<8)|b[9])/65.5f,gy=(int16_t)((b[10]<<8)|b[11])/65.5f,gz=(int16_t)((b[12]<<8)|b[13])/65.5f;float amag=sqrtf(ax*ax+ay*ay+az*az);if(amag<0.80f||amag>1.20f||fabsf(gx)>5||fabsf(gy)>5||fabsf(gz)>5)return ESP_ERR_INVALID_STATE;vTaskDelay(pdMS_TO_TICKS(10));}for(int i=0;i<400;i++){if(rd(0x3b,b,14)!=ESP_OK)return ESP_FAIL;sums[0]+=(int16_t)((b[8]<<8)|b[9]);sums[1]+=(int16_t)((b[10]<<8)|b[11]);sums[2]+=(int16_t)((b[12]<<8)|b[13]);vTaskDelay(pdMS_TO_TICKS(3));}for(int j=0;j<3;j++)bias[j]=(float)sums[j]/400.0f/65.5f;return ESP_OK;}
esp_err_t imu_read(imu_sample_t *o,float dt){uint8_t b[14];if(!o||rd(0x3b,b,14)!=ESP_OK){healthy=false;return ESP_FAIL;}int16_t raw[7];for(int i=0;i<7;i++)raw[i]=(int16_t)(((uint16_t)b[2*i]<<8)|b[2*i+1]);o->ax=raw[0]/8192.0f;o->ay=raw[1]/8192.0f;o->az=raw[2]/8192.0f;o->gx=raw[4]/65.5f-bias[0];o->gy=raw[5]/65.5f-bias[1];o->gz=raw[6]/65.5f-bias[2];float ar=atan2f(o->ay,o->az)*57.29578f,ap=atan2f(-o->ax,sqrtf(o->ay*o->ay+o->az*o->az))*57.29578f;att_r=filter_alpha*(att_r+o->gx*dt)+(1-filter_alpha)*ar;att_p=filter_alpha*(att_p+o->gy*dt)+(1-filter_alpha)*ap;o->roll=att_r;o->pitch=att_p;healthy=true;return ESP_OK;}
void imu_set_filter_alpha(float a){if(a>=.90f&&a<=.999f)filter_alpha=a;}
bool imu_healthy(void){return healthy;}
