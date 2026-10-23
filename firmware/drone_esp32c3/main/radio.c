#include "radio.h"
#include "board_config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define CMD_W 0x20
#define CMD_RX 0x61
#define CMD_FLUSH_RX 0xe2
#define CMD_NOP 0xff
#define CRSF_MAX_FRAME 64
#define CRSF_RC_CHANNELS 0x16
#define CRSF_LINK_TIMEOUT_US 1000000LL

static spi_device_handle_t dev;
static const char *TAG="radio";
static volatile uint32_t valid_packets,invalid_packets;
static volatile receiver_mode_t selected_mode=RECEIVER_NRF24;
static volatile bool nrf_present;
static int64_t nrf_probe_us;
static portMUX_TYPE crsf_mux=portMUX_INITIALIZER_UNLOCKED;
static whoopy_control_t crsf_packet;
static uint32_t crsf_generation,crsf_consumed_generation;
static uint16_t crsf_sequence;
static int64_t crsf_last_frame_us;

static uint8_t reg_read(uint8_t r){uint8_t tx[2]={r,0xff},rx[2]={0};spi_transaction_t t={.length=16,.tx_buffer=tx,.rx_buffer=rx};spi_device_transmit(dev,&t);return rx[1];}
static void reg_write(uint8_t r,uint8_t v){uint8_t tx[2]={CMD_W|r,v};spi_transaction_t t={.length=16,.tx_buffer=tx};spi_device_transmit(dev,&t);}
static void command(uint8_t c){spi_transaction_t t={.length=8,.tx_buffer=&c};spi_device_transmit(dev,&t);}
static bool nrf_probe(void){return reg_read(0x00)==0x0f;}
static void update_nrf_presence(void){int64_t now=esp_timer_get_time();if(now-nrf_probe_us<500000)return;nrf_probe_us=now;nrf_present=nrf_probe();}
static uint8_t crsf_crc(const uint8_t *data,size_t len){uint8_t crc=0;for(size_t i=0;i<len;i++){crc^=data[i];for(int b=0;b<8;b++)crc=(crc&0x80)?(uint8_t)((crc<<1)^0xd5):(uint8_t)(crc<<1);}return crc;}
static uint16_t channel(const uint8_t *p,unsigned ch){unsigned bit=ch*11;uint16_t v=0;for(unsigned i=0;i<11;i++)v|=(uint16_t)(((p[(bit+i)/8]>>((bit+i)%8))&1u)<<i);return v;}
static uint16_t scale_axis(uint16_t v){int32_t x=((int32_t)v-992)*500/820+500;if(x<0)x=0;if(x>1000)x=1000;return (uint16_t)x;}
static uint16_t scale_throttle(uint16_t v){int32_t x=((int32_t)v-172)*1000/1639;if(x<0)x=0;if(x>1000)x=1000;return (uint16_t)x;}
static void crsf_frame(const uint8_t *frame,size_t total){uint8_t len=frame[1];if(len<2||len>CRSF_MAX_FRAME-2||crsf_crc(frame+2,len-1)!=frame[len+1]){invalid_packets++;return;}int64_t now=esp_timer_get_time();portENTER_CRITICAL(&crsf_mux);crsf_last_frame_us=now;portEXIT_CRITICAL(&crsf_mux);if(frame[2]!=CRSF_RC_CHANNELS||len!=24)return;const uint8_t *ch=frame+3;uint16_t arm=channel(ch,4);whoopy_control_t p={.type=WHOOPY_PACKET_CONTROL,.version=WHOOPY_PROTO_VERSION,.sequence=crsf_sequence++,.roll=scale_axis(channel(ch,0)),.pitch=scale_axis(channel(ch,1)),.throttle=scale_throttle(channel(ch,2)),.yaw=scale_axis(channel(ch,3)),.flags=arm>992?WHOOPY_FLAG_ARM:WHOOPY_FLAG_DISARM,.sender_ms=(uint32_t)(now/1000)};portENTER_CRITICAL(&crsf_mux);crsf_packet=p;crsf_generation++;portEXIT_CRITICAL(&crsf_mux);valid_packets++;(void)total;}
static void crsf_task(void *arg){uint8_t input[128],frame[CRSF_MAX_FRAME];size_t used=0,expected=0;for(;;){int n=uart_read_bytes(UART_NUM_1,input,sizeof(input),pdMS_TO_TICKS(10));for(int i=0;i<n;i++){uint8_t b=input[i];if(used==0){frame[used++]=b;continue;}if(used==1){if(b<2||b>CRSF_MAX_FRAME-2){used=0;expected=0;continue;}frame[used++]=b;expected=(size_t)b+2;continue;}frame[used++]=b;if(used==expected){crsf_frame(frame,used);used=0;expected=0;}else if(used>=sizeof(frame)){used=0;expected=0;}}}}

esp_err_t radio_init(void){gpio_config_t g={.pin_bit_mask=1ULL<<PIN_RF_CE,.mode=GPIO_MODE_OUTPUT};ESP_ERROR_CHECK(gpio_config(&g));gpio_set_level(PIN_RF_CE,0);spi_bus_config_t b={.mosi_io_num=PIN_RF_MOSI,.miso_io_num=PIN_RF_MISO,.sclk_io_num=PIN_RF_SCK,.quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=64};ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST,&b,SPI_DMA_DISABLED));spi_device_interface_config_t d={.clock_speed_hz=8000000,.mode=0,.spics_io_num=PIN_RF_CSN,.queue_size=1};ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST,&d,&dev));vTaskDelay(pdMS_TO_TICKS(20));reg_write(0x00,0x0f);reg_write(0x01,0x01);reg_write(0x02,0x01);reg_write(0x03,0x03);reg_write(0x04,0x2f);reg_write(0x05,76);reg_write(0x06,0x06);reg_write(0x11,32);const uint8_t addr[5]={'W','H','O','P','Y'};uint8_t tx[6]={CMD_W|0x0a,addr[0],addr[1],addr[2],addr[3],addr[4]};spi_transaction_t t={.length=48,.tx_buffer=tx};ESP_ERROR_CHECK(spi_device_transmit(dev,&t));reg_write(0x07,0x70);command(CMD_FLUSH_RX);gpio_set_level(PIN_RF_CE,1);vTaskDelay(pdMS_TO_TICKS(2));nrf_present=nrf_probe();nrf_probe_us=esp_timer_get_time();ESP_LOGI(TAG,"nRF24 probe: %s",nrf_present?"detected":"not detected");
 uart_config_t uc={.baud_rate=CRSF_UART_BAUD,.data_bits=UART_DATA_8_BITS,.parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,.flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};esp_err_t ue=uart_driver_install(UART_NUM_1,1024,0,0,NULL,0);if(ue==ESP_OK)ue=uart_param_config(UART_NUM_1,&uc);if(ue==ESP_OK)ue=uart_set_pin(UART_NUM_1,PIN_CRSF_TX,PIN_CRSF_RX,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);if(ue!=ESP_OK)ESP_LOGE(TAG,"CRSF UART init failed: %s",esp_err_to_name(ue));else{xTaskCreate(crsf_task,"crsf_rx",3072,NULL,configMAX_PRIORITIES-4,NULL);ESP_LOGI(TAG,"CRSF UART ready at %d baud, RX GPIO%d",CRSF_UART_BAUD,PIN_CRSF_RX);}return ESP_OK;}

bool radio_receive(whoopy_control_t *p){if(!p)return false;update_nrf_presence();if(selected_mode==RECEIVER_ELRS){bool fresh=false;portENTER_CRITICAL(&crsf_mux);if(crsf_generation!=crsf_consumed_generation){*p=crsf_packet;crsf_consumed_generation=crsf_generation;fresh=true;}portEXIT_CRITICAL(&crsf_mux);return fresh;}if(!nrf_present)return false;uint8_t s=reg_read(0x07);if(!(s&0x40))return false;uint8_t tx[33]={CMD_RX},rx[33]={0};spi_transaction_t t={.length=264,.tx_buffer=tx,.rx_buffer=rx};if(spi_device_transmit(dev,&t)!=ESP_OK)return false;reg_write(0x07,0x40);if(!whoopy_decode_control(rx+1,32,p)){invalid_packets++;return false;}valid_packets++;return true;}
void radio_set_mode(receiver_mode_t mode){if(mode==RECEIVER_NRF24||mode==RECEIVER_ELRS)selected_mode=mode;}
receiver_mode_t radio_get_mode(void){return selected_mode;}
bool radio_nrf_detected(void){return nrf_present;}
bool radio_elrs_detected(void){int64_t last;portENTER_CRITICAL(&crsf_mux);last=crsf_last_frame_us;portEXIT_CRITICAL(&crsf_mux);return last>0&&esp_timer_get_time()-last<CRSF_LINK_TIMEOUT_US;}
void radio_get_stats(radio_stats_t*stats){if(!stats)return;stats->valid_packets=valid_packets;stats->invalid_packets=invalid_packets;}
