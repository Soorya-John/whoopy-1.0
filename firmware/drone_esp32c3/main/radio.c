#include "radio.h"
#include "board_config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#define CMD_W 0x20
#define CMD_RX 0x61
#define CMD_FLUSH_RX 0xe2
#define CMD_NOP 0xff
static spi_device_handle_t dev;static const char*TAG="radio";
static uint8_t reg_read(uint8_t r){uint8_t tx[2]={r,0xff},rx[2]={0};spi_transaction_t t={.length=16,.tx_buffer=tx,.rx_buffer=rx};spi_device_transmit(dev,&t);return rx[1];}
static void reg_write(uint8_t r,uint8_t v){uint8_t tx[2]={CMD_W|r,v};spi_transaction_t t={.length=16,.tx_buffer=tx};spi_device_transmit(dev,&t);}
static void command(uint8_t c){spi_transaction_t t={.length=8,.tx_buffer=&c};spi_device_transmit(dev,&t);}
esp_err_t radio_init(void){gpio_config_t g={.pin_bit_mask=1ULL<<PIN_RF_CE,.mode=GPIO_MODE_OUTPUT};ESP_ERROR_CHECK(gpio_config(&g));gpio_set_level(PIN_RF_CE,0);spi_bus_config_t b={.mosi_io_num=PIN_RF_MOSI,.miso_io_num=PIN_RF_MISO,.sclk_io_num=PIN_RF_SCK,.quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=64};ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST,&b,SPI_DMA_DISABLED));spi_device_interface_config_t d={.clock_speed_hz=8000000,.mode=0,.spics_io_num=PIN_RF_CSN,.queue_size=1};ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST,&d,&dev));vTaskDelay(pdMS_TO_TICKS(20));reg_write(0x00,0x0f);reg_write(0x01,0x01);reg_write(0x02,0x01);reg_write(0x03,0x03);reg_write(0x04,0x2f);reg_write(0x05,76);reg_write(0x06,0x06);reg_write(0x11,32);const uint8_t addr[5]={'W','H','O','P','Y'};uint8_t tx[6]={CMD_W|0x0a,addr[0],addr[1],addr[2],addr[3],addr[4]};spi_transaction_t t={.length=48,.tx_buffer=tx};ESP_ERROR_CHECK(spi_device_transmit(dev,&t));reg_write(0x07,0x70);command(CMD_FLUSH_RX);gpio_set_level(PIN_RF_CE,1);vTaskDelay(pdMS_TO_TICKS(2));ESP_LOGI(TAG,"RX channel 76, 1Mbps; verify module variant and supply");return ESP_OK;}
bool radio_receive(whoopy_control_t *p){uint8_t s=reg_read(0x07);if(!(s&0x40))return false;uint8_t tx[33]={CMD_RX},rx[33]={0};spi_transaction_t t={.length=264,.tx_buffer=tx,.rx_buffer=rx};if(spi_device_transmit(dev,&t)!=ESP_OK)return false;reg_write(0x07,0x40);return whoopy_decode_control(rx+1,32,p);}
