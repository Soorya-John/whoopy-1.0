#include "radio_tx.h"
#include "board_config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
static spi_device_handle_t spi;static uint32_t ok_count,fail_count;
static uint8_t reg(uint8_t r){uint8_t tx[2]={r,0xff},rx[2]={0};spi_transaction_t t={.length=16,.tx_buffer=tx,.rx_buffer=rx};spi_device_transmit(spi,&t);return rx[1];}
static void wr(uint8_t r,uint8_t v){uint8_t tx[2]={(uint8_t)(0x20|r),v};spi_transaction_t t={.length=16,.tx_buffer=tx};spi_device_transmit(spi,&t);}
static void cmd(uint8_t c){spi_transaction_t t={.length=8,.tx_buffer=&c};spi_device_transmit(spi,&t);}
esp_err_t radio_tx_init(void){gpio_config_t g={.pin_bit_mask=1ULL<<PIN_RF_CE,.mode=GPIO_MODE_OUTPUT};ESP_ERROR_CHECK(gpio_config(&g));gpio_set_level(PIN_RF_CE,0);spi_bus_config_t b={.mosi_io_num=PIN_RF_MOSI,.miso_io_num=PIN_RF_MISO,.sclk_io_num=PIN_RF_SCK,.quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=64};ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST,&b,SPI_DMA_DISABLED));spi_device_interface_config_t d={.clock_speed_hz=8000000,.mode=0,.spics_io_num=PIN_RF_CSN,.queue_size=1};ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST,&d,&spi));vTaskDelay(pdMS_TO_TICKS(20));wr(0,0x0e);wr(1,0x01);wr(2,0x01);wr(3,0x03);wr(4,0x2f);wr(5,76);wr(6,0x06);wr(0x11,32);const uint8_t addr[5]={'W','H','O','P','Y'};uint8_t rxaddr[6]={0x2a,addr[0],addr[1],addr[2],addr[3],addr[4]},txaddr[6]={0x30,addr[0],addr[1],addr[2],addr[3],addr[4]};spi_transaction_t t={.length=48,.tx_buffer=rxaddr};ESP_ERROR_CHECK(spi_device_transmit(spi,&t));t.tx_buffer=txaddr;ESP_ERROR_CHECK(spi_device_transmit(spi,&t));wr(7,0x70);cmd(0xe1);gpio_set_level(PIN_RF_CE,1);vTaskDelay(pdMS_TO_TICKS(2));gpio_set_level(PIN_RF_CE,0);return ESP_OK;}
bool radio_tx_send(const whoopy_control_t*p){uint8_t payload[32],tx[33],rx[33]={0};if(!whoopy_encode_control(p,payload))return false;gpio_set_level(PIN_RF_CE,0);cmd(0xe1);tx[0]=0xa0;for(int i=0;i<32;i++)tx[i+1]=payload[i];spi_transaction_t t={.length=264,.tx_buffer=tx,.rx_buffer=rx};if(spi_device_transmit(spi,&t)!=ESP_OK){fail_count++;return false;}gpio_set_level(PIN_RF_CE,1);esp_rom_delay_us(15);gpio_set_level(PIN_RF_CE,0);int64_t start=esp_timer_get_time();uint8_t status;do{status=reg(7);if(status&0x30)break;}while(esp_timer_get_time()-start<4000);if(status&0x20){wr(7,0x20);ok_count++;return true;}if(status&0x10)wr(7,0x10);cmd(0xe1);fail_count++;return false;}
uint32_t radio_tx_successes(void){return ok_count;}uint32_t radio_tx_failures(void){return fail_count;}
