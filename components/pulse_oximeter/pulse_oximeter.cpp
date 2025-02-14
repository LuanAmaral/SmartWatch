#include "pulse_oximeter.h"

PulseOximeter::PulseOximeter(i2c *i2c_driver, SemaphoreHandle_t *i2c_mutex)
{
    this->i2c_module = i2c_driver;
    this->i2c_mutex = i2c_mutex;
}

esp_err_t PulseOximeter::i2c_read(PUL_OXI_DEV_ADDR device_addres, uint8_t reg,uint8_t* buffer,size_t data_size)
{
    esp_err_t ret;
    xSemaphoreTake(*i2c_mutex, portMAX_DELAY);
    ret = i2c_module->read(device_addres, reg, buffer, data_size);
    xSemaphoreGive(*i2c_mutex);
    return ret;
}

esp_err_t PulseOximeter::i2c_write(PUL_OXI_DEV_ADDR device_addr, uint8_t reg,uint8_t* buffer,size_t data_size)
{
    esp_err_t ret;
    xSemaphoreTake(*i2c_mutex, portMAX_DELAY);
    ret = i2c_module->write(device_addr, reg, buffer, data_size);
    xSemaphoreGive(*i2c_mutex);
    return ret;
}

PulseOximeter::~PulseOximeter()
{
}

esp_err_t PulseOximeter::config()
{
    uint8_t rx;
    uint8_t tx;

    // Config Interrupt
    tx = 0b00000000;
    i2c_write(PUL_OXI_ADDR, PUL_OXI_INT_STAT1, &tx, 1);
    i2c_write(PUL_OXI_ADDR, PUL_OXI_INT_STAT2, &tx, 1);
    i2c_write(PUL_OXI_ADDR, PUL_OXI_INT_ENABLE1, &tx, 1);
    i2c_write(PUL_OXI_ADDR, PUL_OXI_INT_ENABLE2, &tx, 1);

    // Config FIFO
    tx =  PUL_OXI_SMP_AVE_2 << 5 | 
          PUL_OXI_FIFO_ROLLOVER_EN << 4 | 
          PUL_OXI_FIFO_A_FULL << 0;
    i2c_write(PUL_OXI_ADDR, PUL_OXI_FIFO_CONFIG, &tx, 1);

    // Config Mode
    tx = PUL_OXI_SHDN_OFF << 7 | 
         PUL_OXI_RESET_OFF << 6 | 
         PUL_OXI_MODE_BOTH << 2 ;
    i2c_write(PUL_OXI_ADDR, PUL_OXI_MODE_CONFIG, &tx, 1);

    // Config SpO2
    tx = PUL_OXI_ADC_RGE_4096 << 5 | 
         PUL_OXI_SPO2_SR_50 << 2 | 
         PUL_OXI_LED_PW_69 << 0;
    i2c_write(PUL_OXI_ADDR, PUL_OXI_SPO2_CONFIG, &tx, 1);

    // Config LED
    tx = PUL_OXI_LED_CURR_25 << 0;
    i2c_write(PUL_OXI_ADDR, PUL_OXI_LED1_PA, &tx, 1);
    i2c_write(PUL_OXI_ADDR, PUL_OXI_LED2_PA, &tx, 1);

    // Config Multi-LED Mode
    tx = PUL_OXI_SLOT1 << 2 | 
         PUL_OXI_SLOT2 << 4;
    i2c_write(PUL_OXI_ADDR, PUL_OXI_MULTI_LED_CTRL1, &tx, 1);

    tx = PUL_OXI_SLOT3 << 2 | 
         PUL_OXI_SLOT4 << 4;
    i2c_write(PUL_OXI_ADDR, PUL_OXI_MULTI_LED_CTRL2, &tx, 1);

    // Config Temperature
    tx = PUL_OXI_TEMP_EN ;    
    i2c_write(PUL_OXI_ADDR, PUL_OXI_TEMP_INT, &tx, 1);


}