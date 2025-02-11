/*
 *  i2c.cpp
 *
 *  Created on: 16 jan 2021
 *      Author: Luan Amaral
 */

#include <stdio.h>
#include "i2c.h"
#include <typeinfo>

i2c::i2c()
{
}

i2c::~i2c()
{
    //TODO
}

esp_err_t i2c::init(int data_pin, int clock_pin, uint32_t freq, i2c_port_t _port)
{
    port = _port;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = data_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = clock_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(port, &conf);
    if (err == ESP_FAIL)
    {
        return err;
    }
    return i2c_driver_install(port, conf.mode, rx_buff, tx_buff, 0);
}

esp_err_t i2c::write(uint8_t address,uint8_t regist, uint8_t *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true); 
    i2c_master_write_byte(cmd, regist, true);
    i2c_master_write(cmd, data, size, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_PERIOD_MS); /*!<maximum wait = 1s */
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c::read(uint8_t address, uint8_t regist, uint8_t *data, size_t size)
{
    if (size ==0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regist, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address<<1) | I2C_MASTER_READ, true);
    if(size > 1)
    {
        i2c_master_read(cmd, data, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_PERIOD_MS); /*!<maximum wait = 1s */
    i2c_cmd_link_delete(cmd);
    return ret;
}