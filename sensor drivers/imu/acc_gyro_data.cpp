# include "bmi08x.h"
# include <iostream>
using namespace std;

int main()
{
    uint8_t acc_dev_addr = BMI08X_ACCEL_I2C_ADDR_PRIMARY; /* User has define this macro depends on the I2C slave address */
    uint8_t gyro_dev_addr = BMI08X_GYRO_I2C_ADDR_PRIMARY; /* User has define this macro depends on the I2C slave address */
    
    bmi08x_dev dev = 
    {
        .intf_ptr_accel = &acc_dev_addr,
        .intf_ptr_gyro = &gyro_dev_addr,
        .intf = BMI08X_I2C_INTF,  
        // .read = user_i2c_read,  
        // .write = user_i2c_write,  
        // .delay_ms = user_delay,
        .variant = BMI088_VARIANT
    };

    int8_t rslt_acc = bmi08a_init(&dev);
    int8_t rslt_gyro = bmi08g_init(&dev);

    bmi08x_sensor_data acc_data;

    uint8_t BMI08X_ACCEL_CHIP_ID_REG = 0;
    if(rslt_acc == BMI08X_OK) 
    {
        /* Read accel chip id */
        rslt_acc = bmi08a_get_regs(BMI08X_ACCEL_CHIP_ID_REG, &data, 1, &dev); // what is data here?
    }

    rslt_acc = bmi08a_get_data(&acc_data, &dev);






}