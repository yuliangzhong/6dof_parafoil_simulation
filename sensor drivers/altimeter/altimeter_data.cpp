# include "bmp3.h"
# include <iostream>
using namespace std;

int main()
{
    bmp3_dev dev = 
    {
        .chip_id = 0,
        .intf = BMP3_I2C_INTF,  
        // .read = user_i2c_read,  
        // .write = user_i2c_write,  
        // .delay_ms = user_delay,
    };
}