# nano-i2c-uart-sw-bridge-library
Arduino Nano Library for I2C to UART Bridge 

## Example

```NanoI2cUartTempSensorDemo.ino
#include "I2cUartSwBridge.h"

// target device:HTU21D(i2c temperature sensor)
#define ADDR 0x40
#define CMD_READ_TEMP_HOLD 0xe3
#define CMD_READ_HUM_HOLD 0xe5
#define CMD_READ_TEMP_NOHOLD 0xf3
#define CMD_READ_HUM_NOHOLD 0xf5
#define CMD_WRITE_USER_REG 0xe6
#define CMD_READ_USER_REG 0xe7
#define CMD_SOFT_RESET 0xfe

I2cUartSwBridge i2cuart;

void setup()
{
    Serial.begin(115200);
    i2cuart.init();
}

void loop()
{
    i2cuart.write_byte(ADDR, CMD_SOFT_RESET);

    i2cuart.write_byte(ADDR, CMD_READ_TEMP_HOLD);
    delay(100);
    data = i2cuart.read_i2c_block_data(ADDR, CMD_READ_TEMP_HOLD, 3);
    delay(100);
    uint16_t tmp = data[0]
    tmp = tmp << 8;
    tmp = tmp | data[1];
    tmp = tmp & 0xFFFC;

    float temp = -46.85 + (175.72 * tmp / 65536)

    Serial.printf("Temperature   : {:.2f}", temp)

    delay(1000);
}

```
