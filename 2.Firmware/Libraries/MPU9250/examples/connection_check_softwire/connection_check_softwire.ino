#include <MPU9250.h>
#include <SoftWire.h>

MPU9250_<SoftWire> mpu;
SoftWire sw(SDA, SCL);  // change to your own pins
char swTxBuffer[64];    // These buffers must be at least as large as
char swRxBuffer[64];    // the largest read or write you perform.

uint8_t addrs[7] = {0};
uint8_t device_count = 0;

template <typename WireType = TwoWire>
void scan_mpu(WireType& wire = Wire) {
    Serial.println("Searching for i2c devices...");
    device_count = 0;
    for (uint8_t i = 0x68; i < 0x70; ++i) {
        wire.beginTransmission(i);
        if (wire.endTransmission() == 0) {
            addrs[device_count++] = i;
            delay(10);
        }
    }
    Serial.print("Found ");
    Serial.print(device_count, DEC);
    Serial.println(" I2C devices");

    Serial.print("I2C addresses are: ");
    for (uint8_t i = 0; i < device_count; ++i) {
        Serial.print("0x");
        Serial.print(addrs[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

template <typename WireType = TwoWire>
uint8_t readByte(uint8_t address, uint8_t subAddress, WireType& wire = Wire) {
    uint8_t data = 0;
    wire.beginTransmission(address);
    wire.write(subAddress);
    // wire.endTransmission(false); // TODO: repeated start fails in SoftWire
    wire.endTransmission();
    wire.requestFrom(address, (size_t)1);
    if (wire.available()) data = wire.read();
    return data;
}

void setup() {
    Serial.begin(115200);
    Serial.flush();

    sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
    sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
    sw.setDelay_us(5);
    sw.begin();

    delay(2000);

    scan_mpu(sw);

    if (device_count == 0) {
        Serial.println("No device found on I2C bus. Please check your hardware connection");
        while (1)
            ;
    }

    // check WHO_AM_I address of MPU
    for (uint8_t i = 0; i < device_count; ++i) {
        Serial.print("I2C address 0x");
        Serial.print(addrs[i], HEX);
        byte ca = readByte(addrs[i], WHO_AM_I_MPU9250, sw);
        if (ca == MPU9250_WHOAMI_DEFAULT_VALUE) {
            Serial.println(" is MPU9250 and ready to use");
        } else if (ca == MPU9255_WHOAMI_DEFAULT_VALUE) {
            Serial.println(" is MPU9255 and ready to use");
        } else if (ca == MPU6500_WHOAMI_DEFAULT_VALUE) {
            Serial.println(" is MPU6500 and ready to use");
        } else {
            Serial.println(" is not MPU series");
            Serial.print("WHO_AM_I is ");
            Serial.println(ca, HEX);
            Serial.println("Please use correct device");
        }
    }
}

void loop() {
}
