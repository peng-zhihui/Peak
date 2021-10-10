# MPU9250
Arduino library for [MPU9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/) Nine-Axis (Gyro + Accelerometer + Compass) MEMS MotionTracking™ Device

This library is based on the [great work](https://github.com/kriswiner/MPU9250) by [kriswiner](https://github.com/kriswiner), and re-writen for the simple usage.

## Usage

### Simple Measurement

``` C++
#include "MPU9250.h"

MPU9250 mpu; // You can also use MPU9255 as is

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    mpu.setup(0x68);  // change to your own address
}

void loop() {
    if (mpu.update()) {
        Serial.print(mpu.getYaw()); Serial.print(", ");
        Serial.print(mpu.getPitch()); Serial.print(", ");
        Serial.println(mpu.getRoll());
    }
}
```

### Calibration

- accel/gyro/mag offsets are **NOT stored** to register if the MPU has powered off ([app note](https://www.digikey.com/en/pdf/i/invensense/mpu-hardware-offset-registers))
- need to **set all offsets at every bootup by yourself** (or calibrate at every bootup)
- device should be stay still during accel/gyro calibration
- round device around during mag calibration

``` C++
#include "MPU9250.h"

MPU9250 mpu; // You can also use MPU9255 as is

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    mpu.setup(0x68);  // change to your own address

    delay(5000);

    // calibrate anytime you want to
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();
}

void loop() { }
```

### Coordinate

The coordinate of quaternion and roll/pitch/yaw angles are basedd on airplane coordinate (Right-Handed, X-forward, Z-down). On the other hand, the coordinate of euler angle is based on the axes of acceleration and gyro sensors (Right-Handed, X-forward, Z-up).Please use `getEulerX/Y/Z()` for euler angles and `getRoll/Pitch/Yaw()` for airplane coordinate angles.


## Other Settings

### I2C Address

You must set your own address based on A0, A1, A2 setting as:

``` C++
mpu.setup(0x70);
```


### Customize MPU9250 Configuration

You can set your own setting using `MPU9250Setting` struct as:

``` C++
MPU9250Setting setting;
setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
setting.gyro_fchoice = 0x03;
setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
setting.accel_fchoice = 0x01;
setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

mpu.setup(0x68, setting);
```

See `custom_setting.ino` example for detail.


#### List of Settings

```C++
enum class ACCEL_FS_SEL { A2G, A4G, A8G, A16G };
enum class GYRO_FS_SEL { G250DPS, G500DPS, G1000DPS, G2000DPS };
enum class MAG_OUTPUT_BITS { M14BITS, M16BITS };

enum class FIFO_SAMPLE_RATE : uint8_t {
    SMPL_1000HZ,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ,
};

enum class GYRO_DLPF_CFG : uint8_t {
    DLPF_250HZ,
    DLPF_184HZ,
    DLPF_92HZ,
    DLPF_41HZ,
    DLPF_20HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_3600HZ,
};

enum class ACCEL_DLPF_CFG : uint8_t {
    DLPF_218HZ_0,
    DLPF_218HZ_1,
    DLPF_99HZ,
    DLPF_45HZ,
    DLPF_21HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_420HZ,
};

struct MPU9250Setting {
    ACCEL_FS_SEL     accel_fs_sel {ACCEL_FS_SEL::A16G};
    GYRO_FS_SEL      gyro_fs_sel {GYRO_FS_SEL::G2000DPS};
    MAG_OUTPUT_BITS  mag_output_bits {MAG_OUTPUT_BITS::M16BITS};
    FIFO_SAMPLE_RATE fifo_sample_rate {FIFO_SAMPLE_RATE::SMPL_200HZ};
    uint8_t          gyro_fchoice {0x03};
    GYRO_DLPF_CFG    gyro_dlpf_cfg {GYRO_DLPF_CFG::DLPF_41HZ};
    uint8_t          accel_fchoice {0x01};
    ACCEL_DLPF_CFG   accel_dlpf_cfg {ACCEL_DLPF_CFG::DLPF_45HZ};
};
```

#### Magnetic Declination

Magnetic declination should be set depending on where you are to get accurate data.
To set it, use this method.

```C++
mpu.setMagneticDeclination(value);
```

You can find magnetic declination in your city [here](http://www.magnetic-declination.com/).

For more details, see [wiki](https://en.wikipedia.org/wiki/Magnetic_declination).


### Quaternion Filter

You can choose quaternion filter using `void selectFilter(QuatFilterSel sel)`. Available quaternion filters are listed below.

```C++
enum class QuatFilterSel {
    NONE,
    MADGWICK, // default
    MAHONY,
};
```

You can also change the calculate iterations for the filter as follows. The default value is 1. Generally 10-20 is good for stable yaw estimation. Please see [this discussion](https://github.com/kriswiner/MPU9250/issues/420) for the detail.

```C++
mpu.setFilterIterations(10);
```


### Other I2C library

You can use other I2C library e.g. [SoftWire](https://github.com/stevemarple/SoftWire).

``` C++
MPU9250_<SoftWire> mpu;
SoftWire sw(SDA, SCL);

// you need setting struct
MPU9250Setting setting;

// in setup()
mpu.setup(0x70, setting, sw);
```


## About I2C Errors

Sometimes this library shows the I2C error number if your connection is not correct. It's based on the I2C error number which is reported by the `Wire.endTransmission()`. It returns following number based on the result of I2C data transmission.

> 0:success
> 1:data too long to fit in transmit buffer
> 2:received NACK on transmit of address
> 3:received NACK on transmit of data
> 4:other error

If you have such errors, please check your hardware connection and I2C address setting first. Please refer [Wire.endTransmission() reference](https://www.arduino.cc/en/Reference/WireEndTransmission) for these errors, and [section 2.3 of this explanation](https://www.ti.com/lit/an/slva704/slva704.pdf) for ACK and NACK.


## APIs

``` C++
bool setup(const uint8_t addr, const MPU9250Setting& setting, WireType& w = Wire);
void verbose(const bool b);
void ahrs(const bool b);
void calibrateAccelGyro();
void calibrateMag();
bool isConnected();
bool isConnectedMPU9250();
bool isConnectedAK8963();
bool available();
bool update();

float getRoll() const;
float getPitch() const;
float getYaw() const;

float getEulerX() const;
float getEulerY() const;
float getEulerZ() const;

float getQuaternionX() const;
float getQuaternionY() const;
float getQuaternionZ() const;
float getQuaternionW() const;

float getAcc(const uint8_t i) const;
float getGyro(const uint8_t i) const;
float getMag(const uint8_t i) const;
float getLinearAcc(const uint8_t i) const;

float getAccX() const;
float getAccY() const;
float getAccZ() const;
float getGyroX() const;
float getGyroY() const;
float getGyroZ() const;
float getMagX() const;
float getMagY() const;
float getMagZ() const;
float getLinearAccX() const;
float getLinearAccY() const;
float getLinearAccZ() const;

float getAccBias(const uint8_t i) const;
float getGyroBias(const uint8_t i) const;
float getMagBias(const uint8_t i) const;
float getMagScale(const uint8_t i) const;

float getAccBiasX() const;
float getAccBiasY() const;
float getAccBiasZ() const;
float getGyroBiasX() const;
float getGyroBiasY() const;
float getGyroBiasZ() const;
float getMagBiasX() const;
float getMagBiasY() const;
float getMagBiasZ() const;
float getMagScaleX() const;
float getMagScaleY() const;
float getMagScaleZ() const;

float getTemperature() const;

void setAccBias(const float x, const float y, const float z);
void setGyroBias(const float x, const float y, const float z);
void setMagBias(const float x, const float y, const float z);
void setMagScale(const float x, const float y, const float z);
void setMagneticDeclination(const float d);

void selectFilter(QuatFilterSel sel);
void setFilterIterations(const size_t n);

bool selftest();
```

## License

MIT

## Acknowledgments / Contributor

- [Yuta Asai](https://github.com/asaiyuta)
