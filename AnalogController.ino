#include "quaternionFilters.h"
#include "MPU9250.h"
MPU9250 myIMU;

#ifdef SOFT_IIC
#include "SoftWire.h"
SoftWire Wire;
#else
#include <Wire.h>
#endif

#include "HID-Project.h"

//#define DEBUG_FLAG
//#define DEBUG_MPU
//#define DEBUG_ANALOG

#define KEY_L1 4
#define KEY_L2 5
#define KEY_L3 6
#define KEY_R1 7
#define KEY_R2 8
#define KEY_R3 9

#define SLIDER_UP A0
#define SLIDER_DOWN A1
#define SLIDER_LEFT A2
#define SLIDER_RIGHT A3

#define LED_R 14
#define LED_Y 15
#define LED_G 16

#define MODE_1 10
#define MODE_2 0
#define MODE_3 1

bool first = true;
float data[3];
uint16_t count = 50;

void _writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t _readByte(uint8_t address, uint8_t subAddress) {
    uint8_t data;                            // `data` will store the register data
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t)1);   // Read one byte from slave register address
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void _readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count);  // Read bytes from slave register address
    while (Wire.available()) {
        dest[i++] = Wire.read();
    }                                  // Put read results in the Rx buffer
}

void setup() {
    Wire.begin();
#ifdef DEBUG_FLAG
    Serial.begin(115200);
#endif
    myIMU.wB = &_writeByte;
    myIMU.rB = &_readByte;
    myIMU.rBs = &_readBytes;

    byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    if (c == 0x71) {
        myIMU.MPU9250SelfTest(myIMU.SelfTest);
        myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
        myIMU.initMPU9250();
        myIMU.initAK8963(myIMU.magCalibration);
    } else {
        while (1);
    }

    pinMode(KEY_L1, INPUT_PULLUP); pinMode(KEY_L2, INPUT_PULLUP); pinMode(KEY_L3, INPUT_PULLUP);
    pinMode(KEY_R1, INPUT_PULLUP); pinMode(KEY_R2, INPUT_PULLUP); pinMode(KEY_R3, INPUT_PULLUP);

    pinMode(LED_R, OUTPUT); digitalWrite(LED_R, LOW);
    pinMode(LED_Y, OUTPUT); digitalWrite(LED_Y, LOW);
    pinMode(LED_G, OUTPUT); digitalWrite(LED_G, LOW);

    pinMode(MODE_1, INPUT_PULLUP); pinMode(MODE_2, INPUT_PULLUP); pinMode(MODE_3, INPUT_PULLUP);
    
    Gamepad.begin();
}

void loop() {
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
        myIMU.readAccelData(myIMU.accelCount);
        myIMU.getAres();
        myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes;
        myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes;
        myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes;

        myIMU.readGyroData(myIMU.gyroCount);
        myIMU.getGres();
        myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
        myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
        myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

        myIMU.readMagData(myIMU.magCount);
        myIMU.getMres();
        myIMU.magbias[0] = +470.;
        myIMU.magbias[1] = +120.;
        myIMU.magbias[2] = +125.;
        myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.magCalibration[0] -
            myIMU.magbias[0];
        myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.magCalibration[1] -
            myIMU.magbias[1];
        myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.magCalibration[2] -
            myIMU.magbias[2];
    }

    myIMU.updateTime();

    MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
        myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
        myIMU.mx, myIMU.mz, myIMU.deltat);

    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 10) {
        myIMU.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
            *(getQ() + 3)), *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1)
            - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
        myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
            *(getQ() + 2)));
        myIMU.roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) *
            *(getQ() + 3)), *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1)
            - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
        myIMU.pitch *= RAD_TO_DEG;
        myIMU.yaw *= RAD_TO_DEG;

        myIMU.yaw -= 8.5; //ref: http://www.ngdc.noaa.gov/geomag-web/#declination
        myIMU.roll *= RAD_TO_DEG;

        if (count > 0) count -= 1;
        else if (first) {
            data[0] = myIMU.yaw;
            data[1] = myIMU.pitch;
            data[2] = myIMU.roll;
            first = false;
        }
        else {
            myIMU.yaw -= data[0];
            myIMU.pitch -= data[1];
            myIMU.roll -= data[2];
        }
        myIMU.count = millis();
    }

    work();
}

int t = 0; bool ctrl = false;

float cal[] = { 0, 0, 0 };
float HALF_ANGLE = 75.0F;

inline float getYaw() { return myIMU.yaw - cal[0]; }
inline float getPitch() { return myIMU.pitch - cal[1]; }
inline float getRoll() { return myIMU.roll - cal[2]; }

float fix(float angle) {
    angle += 180.0F;
    while (angle < 0.0F) angle += 360.0F;
    while (angle > 360.0F) angle -= 360.0F;
    if (angle < (180.0F - HALF_ANGLE)) angle = (180.0F - HALF_ANGLE);
    if (angle > (180.0F + HALF_ANGLE)) angle = (180.0F + HALF_ANGLE);
    return angle;
}

int mapf(float in, int to) {
    in -= 180.0F;
    float result = (float) to / HALF_ANGLE * in;
    return result;
}

#define GKEY(k) (digitalRead(k) == LOW)

void setLED(uint8_t pin) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_Y, LOW);
    digitalWrite(LED_G, LOW);

    digitalWrite(pin, HIGH);
}

void work() {
    #ifdef DEBUG_FLAG
        #ifdef DEBUG_MPU
            Serial.print("Yaw, Pitch, Roll:\t");
            Serial.print(fix(getYaw()), 2);
            Serial.print(",\t");
            Serial.print(fix(getPitch()), 2);
            Serial.print(",\t");
            Serial.print(fix(getRoll()), 2);
            Serial.print(" || rate=\t");
            Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
            Serial.println(" Hz");
        #endif
        #ifdef DEBUG_ANALOG
            Serial.print("Analog:\t");
            Serial.print(analogRead(SLIDER_UP));
            Serial.print(",\t");
            Serial.print(analogRead(SLIDER_DOWN));
            Serial.print(",\t");
            Serial.print(analogRead(SLIDER_LEFT));
            Serial.print(",\t");
            Serial.print(analogRead(SLIDER_RIGHT));
            Serial.println();
        #endif
    #endif
    
    if (ctrl) {
        if (GKEY(KEY_L3) && GKEY(KEY_R3)) {
            cal[0] = myIMU.yaw;
            cal[1] = myIMU.pitch;
            cal[2] = myIMU.roll;
            
            return;
        }

        float mpuX, mpuY, mpuZ;
        float adcX, adcY, adcZ;

        mpuX = mapf(fix(getRoll()), 32767);
        mpuY = mapf(fix(getPitch()), 32767);
        mpuZ = mapf(fix(getYaw()), -127);

        adcX = analogRead(SLIDER_UP) * 64 - 32767;
        adcY = (analogRead(SLIDER_LEFT) - analogRead(SLIDER_RIGHT)) * 64 - 32767;
        adcZ = -(analogRead(SLIDER_DOWN) / 4 - 127);

        int x, y, z;
        
        if (GKEY(MODE_1)) {
            setLED(LED_Y);
            x = mpuX;
            y = mpuY;
            z = mpuZ;
        } else if (GKEY(MODE_2)) {
            setLED(LED_R);
            x = adcX;
            y = mpuX;
            z = adcZ;
        } else if (GKEY(MODE_3)) {
            setLED(LED_R);
            x = adcX;
            y = mpuY;
            z = adcZ;
        } else {
            setLED(LED_G);
            x = adcX;
            y = adcY;
            z = adcZ;
        }
        
        if (x > 32767) x = 32767; if (x < -32767) x = -32767;
        if (y > 32767) y = 32767; if (y < -32767) y = -32767;
        if (z > 127) z = 127; if (z < -127) z = -127;
        
        Gamepad.xAxis(x);
        Gamepad.yAxis(y);
        Gamepad.zAxis(z);

        if (GKEY(KEY_L1)) Gamepad.press(1);
        else Gamepad.release(1);
        if (GKEY(KEY_L2)) Gamepad.press(2);
        else Gamepad.release(2);
        if (GKEY(KEY_L3)) Gamepad.press(3);
        else Gamepad.release(3);
        if (GKEY(KEY_R1)) Gamepad.press(4);
        else Gamepad.release(4);
        if (GKEY(KEY_R2)) Gamepad.press(5);
        else Gamepad.release(5);
        if (GKEY(KEY_R3)) Gamepad.press(6);
        else Gamepad.release(6);
        
        Gamepad.write();
    } else {
        if (t == 0) t = millis();
        else {
            if (millis() - t > 1000) {
                cal[0] = myIMU.yaw;
                cal[1] = myIMU.pitch;
                cal[2] = myIMU.roll;
                ctrl = true;
            }
        }
    }
    
}

