#include "functions.h"

static MPU6050 mpu;

static bool dmpReady = false;
static uint8_t devStatus;
static uint16_t packetSize;
static uint8_t fifoBuffer[64];

static Quaternion q;
static VectorFloat gravity;
static float ypr[3];

static float yawOffset = 0.0, pitchOffset = 0.0, rollOffset = 0.0;
static float offset_x = 0.0, offset_y = 0.0, offset_z = 0.0;

void initMPU() {
    mpu.initialize();
    
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        //calibrateMPU();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void getAccelerometerData() { 
    if (!dmpReady || SD_state == 0) return;

    int16_t ax, ay, az;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.getAcceleration(&ax, &ay, &az);

        // Apply offsets
        float accel_x = ax / 16384.0 - offset_x;
        float accel_y = ay / 16384.0 - offset_y;
        float accel_z = az / 16384.0 - offset_z;
        float yaw = (ypr[0] * 180 / M_PI) - yawOffset;
        float pitch = (ypr[1] * 180 / M_PI) - pitchOffset;
        float roll = (ypr[2] * 180 / M_PI) - rollOffset;
       
        char data[75];
        sprintf(data, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", accel_x, accel_y, accel_z, yaw, pitch, roll);
        logToSD(0x100, 0x123, data);
    }
}

void calibrateMPU() {
    const int samples = 2000;
    const float learningRate = 0.1;
    const float threshold = 0.02;

    float yawSum = 0, pitchSum = 0, rollSum = 0;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < samples; i++) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            int16_t ax, ay, az;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.getAcceleration(&ax, &ay, &az);

            sum_x += ax;
            sum_y += ay;
            sum_z += az;
            
            yawSum += ypr[0] * 180 / M_PI;
            pitchSum += ypr[1] * 180 / M_PI;
            rollSum += ypr[2] * 180 / M_PI;
        }
        delay(3);
    }

    yawOffset = yawSum / samples;
    pitchOffset = pitchSum / samples;
    rollOffset = rollSum / samples;

    offset_x = (sum_x / samples) / 16384.0;
    offset_y = (sum_y / samples) / 16384.0;
    offset_z = (sum_z / samples) / 16384.0;
    

    // Refinement Loop
    for (int i = 0; i < 3000; i++) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            float currentYaw = ypr[0] * 180 / M_PI - yawOffset;
            float currentPitch = ypr[1] * 180 / M_PI - pitchOffset;
            float currentRoll = ypr[2] * 180 / M_PI - rollOffset;

            // Adjust offsets gradually
            yawOffset += currentYaw * learningRate;
            pitchOffset += currentPitch * learningRate;
            rollOffset += currentRoll * learningRate;

            // Stop adjusting if all values are within the threshold
            if (abs(currentYaw) < threshold && abs(currentPitch) < threshold && abs(currentRoll) < threshold) {
                Serial.println(F("Final Calibration Complete!"));
                break;
            }
        }
        delay(3);
    }
}
