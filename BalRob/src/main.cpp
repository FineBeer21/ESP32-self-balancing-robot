#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <FastAccelStepper.h>

//def pins
const int DIR_PIN1 = 25;
const int DIR_PIN2 = 32;
const int STEP_PIN1 = 13;
const int STEP_PIN2 = 14;

// Engine vars
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *leftMotor = NULL;
FastAccelStepper *rightMotor = NULL;

int motorAcceleration = 15000; // Acceleration in steps/s^2, you can change this value to adjust the acceleration of the motors

MPU6050 mpu(Wire);

// Freq timers
float dt = 0.005;
float sTimer = dt * 1000000.0f; // 5000 micro seconds = 200 HZ
unsigned long lastMeasured = 0;

float printTimer = 20000.0f;

// PID coefficient
float Kp = 7.0f;
float Ki = 0.2f;
float Kd = 0.1f;

float lastError = 0.0f;
float lastDer = 0.0f;

float lpFilter = 0.05f;
float integralSum = 0.0f;

int maxFreq = 12000;

float angleOffset = -0.85f; // Usefull if center of gravity is not 0.0 or when wanting to move forward/backward

float normalizedFreq = 250.0f;

// For control - using teleplot extention
struct teleplotData
{
    float currentAngle = 0.0f;
    float currentGyro = 0.0f;
    float currentPID = 0.0f;
    float outputFreq = 0.0f;
    u_int32_t stepsCount = 0;
};

teleplotData teleDa;

// PID function
float PID (float setPoint, float currentRoll, float &lastError, float &integralSum, float gyroRate, float dt = 0.005f)
{
    float error = (setPoint - currentRoll); 
    if(fabs(error) < 1.0f) error *= fabs(error);
        
    integralSum = (integralSum + error * dt) * 0.999f;
    lastDer = lpFilter * gyroRate + (1.0f - lpFilter) * lastDer;
    teleDa.currentGyro = lastDer; // Updating for teleplot

    float rawPID =  (Kp * error) + (Ki * integralSum) + (Kd * lastDer);
    return rawPID;
}

// Motor control function
void fastAccMotor(float calculatedPID, FastAccelStepper *motor){
    float outputFreq = calculatedPID * normalizedFreq;
    static bool movingForward; // making sure we don't call runForward() or runBackward() multiple times in a row, if you want to add rotation theres need to be one for each motor

    teleDa.outputFreq = outputFreq; // Updating for teleplot
    
    uint32_t absFreq = (uint32_t)abs(outputFreq); 

    if(absFreq > maxFreq) absFreq = maxFreq;
    if (absFreq < 20) { 
    motor->setSpeedInHz(0); // stop the motor if the frequency is too low
    motor->stopMove();
    } else {
        motor->setSpeedInHz(absFreq);

        bool shouldMoveForward = (outputFreq > 0);

        if (shouldMoveForward && !movingForward) {
        motor->runForward();
        movingForward = true;
        } else if(!shouldMoveForward && movingForward) {
        motor->runBackward();
        movingForward = false;
        }   
    }
}


void setup() {
    Serial.begin(921600); // reccommended to use 921600 for teleplot
    Wire.begin(); 
    Wire.setTimeOut(10); // I2C timout in ms

    byte status = mpu.begin();
    Serial.print(F("loading mpu... Status is: "));
    Serial.println(status);

    while (status!=0) {}

    Serial.println(F("MPU ON, calc offset"));

    // Set offsets for MPU6050, these values should be calibrated for your specific MPU6050 module
    mpu.setAccOffsets(-0.01f, 0.07f, -2.06f);
    mpu.setGyroOffsets(-1.98f, 0.41f, -0.32f);

    // Engine setup
    engine.init();

    leftMotor = engine.stepperConnectToPin(STEP_PIN1);
  if (leftMotor) {
    leftMotor->setDirectionPin(DIR_PIN1);
    leftMotor->setAcceleration(motorAcceleration);
  }

  rightMotor = engine.stepperConnectToPin(STEP_PIN2);
  if (rightMotor) {
    rightMotor->setDirectionPin(DIR_PIN2);
    rightMotor->setAcceleration(motorAcceleration);
  }
}

void loop(){
    unsigned long currentMicro = micros();

    if(currentMicro - lastMeasured >= sTimer){
        float dty = (currentMicro - lastMeasured) / 1000000.0f;

        mpu.update();

        float accAngle = mpu.getAccAngleY();
        float gyroRate = -mpu.getGyroY();

        // Complementary filter to combine accelerometer and gyroscope data, I have inverted the gyroRate because the MPU6050 is mounted upside down on my robot, so the gyro readings are inverted
        static float filteredAngle = 0.0f;
        filteredAngle = 0.98f * (filteredAngle + gyroRate * dty) + 0.02f * accAngle; 

        float calculatedPID = PID(angleOffset, filteredAngle, lastError, integralSum, -gyroRate, dty);

        // Update teleplot data
        teleDa.currentAngle = filteredAngle;
        teleDa.currentPID = calculatedPID;

        // Update motors
        fastAccMotor(calculatedPID, leftMotor);
        fastAccMotor(-calculatedPID, rightMotor);

        lastMeasured = currentMicro;
    }

    // Teleplot data output
    static long lastPrinted = 0;
    if(currentMicro - lastPrinted > printTimer){
        lastPrinted = currentMicro;

        Serial.printf(">MPU_roll:%.2f\n", teleDa.currentAngle);
        Serial.printf(">MPU_gyro:%.2f\n", teleDa.currentGyro);
        Serial.printf(">PID:%.2f\n", teleDa.currentPID);
        Serial.printf(">outputFreq:%.2f\n", teleDa.outputFreq);
    }
}