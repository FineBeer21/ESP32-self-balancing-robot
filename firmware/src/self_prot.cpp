#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

//def pins
const int DIR_PIN1 = 25;
const int DIR_PIN2 = 32;
const int STEP_PIN1 = 13;
const int STEP_PIN2 = 14;

// Engine protocol vars
volatile int32_t stepCount = 0;
volatile int8_t motorDirection = 1; // 1 or -1
volatile uint32_t targetInterval = 1000;
volatile bool isRunning = true; // Control if needed

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *timer = NULL;

MPU6050 mpu(Wire);

// Time vars
float dt = 0.001f;
float measureTimer = dt * 1000000.0f;
unsigned long lastMeasured = 0;

unsigned long lastPrinted = 0;
float printTimer = 20000.0f; // in us it's 50HZ


float normalizedFreq = 40.0f;

float angleOffset = -0.85f; // Usefull if center of gravity is not 0.0 or when wanting to move forward/backward



// PID coefficient
float kp = 4.0f;
float ki = 0.0f;
float kd = 0.0f;

float integralSum = 0.0f;

// Teleplot 
struct teleplotData
{
    float currentAngle = 0.0f;
    float currentGyro = 0.0f;
    float currentPID = 0.0f;
    u_int32_t stepsCount = 0;
};

volatile teleplotData teleDa;


//IRAM ISR moving motor one step
void IRAM_ATTR onTimer(){
    if(!isRunning) return;

    digitalWrite(STEP_PIN1, HIGH);
    digitalWrite(STEP_PIN2, HIGH);
    ets_delay_us(2);
    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);

    stepCount += motorDirection;
}

void setSpeed(float freq, int8_t dir){
    // Deadband - making sure we dont divide by zero
    if (freq < 1.0f) freq = 1.0f;
    
    uint32_t intervalSpeedUs = (uint32_t)(1000000.0f / freq);

    if(intervalSpeedUs < 20) intervalSpeedUs = 20;

    portENTER_CRITICAL(&timerMux);
    isRunning = true;
    targetInterval = intervalSpeedUs;
    if (motorDirection != dir) {
        digitalWrite(DIR_PIN1, dir > 0 ? HIGH : LOW);
        digitalWrite(DIR_PIN2, dir < 0 ? HIGH : LOW);
        motorDirection = dir;
    }
    timerAlarmWrite(timer, targetInterval, true);
    portEXIT_CRITICAL(&timerMux);
}

// For control
int32_t getExactStep(){
    int32_t steps;
    portENTER_CRITICAL(&timerMux);
    steps = stepCount;
    portEXIT_CRITICAL(&timerMux);
    return steps;
}

float PID (float setPoint, float currentRoll, float gyroRate, float realDT)
{
    float NormalizedError = (setPoint - currentRoll); 
    integralSum = (integralSum + NormalizedError * realDT) * 0.999f;

    float rawPID =  (kp * NormalizedError) + (ki * integralSum) - (kd * gyroRate);
    return rawPID;
}

void sensorLoop(unsigned long currentMicros){
    if(currentMicros - lastMeasured >= measureTimer){
        float dty = (currentMicros - lastMeasured) / 1000000.0f;
        lastMeasured = currentMicros;

        float accAngle = mpu.getAccAngleY();
        float gyroRate = -mpu.getGyroY();

        static float filteredAngle = 0.0f;
        filteredAngle = 0.98f * (filteredAngle + gyroRate * dty) + 0.02f * accAngle;


        float calculatedPID = PID(angleOffset, filteredAngle, gyroRate, dty);

        // Change motor freq
        float freq = fabs(calculatedPID * normalizedFreq);
        setSpeed(freq, (calculatedPID > 0) ? 1 : -1);

        teleDa.currentAngle = filteredAngle;
        teleDa.currentGyro = gyroRate;

    }
}

void writeTeleplotLoop(unsigned long currentMicro){
    if(currentMicro - lastPrinted >= printTimer){
        lastPrinted = currentMicro;

        //updating steps
        int32_t steps;
        portENTER_CRITICAL(&timerMux);
        steps = stepCount;
        portEXIT_CRITICAL(&timerMux);
        teleDa.stepsCount = steps;

        Serial.printf(">MPU_roll:%.2f\n", teleDa.currentAngle);
        Serial.printf(">MPU_gyro:%.2f\n", teleDa.currentGyro);
        Serial.printf(">Motors_steps:%d\n", teleDa.stepsCount);
    }
}

void setup(){
    Serial.begin(921600);

    // MPU setup
    Wire.begin();
    Wire.setTimeOut(10);

    byte status = mpu.begin();
    Serial.print(F("loading mpu... Status is: "));
    Serial.println(status);

    while (status!=0) {}

    Serial.println(F("MPU ON"));

    mpu.setAccOffsets(-0.01f, 0.07f, -2.06f);
    mpu.setGyroOffsets(-1.98f, 0.41f, -0.32f);

    // Motors setup
    pinMode(STEP_PIN1, OUTPUT);
    pinMode(STEP_PIN2, OUTPUT);
    pinMode(DIR_PIN1, OUTPUT);
    pinMode(DIR_PIN2, OUTPUT);

    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);

    // Timer setup
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, targetInterval, true);
    timerAlarmEnable(timer);
}

void loop(){
    static unsigned long lastUpdate = 0;
    unsigned long currentMillis = millis();
    unsigned long currentMicros = micros();

    mpu.update();




  sensorLoop(currentMicros);
  writeTeleplotLoop(currentMicros);
}