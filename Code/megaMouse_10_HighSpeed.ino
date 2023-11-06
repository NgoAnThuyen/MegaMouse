/*-------------------------------VL530XV2--------------------------*/
#include <Wire.h>
#include <VL53L0X.h>
const uint8_t sensorCount = 3;
const uint8_t xshutPins[sensorCount] = {4,2,15};
VL53L0X sensors[sensorCount];
/*-------------------------------PWM--------------------------*/
const int ledPin1 = 25; 
const int ledPin2 = 13;  
// configuration of PWM channels
const int freq = 5000;  // tần số xung
const int ledChannel1 = 0; // kênh PWM
const int ledChannel2 = 1; // kênh PWM
const int resolution = 8; // độ phân giải 8bit
/*-------------------------------MOTOR DIRECTION--------------------------*/
#define DIR1_1 27
#define DIR1_2 26
#define DIR2_1 12
#define DIR2_2 14
/*-------------------------------GLOBAL VARIABLES--------------------------*/
unsigned long previousMillis = 0;   
long thetaLeft, thetaRight, previousThetaLeft, previousThetaRight;
int sensorLeft, sensorMid, sensorRight ;
bool running = false;
int previousState;
bool busy;
bool missLeftWall= false;
int cnt;
float baseFwd, turn;
float desiredLeftDuty, leftDuty, previousLeftDuty;
float desiredRightDuty, rightDuty, previousRightDuty;
float sumErrorLeft, previousErrorLeft;
int previousSensorLeft;
int cntTurn180;
/*-------------------------------ROBOT PARAMETERS--------------------------*/

void setup()
{  
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

   // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    digitalWrite(xshutPins[i], HIGH);
    delay(100);
    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }
    sensors[i].setAddress(0x50 + i);
    // sensors[i].setMeasurementTimingBudget(20000);
    sensors[i].startContinuous();
  }
  delay(500);
  ledcSetup(ledChannel1, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);
  ledcAttachPin(ledPin1, ledChannel1);
  ledcAttachPin(ledPin2, ledChannel2);

  pinMode(DIR1_1, OUTPUT);
  pinMode(DIR1_2, OUTPUT);
  pinMode(DIR2_1, OUTPUT);
  pinMode(DIR2_2, OUTPUT);
  running = false;
  busy = true;
}
void loop() {
  unsigned long nowMillis = millis();  
  if (nowMillis - previousMillis >= 100) // 50 ms LOOP
  {
    sensorLeft = sensors[0].readRangeContinuousMillimeters();
    sensorMid = sensors[1].readRangeContinuousMillimeters();
    sensorRight = sensors[2].readRangeContinuousMillimeters() - 55;
    
    sensorLeft = constrain(sensorLeft,30, 250);
    sensorMid = constrain(sensorMid,30, 250);
    sensorRight = constrain(sensorRight,30, 250);

    if (sensorMid < 40) running = true;
  Serial.print("L:"); Serial.print(sensorLeft); Serial.print('\t');
  Serial.print("M: "); Serial.print(sensorMid); Serial.print('\t');
  Serial.print("R: "); Serial.print(sensorRight); Serial.print('\t');

  int state = getWallState(sensorLeft, sensorMid, sensorRight, 75);
  if ((sensorLeft - previousSensorLeft) >= 100) missLeftWall = true;
  Serial.print("state "); Serial.print(state); Serial.print('\t');
  Serial.print("missLeftWall "); Serial.print(missLeftWall); Serial.print('\t');
  
  if (running)
    {
      if (!missLeftWall)
      {
        if (state == 0)
        {
          turn = PID_LeftWall(sensorLeft,70.0,2.5,0.0,0.1,0.05);
          Serial.print(" PID LEFT ");
          baseFwd = 250;
        }
        else if (state == 1)
        {
          sumErrorLeft = 0;
          baseFwd = 0;
          Serial.print(" TURN RIGHT ");
          turn = 250;
        }
      }
      else 
      {
        sumErrorLeft = 0;
        cnt+=1;
        if (cnt <= 5) // tuning  number 10 for other condition // timming
        {
          turn = 0;
          baseFwd = 250;
          Serial.print(" Straight");
        }
        else if (cnt <= 8) // tuning  number 15 for other condition /timming
        {
          baseFwd = 0;
          turn = -100;          
          Serial.print(" Turn Left with holding position ");
        }
        else 
        {
          baseFwd = 150;
          turn = -100;
          Serial.print(" Turn Left");
        }

        if (sensorLeft <= 110)
        {
          cnt = 0;
          missLeftWall = false;
          baseFwd = 0;
          turn = 0;
        }
      }

        leftDuty = baseFwd + turn;
        rightDuty = baseFwd - turn;
        leftDuty = constrain(leftDuty,-250.0,250.0);
        rightDuty = constrain(rightDuty,-250.0,250.0);
        // Serial.print(running);
        Left_Motor_PWM((int)leftDuty);
        Right_Motor_PWM((int)rightDuty);

    }

    Serial.println();
    previousSensorLeft = sensorLeft;
    previousMillis = nowMillis;
  }

}
int getWallState(int sensorLeft, int sensorMid, int sensorRight, int threshold)
{
  int wallState;
  if (sensorMid > threshold)
    wallState = 0;
  else
    wallState = 1;
  return wallState;
}

float PID_LeftWall(int sensorLeft, float desiredSensorLeft,float Kp, float Ki, float Kd, float Ts)
{
  float errorLeft = desiredSensorLeft - (float)sensorLeft;
  sumErrorLeft += errorLeft;
  float turn_= Kp*errorLeft + Ki*sumErrorLeft*Ts + Kd* (errorLeft - previousErrorLeft)/Ts;
  previousErrorLeft = errorLeft;
  return turn_;
}

void Left_Motor_PWM(float PWM_)
{
  if (PWM_ >=0) 
  {
    ledcWrite(ledChannel1, PWM_);
    digitalWrite(DIR1_1, LOW);
    digitalWrite(DIR1_2, HIGH);
  }
  else
  {
    ledcWrite(ledChannel1, abs(PWM_));
    digitalWrite(DIR1_1, HIGH);
    digitalWrite(DIR1_2, LOW);
  }
}

void Right_Motor_PWM(float PWM_)
{
  if (PWM_ >=0) 
  {
    ledcWrite(ledChannel2, PWM_);
    digitalWrite(DIR2_1, HIGH);
    digitalWrite(DIR2_2, LOW);
  }
  else
  {
    ledcWrite(ledChannel2, abs(PWM_));
    digitalWrite(DIR2_1, LOW);
    digitalWrite(DIR2_2, HIGH);
  }
}

