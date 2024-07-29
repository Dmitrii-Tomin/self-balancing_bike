

#include <Wire.h>
#define RCPinFWD 2

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};
int Read = 0;
float distance = 0.0; 
float elapsedTime, currentTime, previousTime;
float distance_previous_error, distance_error;

float kp=9;
float ki=0.05; //with the weight weight balance and rpm limit you might not need it
float kd=0.4;
int weight_balance = 5;
float rpm_limit = 0.030;

float distance_setpoint = 0;
float PID_p, PID_i, PID_d, PID_total;
int speed;
int speed2;
volatile long counter = 0;
float multiplier;
float time;
float rpm;
float error;

volatile long StartTimeFWD = 0;
volatile long CurrentTimeFWD = 0;
volatile long PulsesFWD = 0;
int PulseWidthFWD = 0;





//Kalman filter:

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}




//initializing, reading and converting IMU readings:

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);   Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096-0.07;
  AccY=(float)AccYLSB/4096+0.02;
  AccZ=(float)AccZLSB/4096+0.25;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}




//setting the PWM frequency for pin 9 and 10 to 20khz:

void analogWriteSAH_Init( void )
{
  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);
  
  TCCR1A = 
      (1 << COM1A1) | (0 << COM1A0) |   // COM1A1, COM1A0 = 1, 0
      (1 << COM1B1) | (0 << COM1B0) |
      (1 << WGM11) | (0 << WGM10);      // WGM11, WGM10 = 1, 0

  ICR1 = 799;


  TCNT1 = 0;


  OCR1A = 0;
  OCR1B = 0;


  TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);

  DDRB |= (1 << DDB1);
  DDRB |= (1 << DDB2);


  TCCR1B =
      (0 << ICNC1) | (0 << ICES1) |
      (1 << WGM13) | (1 << WGM12) |
      (0 << CS12) | (0 << CS11) | (1 << CS10);
}

void analogWriteSAHA( uint16_t value )
{
  if ( (value >= 0) && (value < 800) )
  {
    OCR1A = value;
  }
}

void analogWriteSAHB( uint16_t value )
{
  if ( (value >= 0) && (value < 800) )
  {
    OCR1B = value;
  }
}




//measuring pwm frequency of reciever

void PulseTimerFWD(){
  CurrentTimeFWD = micros();
  if (CurrentTimeFWD > StartTimeFWD){
    PulsesFWD = CurrentTimeFWD - StartTimeFWD;
    StartTimeFWD = CurrentTimeFWD;
  }
}

//measuring reaction wheel rotation

void ai0() {
  if(digitalRead(3)==LOW) {
    counter--;
  }
  else{
    counter++;
  }
}

//--------------------------------------------------------------------------------------
  
void setup() {

  Serial.begin(57600);
  analogWriteSAH_Init();
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(RCPinFWD, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP); // internalเป็น pullup input pin 3
  attachInterrupt(1, ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(RCPinFWD),PulseTimerFWD,CHANGE);
  digitalWrite(13, LOW);
  
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  //measuring gyro drift
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();
}

//------------------------------------------------------------------------------------------

void loop() {

  //correcting drift
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RateYaw-=RateCalibrationYaw;

  //passing values through Kalman filter
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];

  //measuring loop time
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime)/1000; // Divide by 1000 to get seconds

  //measuring reaction speed "rpm"
  multiplier = 1 / elapsedTime;
  rpm = (counter - error) * multiplier;
  error = counter;

  //measuring error 
  distance = KalmanAngleRoll * -10;   
  distance_error = distance_setpoint - distance;   

  //correcting for shifted center of mass
  if (distance_error < distance_setpoint){
    distance_setpoint -= weight_balance * elapsedTime;
  }
  else{
    distance_setpoint += weight_balance * elapsedTime;
  }
  

  //preventing reaction wheel from reaching max speed(no change in speed = no reaction on bike = no balancing)
  if (PID_total < 0){
    if (rpm > 700){
      distance_setpoint -= rpm_limit;
    }
  }
  if (PID_total > 0){
    if (rpm > 700){
      distance_setpoint += rpm_limit;
    }
  }

  //measuring new error
  distance_error = distance_setpoint - distance;   


  PID_p = kp * distance_error;
  
  float dist_diference = distance_error - distance_previous_error; //measuring difference in angle    
  PID_d = kd*(dist_diference/elapsedTime);
      
  if(-150 < distance_error && distance_error < 150)
  {
    PID_i = PID_i + (ki * distance_error);
  }
  else
  {
    PID_i = 0;
  }
  
  PID_total = PID_p + PID_i + PID_d;  


  //breaking reaction wheel if bike falls
  if (KalmanAngleRoll > 30 || KalmanAngleRoll < -30){
    digitalWrite(13, LOW);
    digitalWrite(8, LOW);
    distance_setpoint = 0;
  }
  else{
    digitalWrite(13, HIGH);
    digitalWrite(8, HIGH);
  }


  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
  
  //changing spin direction of reaction wheel
  if (PID_total < 0){
    digitalWrite(12, LOW);
    speed =  map(PID_total, 0, -450, 750, 0);
  }
  if (PID_total > 0){
    digitalWrite(12, HIGH);
    speed =  map(PID_total, 0, 450, 750, 0);
  }

  
  //map the output of the reviever to the spped of the rear motor
  if (PulsesFWD < 2000){
    PulseWidthFWD = PulsesFWD;
  } 

  if (PulseWidthFWD < 1500){
    digitalWrite(7, HIGH);
    speed2 = map(PulseWidthFWD, 1500, 1000, 800, 650);
  }
  else{
    digitalWrite(7, LOW);
    speed2 = map(PulseWidthFWD, 2000, 1500, 500, 800);
  }

  //driving motors
  analogWriteSAHB(speed2);
  analogWriteSAHA(speed);

  //Serial.print(distance_setpoint);  Serial.print("\t");
  //Serial.println(KalmanAngleRoll);
  
  distance_previous_error = distance_error;
}
