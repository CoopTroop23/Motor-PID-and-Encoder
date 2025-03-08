#include <PID_v1.h>

/* HOW TO USE:
  1) create global motor variable
  2) initialize motor variable in setup()
  3) enable motor
  4) set the pid tunings
  5) use move() to move the motor (open loop)
  6) use PIDmove() to move the motor (closed loop)

*/
struct Drivemotor{
  public:
  static Drivemotor* instance;   // Static instance pointer for ISR
  //pins
  byte enPin; //enable
  byte dir1Pin; //direction
  byte dir2Pin;
  byte pwmPin; //speed
  byte Apin; //feedback
  byte Bpin;

  //variables 
  //stuff for encoder and interrupt
  volatile double encoderCount = 0; //input to pid
  bool BpinState = 0;
  
  
  double setpoint = 0; //setpoint for pid
  double output = 0;//figure out pid controll connections. perhaps pid init funciton
  int maxSpeed = 255;

  PID pid;

  Drivemotor() : pid(&encoderCount, &output, &setpoint, 1, 0, 0, DIRECT) {}

  //functions
  void initialize(int en,int d1,int d2,int pwm,int a,int b){
    enPin = en;
    dir1Pin = d1;
    dir2Pin = d2;
    pwmPin = pwm;
    Apin = a;
    Bpin = b;

    //define pinmode
    pinMode(enPin,OUTPUT);
    pinMode(dir1Pin,OUTPUT);
    pinMode(dir2Pin,OUTPUT);
    pinMode(pwmPin,OUTPUT);
    pinMode(Apin,INPUT_PULLUP); //maybe just regular inputs?
    pinMode(Bpin,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Apin),ISRfunction,RISING);
    pid.SetMode(AUTOMATIC);
  }

  void enable(){
    digitalWrite(enPin, HIGH);
  }

  void disable(){
    digitalWrite(enPin, LOW);
  }

  void setMaxSpeed(int s){
    maxSpeed = s;
  }

  void move(int mag, int dir){
    switch(dir){
      case -1:
        digitalWrite(dir1Pin,LOW);
        digitalWrite(dir2Pin,HIGH);
        break;
      case 0:
        digitalWrite(dir1Pin,LOW);
        digitalWrite(dir2Pin,LOW);
        break;
      case 1:
        digitalWrite(dir1Pin,HIGH);
        digitalWrite(dir2Pin,LOW);
        break;
    }

    if(mag > maxSpeed){
      mag = maxSpeed;
    }
    analogWrite(pwmPin,mag);
  }

  void PIDmove(double s){
    setpoint = s;
    pid.Compute();
    //constrain and separate output
    int mag = abs(output);
    int dir = output/mag; //forward or reverse
    move(mag, dir);
  }

  void test(){
    Serial.print(instance->BpinState);
    Serial.print(' ');
    Serial.println(BpinState);
    Serial.print(instance->pwmPin);
    Serial.print(' ');
    Serial.println(pwmPin);
    
  }

  private:

  static void ISRfunction(){
    //the interrupt is triggered by the A signal
    //direction is determined by the B signal  
      instance->BpinState = digitalRead(instance->Bpin);
      if(instance->BpinState == true){
        instance->encoderCount++;
      }else{
        instance->encoderCount--;
      }

      
    

  }

};


Drivemotor* Drivemotor::instance = nullptr; //idk chatGPT gave it to me :/


//void driveMotorPWM(Drivemotor m, int setpoint,);

Drivemotor leftDrive;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  leftDrive.initialize(52,10,11,9,20,21);
  leftDrive.maxSpeed = 100;
  leftDrive.enable();
  leftDrive.pid.SetTunings(1.0, 0, 0);
  leftDrive.test();
}

double pencoder = leftDrive.encoderCount;

void loop() {
  // put your main code here, to run repeatedly:
  leftDrive.move(50,0);
  // if(leftDrive.encoderCount != pencoder){
  //   Serial.println(leftDrive.encoderCount);
  //   pencoder = leftDrive.encoderCount;
  // }
  //Serial.println(leftDrive.encoderCount);
  //leftDrive.PIDmove(100);//100 encoder ticks
  //Serial.println(leftDrive.encoderCount);
  delay(10);
}
