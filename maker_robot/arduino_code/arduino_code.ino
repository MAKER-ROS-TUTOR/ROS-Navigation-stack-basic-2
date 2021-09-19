#include <PID_v1.h>
#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <std_msgs/UInt16.h>

#define PULSES_PER_TURN (1856.0)   //  Encoder Resolution:  CPR *4  for convert to PPC
#define FOR 1  //  robot Forward test

#define encodPinA1      3                       // Quadrature encoder A pin
#define encodPinB1      8                       // Quadrature encoder B pin
#define M1              5                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              6

ros::NodeHandle  nh;
rospy_tutorials::Floats right_joint_state, left_joint_state;

String inString = "";    // string to hold input type 0 - 255 for test motor with arduino serial
int motorPWM = 0;

boolean Direction;//the rotation direction 

long previousMillis = 0;
long currentMillis = 0;
//-----------------------------------------------------------------------------

double  kp =1, ki =20 , kd =0;             // modify for optimal performance
double  input = 0, output = 0, setpoint = 0;
unsigned long lastTime,now;
volatile long encoderPos = 0,last_pos=0,lastpos=0;

double pos = 0;

PID myPID(&input, &output, &setpoint, kp, ki, kd,DIRECT);  

ros::Publisher rightPub("right_ticks", &right_joint_state);
ros::Publisher leftPub("left_ticks", &left_joint_state);


void pwm_cb( const std_msgs::UInt16& cmd_msg){
  motorPWM = cmd_msg.data; //set servo angle, should be from 0-180  
  
}

ros::Subscriber<std_msgs::UInt16> sub("pwm_to_arduino", pwm_cb);
void setup()
{
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  
  right_joint_state.data = (float *)malloc(sizeof(float)*2);
  right_joint_state.data_length = 2;

  left_joint_state.data = (float *)malloc(sizeof(float)*2);
  left_joint_state.data_length = 2;
  
  
  // ROS Setup
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);

  nh.subscribe(sub);
  

  TCCR1B = TCCR1B & 0b11111000 | 1;                   // To prevent Motor Noise
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
 
  EncoderInit();//Initialize the module
  
}
 
void loop()
{
    
  //  _TEST_MOTOR_PWM_SERIAL();
     run_motor();

     now = millis();
     currentMillis = millis();

     if((currentMillis - lastTime) > 500 )
   {
      input = (360.0*1000*(encoderPos-last_pos)) /( PULSES_PER_TURN *(now - lastTime));
      lastTime=now;
      last_pos=encoderPos;
   }

    if( (currentMillis - lastTime) > 100 ){
   
      pos = (360.0*(encoderPos-lastpos))/PULSES_PER_TURN;
   
      lastpos=encoderPos;
      lastTime=now;
      
      right_joint_state.data[0]=pos;
      right_joint_state.data[1]=0;

     
      left_joint_state.data[0]=pos;
      left_joint_state.data[1]=0;

      leftPub.publish(&left_joint_state);
      rightPub.publish(&right_joint_state);

    
       nh.spinOnce();
      
    }

    
    //  encoderPos--;
    // if( encoderPos > (2096 * 4) ) encoderPos=0;
     
  
     myPID.Compute();                                    // calculate new output
     // pwmOut(output);                                     // drive L298N H-Bridge module
     delay(10);

}

int readSerialValue()
{
     int v= -1;
  // Read serial input:
  
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string,
    // then the string's value:
    if (inChar == '\n') {
      Serial.print("String: ");
      Serial.println(inString);
      v = inString.toInt();
      Serial.print("Value:");
      Serial.println(v);
      
           
      // clear the string for new input:
      inString = "";
    }
  }
   return v;
}

void EncoderInit()
{
  
  pinMode(encodPinA1,INPUT);  
  pinMode(encodPinB1,INPUT_PULLUP);
  
  attachInterrupt( digitalPinToInterrupt(encodPinA1), wheelSpeed, RISING );
 //  attachInterrupt(digitalPinToInterrupt(encoder0pinB), wheelSpeed, CHANGE);
}
void wheelSpeed()
{
  
  encoderPos++;

 // if( encoderPos > (2096 * 4) ) encoderPos=0;
 // Serial.println("speed");
  
}
void _TEST_MOTOR_PWM_SERIAL()
{
  int pwm = readSerialValue();

    if( pwm != -1){

       if( pwm > 255) pwm = 255;
       else if( pwm < -1) pwm = 0;

       motorPWM = pwm;
    }

    if( FOR == 1){
      analogWrite(M1, motorPWM);
      analogWrite(M2, 0);
    }else{
      analogWrite(M1, 0);
      analogWrite(M2, motorPWM);
    }
}
void run_motor(){
  
    if( FOR == 1){
      analogWrite(M1, motorPWM);
      analogWrite(M2, 0);
    }else{
      analogWrite(M1, 0);
      analogWrite(M2, motorPWM);
    }
    
}
