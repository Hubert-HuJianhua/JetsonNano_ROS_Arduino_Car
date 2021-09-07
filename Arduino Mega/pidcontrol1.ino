#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#define BUTTON 32
#define LED 33

ros::NodeHandle node_handle;

std_msgs::String button_msg;
std_msgs::UInt16 led_msg;
std_msgs::Int32MultiArray ArcadeDrive_num;

class SimplePID{
  private:
    float kp, kd, ki, umax;
    float eprev, eintegral;
    
  public:
    // Default initialization list
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
    
    // Set the parameters
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // Evaluate the signal
    void evalu(int value, int target, float deltaT,int &pwr, int &dir){

      if(target!=0){  
      // error
      int e = target - value;
      
      float dedt = (e-eprev)/(deltaT);
      eintegral = eintegral + e*deltaT;
      float u = kp*e + kd*dedt + ki*eintegral;
    
      // motor power
      pwr = (int) fabs(u);
      if( pwr > umax ){
        pwr = umax;
      }
           
      // motor direction
      dir = 1;
      if(u<0){
        dir = -1;
      }
            
      // store previous error
      eprev = e;
      }
      else{
        pwr = 0;
        }
    }
    
};



#define NMOTORS 4
const int motor_PWM[] = {6,7,8,9};
const int motor_H[] = {31,28,27,24};
const int motor_L[] = {30,29,26,25};

Encoder EncFrontLeft(2, 4);
Encoder EncFrontRight(18, 16);
Encoder EncRearLeft(3, 5);
Encoder EncRearRight(19, 17);

SimplePID pid[NMOTORS];

long prevT = 0;
long old_time = 0;

volatile long pos[] = {0,0,0,0};
volatile long target[] = {0,0,0,0};

/*------------ FUNCTIONS ------------*/
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void ArcadeDrivesubscriberCallback(const std_msgs::Int32MultiArray& ArcadeDrive_num){
        target[0]=ArcadeDrive_num.data[0];
        target[1]=ArcadeDrive_num.data[1];
        target[2]=ArcadeDrive_num.data[2];
        target[3]=ArcadeDrive_num.data[3];
  }
  void subscriberCallback(const std_msgs::UInt16& led_msg) {
  if (led_msg.data  == 1) {
    digitalWrite(LED, HIGH); 
  } else {
    digitalWrite(LED, LOW);
  }
}
ros::Subscriber<std_msgs::Int32MultiArray> ArcadeDrive_subscriber("ArcadeDrive", &ArcadeDrivesubscriberCallback);
ros::Publisher button_publisher("button_press", &button_msg);
ros::Subscriber<std_msgs::UInt16> led_subscriber("toggle_led", &subscriberCallback);

void setup() {
  node_handle.initNode();
  node_handle.advertise(button_publisher);
  node_handle.subscribe(led_subscriber);
  node_handle.subscribe(ArcadeDrive_subscriber);
  for(int k = 0; k < NMOTORS; k++){
    pid[k].setParams(13.1,12.1,1.1,255);
  }
}


void loop(){

  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT; // Store the previous time value
  if(currT - old_time >= 10000) // 如果计时时间已达1秒
  {  
     pos[0] = -(float)EncFrontLeft.read()*4.16;
     pos[1] = -(float)EncFrontRight.read()*3.84; //3.84 = 6000/(13*2*2*30)
     pos[2] = (float)EncRearLeft.read()*3.84;
     pos[3] = -(float)EncRearRight.read()*3.84;
    
//    pos[0]=VFrontLeft;
//    pos[1]=VFrontRight;
//    pos[2]=VRearLeft;
//    pos[3]=VRearRight;

    EncFrontLeft.write(0);
    EncFrontRight.write(0);
    EncRearLeft.write(0);
    EncRearRight.write(0);
    old_time=  micros();     // 记录每秒测速时的时间节点   
  }

  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir); // compute the position
    setMotor(dir,pwr,motor_PWM[k],motor_H[k],motor_L[k]); // signal the motor
  }
  node_handle.spinOnce();
  
}
