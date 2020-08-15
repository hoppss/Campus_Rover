#include <ros.h>
#include <campusrover_msgs/ArmTablePosition.h>
#include <campusrover_msgs/ArmTableHomeReturn.h>
#include <campusrover_msgs/ElevatorStatusChecker.h>
ros::NodeHandle nh;

const int z_en = A8;  
const int z_ms1 = A9;
const int z_ms2 = A10;
const int z_ms3 = A11;    
const int z_reset = A12;
const int z_sleep = A13;
const int z_stp = A14; 
const int z_dir = A15;
const int y_en = A0;  
const int y_ms1 = A1;
const int y_ms2 = A2;
const int y_ms3 = A3;    
const int y_reset = A4;
const int y_sleep = A5;
const int y_stp = A6; 
const int y_dir = A7;
const int y_sensor =7;
const int z_sensor =6;
long y_data;
long y_now;
long y_x;
long y_st;
long z_data;
long z_now;
long z_x;
long z_st;
long moto_step;
long y_zero;
long z_zero;
bool go_signal;
bool zero_signal;


campusrover_msgs::ElevatorStatusChecker::Request finish_req;
campusrover_msgs::ElevatorStatusChecker::Response finish_res;

void go(const campusrover_msgs::ArmTablePosition::Request & req, campusrover_msgs::ArmTablePosition::Response & res)
{
  if(y_now!=8888 && z_now!=8888)
  {
    y_data=(int)((1000*req.position.y) + 0.5);
    z_data=(int)((1000*req.position.z) + 0.5);
    if (y_data>0 && y_data<500 && z_data>0 && z_data<700)
    {
      go_signal=true; 
    }
  } 
  else
  {
    nh.loginfo("Please Call Home Return Service");
  }
  nh.loginfo("y_data: ");
  nh.loginfo(y_data);
  nh.loginfo("z_data: ");
  nh.loginfo(z_data);
}

void zero(const campusrover_msgs::ArmTableHomeReturn::Request & req, campusrover_msgs::ArmTableHomeReturn::Response & res)
{
  y_zero=1;
  z_zero=1;
  zero_signal = true;
}

ros::ServiceServer <campusrover_msgs::ArmTablePosition::Request, 
                    campusrover_msgs::ArmTablePosition::Response> server1("table_position",&go);
ros::ServiceServer <campusrover_msgs::ArmTableHomeReturn::Request , 
                    campusrover_msgs::ArmTableHomeReturn::Response> server2("zero_return",&zero);
ros::ServiceClient <campusrover_msgs::ElevatorStatusChecker::Request ,
                    campusrover_msgs::ElevatorStatusChecker::Response> client("elevator_status_checker");


void moto(long y_motor_step,long z_motor_step, int y_axis_speed_level, int z_axis_speed_level)
{
  long y_step = 0; 
  long z_step = 0;
  int z_axis_home_return_speed_count = 0;
  int y_axis_home_return_speed_count = 0;
  bool z_HL_switch_flag = true;
  bool y_HL_switch_flag = true;
  int y_axis_speed_count_num = 2*y_axis_speed_level - 1;
  int z_axis_speed_count_num = 2*z_axis_speed_level - 1;;
  
  while(y_step < y_motor_step || z_step < z_motor_step)
  {
    z_axis_home_return_speed_count++;
    y_axis_home_return_speed_count++;
    
    if(z_axis_home_return_speed_count >= z_axis_speed_count_num && z_HL_switch_flag == true && z_step < z_motor_step)
    {
      digitalWrite(z_stp,HIGH);
      z_axis_home_return_speed_count = 0;
      z_HL_switch_flag = false;
    }

    if(y_axis_home_return_speed_count >= y_axis_speed_count_num && y_HL_switch_flag == true && y_step < y_motor_step)
    {
      digitalWrite(y_stp,HIGH);
      y_axis_home_return_speed_count = 0;
      y_HL_switch_flag = false;
    }
    delayMicroseconds(12);
    
    z_axis_home_return_speed_count++;
    y_axis_home_return_speed_count++;
    
    if(z_axis_home_return_speed_count >= z_axis_speed_count_num && z_HL_switch_flag == false && z_step < z_motor_step)
    {
      digitalWrite(z_stp,LOW);
      z_axis_home_return_speed_count = 0;
      z_HL_switch_flag = true;
      z_step++;
    }

    if(y_axis_home_return_speed_count >= y_axis_speed_count_num && y_HL_switch_flag == false && y_step < y_motor_step)
    {
      digitalWrite(y_stp,LOW);
      y_axis_home_return_speed_count = 0;
      y_HL_switch_flag = true;
      y_step++;
    }
    delayMicroseconds(12);
  }
// Serial.print(moto_step);
}

void home_return()
{
  int z_axis_home_return_speed_count = 0;
  int y_axis_home_return_speed_count = 0;
  bool z_HL_switch_flag = true;
  bool y_HL_switch_flag = true;
  
  while(y_zero!=0 || z_zero!=0)
  {
    z_axis_home_return_speed_count++;
    y_axis_home_return_speed_count++;
    
    if(z_axis_home_return_speed_count >= 1 && z_HL_switch_flag == true && z_zero!=0)
    {
      digitalWrite(z_stp,HIGH);
      z_axis_home_return_speed_count = 0;
      z_HL_switch_flag = false;
    }

    if(y_axis_home_return_speed_count >= 3 && y_HL_switch_flag == true && y_zero!=0)
    {
      digitalWrite(y_stp,HIGH);
      y_axis_home_return_speed_count = 0;
      y_HL_switch_flag = false;
    }
    delayMicroseconds(40);
    
    z_axis_home_return_speed_count++;
    y_axis_home_return_speed_count++;
    
    if(z_axis_home_return_speed_count >= 1 && z_HL_switch_flag == false && z_zero!=0)
    {
      digitalWrite(z_stp,LOW);
      z_axis_home_return_speed_count = 0;
      z_HL_switch_flag = true;
    }

    if(y_axis_home_return_speed_count >= 3 && y_HL_switch_flag == false && y_zero!=0)
    {
      digitalWrite(y_stp,LOW);
      y_axis_home_return_speed_count = 0;
      y_HL_switch_flag = true;
    }
    delayMicroseconds(40);

    if (digitalRead(y_sensor) == HIGH){y_zero=0;}
    if (digitalRead(z_sensor) == HIGH){z_zero=0;}
  }
}

void y_motor_direction(int direct)
{
  if(direct<0)
  {digitalWrite(y_dir,LOW);}
  else
  {digitalWrite(y_dir,HIGH);}
}

void z_motor_direction(int direct)
{
  if(direct>0)
  {digitalWrite(z_dir,LOW);}
  else
  {digitalWrite(z_dir,HIGH);}
}


void setup() {
   nh.initNode();
   nh.advertiseService(server1);
   nh.advertiseService(server2);
   nh.serviceClient(client);
   pinMode(y_sensor,OUTPUT);
   pinMode(z_sensor,OUTPUT);
   pinMode(z_en,OUTPUT);
   pinMode(y_en,OUTPUT);
   digitalWrite(z_en,HIGH);
   digitalWrite(y_en,HIGH);
   pinMode(z_ms1,OUTPUT);
   pinMode(y_ms1,OUTPUT);
   pinMode(z_ms2,OUTPUT);
   pinMode(y_ms2,OUTPUT);
   pinMode(z_ms3,OUTPUT);
   pinMode(y_ms3,OUTPUT);
   pinMode(z_reset,OUTPUT);
   pinMode(y_reset,OUTPUT);
   pinMode(z_sleep,OUTPUT);
   pinMode(y_sleep,OUTPUT);
   pinMode(z_stp,OUTPUT);
   pinMode(y_stp,OUTPUT);
   pinMode(z_dir,OUTPUT);
   pinMode(y_dir,OUTPUT);
   digitalWrite(z_ms1,HIGH);
   digitalWrite(y_ms1,HIGH);
   digitalWrite(z_ms2,HIGH);
   digitalWrite(y_ms2,HIGH);
   digitalWrite(z_ms3,HIGH);
   digitalWrite(y_ms3,HIGH);
   digitalWrite(z_reset,HIGH);
   digitalWrite(y_reset,HIGH);
   digitalWrite(z_sleep,HIGH);
   digitalWrite(y_sleep,HIGH); 
   delay(1000);
   digitalWrite(z_en,LOW);
   digitalWrite(y_en,LOW);
   Serial.begin(57600);
  // Serial.println("connected");
   nh.loginfo("connected");
   y_now=8888;
   z_now=8888;
}

void loop() 
{
  if(go_signal==true)
  {
  //   Serial.println("yz_data=");
  //   Serial.println(y_data);
  //   Serial.println(z_data); 
   y_x=y_data-y_now;
   z_x=z_data-z_now;
   y_motor_direction(y_x);
   z_motor_direction(z_x);
   y_st=3200*abs(y_x)/37.73052777;
   z_st=3200*abs(z_x)/8.0;
  if(abs(z_x) < 100)
  {
    moto(y_st,z_st,4,5); //1:fastest 2:faster 3:normal 4:lower 5:lower ....
  }
  else
  {
    long z_st_fastest = z_st - (3200*abs(80)/8.0);
    long z_st_acc = (3200*abs(40)/8.0);
    moto(y_st,z_st,2,2); 
//    moto(0,z_st_fastest,4,3); 
//    moto(0,z_st_acc,4,4); 
  }
   
  
   y_now=y_data;
   z_now=z_data;
   y_data=0;
   z_data=0;
  //   Serial.println("yz_now=");
  //   Serial.println(y_now);
  //   Serial.println(z_now);
  //   Serial.println("done");
   nh.loginfo("done");
   finish_req.node_name.data="ArmTablePosition";
   finish_req.status.data=true;
  //   client.call(finish_req,finish_res);
   go_signal=false;
  }
  
  if (zero_signal==true)
  {
    
    y_motor_direction(-1);
    z_motor_direction(-1);
    home_return();

    delay(200);
    y_motor_direction(1);
    z_motor_direction(1);
    delay(200);
    moto(400,1600,4,5);
    y_now=0;
    z_now=0;
    finish_req.node_name.data="ArmTableHomeReturn";
    finish_req.status.data=true;
//    client.call(finish_req,finish_res);/
    nh.loginfo("Home Return done!!");
    zero_signal=false;
  }
  nh.spinOnce();
  
}
