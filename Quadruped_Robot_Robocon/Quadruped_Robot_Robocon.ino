/* -----------------------------------------------------------------------------
  - Project: Remote control Crawling robot
  - Author:  panerqiang@sunfounder.com
  - Date:  2015/1/27
   -----------------------------------------------------------------------------
  - Overview
  - This project was written for the Crawling robot desigened by Sunfounder.
    This version of the robot has 4 legs, and each leg is driven by 3 servos.
  This robot is driven by a Ardunio Nano Board with an expansion Board.
  We recommend that you view the product documentation before using.
  - Request
  - This project requires some library files, which you can find in the head of
    this file. Make sure you have installed these files.
  - How to
  - Before use,you must to adjust the robot,in order to make it more accurate.
    - Adjustment operation
    1.uncomment ADJUST, make and run
    2.comment ADJUST, uncomment VERIFY
    3.measure real sites and set to real_site[4][3], make and run
    4.comment VERIFY, make and run
  The document describes in detail how to operate.
   ---------------------------------------------------------------------------*/

// modified by Regis for spider project
// modified by John Crombie
// Added additional moves and LiPo battery monitor on balance connector

/* Includes ------------------------------------------------------------------*/
#include <FlexiTimer2.h>//to set a timer to manage all servos


#include <SoftwareSerial.h>
#include <CytronServoShield.h>

SoftwareSerial swSerial1(2, 3);
SoftwareSerial swSerial2(10, 11);

/* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs
CytronServoShield servo1;
CytronServoShield servo2;
//define servos' ports
const int servo_pin[4][3] = { {2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13} };
const float servo_error[4][3] = { {-10,-10,0}, {0,0,0}, {-10,-10,0}, {0,0,0} }; // JC needed to get servo arm at 90 degrees to the body when set to 90 degrees by program. used in polar_to_servo
const float servo_test[4][3] = { {93.11,86.71,132.83},{129.99,125.99,47.17},{119.99,120.99,57.17},{16.83,30.43,122.83}}; // test angles


const int angle_treshold = 30;
/* Size of the robot (mm) ---------------------------------------------------------*/
const float length_a = 80;
const float length_b = 245;
const float length_c = 45;
const float length_side = 210;
const float z_absolute = -180; //Initial z position when robot is started; Position is relative to base/servo closest to base
/* Constants for movement ----------------------------------------------------*/
const float z_default = -180; //Default z position when standing and walking
const float z_up = -130; //How high should the leg be raised during the walking
const float z_boot = z_absolute, z_bow_relative = 25, z_tall = -260; //used for tall and bow function, unnecessary
const float x_default = 130, x_offset = 0; //Default x position during walking and standing
const float y_start = 0, y_step = 80; //starting y position during walking and how big should the step be
const float sprawl_leg = 95; // used for both X and Y on sprawl movement
const int WALK = 0;
const int RELAX = 1;
const int SPRAWL = 2;
const bool FALSE = 0;
const bool TRUE = 1;
/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];    //real-time coordinates of the end of each leg
volatile float angle_expect[4][3];
volatile float site_expect[4][3]; //expected coordinates of the end of each leg
volatile bool capture = true;
float temp_speed[4][3];   //each axis' speed, needs to be recalculated before each movement
float move_speed;     //movement speed
float speed_multiple = 1; //movement speed multiple
int leg_position; // one of WALK, RELAX, SPRAWL
//speed , slowest - fastest (1 - 100), 0 = instant
const float spot_turn_speed = 40;
const float leg_move_speed = 100;
const float body_move_speed = 100;
const float stand_seat_speed = 40;
const float gyrate_speed =0.8;
const float bow_speed = 1;
volatile int rest_counter;      //+1/0.02s, for automatic rest
//functions' parameter
const float KEEP = 255;
//define PI for calculation
const float pi = 3.1415926;
volatile float site_cg[4][3]; // record current position to determine how to move cg
/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;
/* JC Constants for twists from relaxed pose ---------------------------------*/
const float robot_side_length = 70.84; // pivot to pivot
const float robot_front_length = 77.92; // pivot to pivot
const float robot_centre_to_pivot = sqrt(pow(robot_side_length/2,2)+pow(robot_front_length/2,2)); 
const float move_angle = 15; // amount to move each direction
const float temp_hyp = sqrt(pow(x_default - x_offset,2) + pow(y_start + y_step,2)) + robot_centre_to_pivot;
const float angle_1 = atan((y_start + y_step,2)/(x_default - x_offset,2));
const float move_x1 = temp_hyp*cos(angle_1-move_angle);
const float move_y1 = temp_hyp*sin(angle_1-move_angle);
const float move_x2 = temp_hyp*cos(angle_1+move_angle);
const float move_y2 = temp_hyp*sin(angle_1+move_angle);
const float half_step = 10; // forward and backward
const float cg_shift = 14; // distance to move CG to retain balance with leg raised
/* Other----------------------------------------------------------------------*/
bool monitor_voltage = FALSE;
const float minimum_voltage = 3.6; // minimum voltage for lipo battery
const int consecutive_under_voltage_readings = 3;
volatile int count_on_under_voltage;
const float three_cell_ratio = (5.6 + 2.8)/2.8; // values of the resistors in (kilo ohms) for the 3c potential divider
const float two_cell_ratio = (4.7 + 4.7)/4.7; // values of the resistors in (kilo ohms) for the 2c potential divider
const float vcc = 5.26; // measured output from UBEC feeding Arduino. Needed for accurate voltage measurement
/* ---------------------------------------------------------------------------*/

/*
  - setup function
   ---------------------------------------------------------------------------*/
void setup()
{
  //start serial for debug
  Serial.begin(9600); // baud rate chosen for bluetooth compatability

  swSerial1.begin(9600);
  servo1.init(&swSerial1);
  //  For boards in which compiler is not avr-gcc, and SoftwareSerial is used and listen() function is available
  //  use the function below instead to init servo shield
  //  servo1.init(&swSerial1, []{ swSerial1.listen() });
  swSerial2.begin(9600);
  servo2.init(&swSerial2);

  servo1.setChannel(ALL_CHANNELS, ON);
  servo2.setChannel(ALL_CHANNELS, ON);

  //for(int i=0;i<4;i++){ polar_to_servo(i, 30, 60, 45);}
  //delay(2000);
  
  Serial.println("Robot starts initialization");
  //initialize default parameter
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  //wait_all_reach();
  leg_position=WALK;
  
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }

// JC check if we are to monitor voltage
  count_on_under_voltage = 0;
  if (analogRead(A1) >20) // is the voltage checker being used?
  {
    if (analogRead(A1) > 1020) // is the connector reversed?
    {
      Serial.println("Check balance connector orentation");
      while(1) // do not continue
      {
        delay(10000);
      }
    }
    // Start voltage checker
    Serial.println("Voltage being monitored");
    monitor_voltage = FALSE;
  }
  else
  {
    Serial.println("No voltage detected on balance plug");
  }
  
  //start servo service
  FlexiTimer2::set(20, servo_service); // service also checks battery every 500 calls i.e. every 10 seconds
  FlexiTimer2::start();
  Serial.println("Servo service started");
  //initialize servos
  //servo_attach();
  Serial.println("Servos initialized");
  Serial.println("Robot initialization Complete");

  //for(int i=0;i<4;i++){ polar_to_servo(i, 30, 60, 45);}
  //delay(4000);
}


/*
  - loop function
   ---------------------------------------------------------------------------*/
void loop()
{
 //leg position test 
/*  relax_legs();
  set_site(0, KEEP, KEEP, z_default - z_bow_relative);
  set_site(1, KEEP, KEEP, z_default - z_bow_relative);
  set_site(2, KEEP, KEEP, z_default - z_bow_relative);
  set_site(3, KEEP, KEEP, z_default - z_bow_relative);
*/  

  // Serial.println("Stand");
  //stand();
  //delay(2000);
  Serial.println("Forward");
  step_forward(4);

  
}


  /*
  - JC
  - Change_legs to for a X for a relaxed pose
  - blocking function
   ---------------------------------------------------------------------------*/
void relax_legs(void)
{
  move_speed = leg_move_speed;
  switch (leg_position)
  {  
    case WALK:
      if (site_now[3][1] == y_start)
      {
        move_cg(2);
        set_site(2, KEEP, KEEP, z_up);
        wait_all_reach();
        set_site(2, x_default - x_offset, y_start + y_step, z_up);
        wait_all_reach();
        return_cg(2);
        set_site(2, x_default - x_offset, y_start + y_step, z_default);
        wait_all_reach();
        move_cg(3);
        set_site(3, KEEP, KEEP , z_up);
        wait_all_reach();
        set_site(3, x_default - x_offset, y_start + y_step, z_up);
        wait_all_reach();
        return_cg(3);
        set_site(3, x_default - x_offset, y_start + y_step, z_default);
        wait_all_reach();
      }
      else
      {
        move_cg(0);
        set_site(0, KEEP, KEEP, z_up);
        wait_all_reach();
        set_site(0, x_default - x_offset, y_start + y_step, z_up);
        wait_all_reach();
        return_cg(0);
        set_site(0, x_default - x_offset, y_start + y_step, z_default);
        wait_all_reach();
        move_cg(1);
        set_site(1, KEEP, KEEP , z_up);
        wait_all_reach();
        set_site(1, x_default - x_offset, y_start + y_step, z_up);
        wait_all_reach();
        return_cg(1);
        set_site(1, x_default - x_offset, y_start + y_step, z_default);
        wait_all_reach();
      }
      break;
    
    case SPRAWL:
      set_site(1,KEEP, KEEP, z_up);
      wait_all_reach();
      set_site(1, x_default - x_offset, y_start + y_step, KEEP);
      wait_all_reach();
      
      set_site(3,KEEP, KEEP, z_up);
      wait_all_reach();
      set_site(3, x_default - x_offset, y_start + y_step, KEEP);
      wait_all_reach();
      
      set_site(0,KEEP, KEEP, z_up);
      wait_all_reach();
      set_site(0, x_default - x_offset, y_start + y_step, KEEP);
      wait_all_reach();
      
      set_site(2,KEEP, KEEP, z_up);
      wait_all_reach();
      set_site(2, x_default - x_offset, y_start + y_step, KEEP);
      wait_all_reach   ();
      
      for(int i=0; i<4; i++)
      { 
        set_site(i, x_default - x_offset, y_start + y_step, z_default);
      }
      wait_all_reach();
      break;
  }
  leg_position = RELAX;
}
/*
  - JC
  - Change_legs to for a start of walk pose V on one side = on other
  - blocking function
   ---------------------------------------------------------------------------*/
void walk_legs(void)
{
  move_speed = leg_move_speed;

  switch (leg_position)
  {  
    case RELAX:
        move_cg(2);
        set_site(2, KEEP, KEEP, z_up);
        wait_all_reach();
        set_site(2, x_default - x_offset, y_start, z_up);
        wait_all_reach();
        return_cg(2);
        set_site(2, x_default - x_offset, y_start, z_default);
        wait_all_reach();
        move_cg(3);
        set_site(3, KEEP, KEEP, z_up);
        wait_all_reach();
        set_site(3, x_default - x_offset, y_start, z_up);
        wait_all_reach();
        return_cg(3);
        set_site(3, x_default - x_offset, y_start, z_default);
        wait_all_reach();
      break;
    
    case SPRAWL:
      relax_legs();
      walk_legs();
      break;
  }
  leg_position = WALK;
}
/*
  - JC
  - Change_legs to for a start of walk pose V on one side = on other mirror image of walk_legs
  - blocking function
   ---------------------------------------------------------------------------*/
void walk_legs_other(void)
{
  move_speed = leg_move_speed;

  switch (leg_position)
  {  
    case RELAX:
        move_cg(0);
        set_site(0, KEEP, KEEP, z_up);
        wait_all_reach();
        set_site(0, x_default - x_offset, y_start, z_up);
        wait_all_reach();
        return_cg(0);
        set_site(0, x_default - x_offset, y_start, z_default);
        wait_all_reach();
        move_cg(1);
        set_site(1, KEEP, KEEP, z_up);
        wait_all_reach();
        set_site(1, x_default - x_offset, y_start, z_up);
        wait_all_reach();
        return_cg(1);
        set_site(1, x_default - x_offset, y_start, z_default);
        wait_all_reach();
      break;
    
    case SPRAWL:
      relax_legs();
      walk_legs_other();
      break;
  }
  leg_position = WALK;
}

/*
  - JC
  - Change_legs to for a X for a sprawl pose
  - blocking function
   ---------------------------------------------------------------------------*/
void sprawl(void)
{
  move_speed = leg_move_speed;

  relax_legs();
  for (int i=0; i<4; i++)
  {
    set_site(i,sprawl_leg,sprawl_leg,KEEP);
  }
  wait_all_reach();
  leg_position = SPRAWL;
}
/*
  - JC
  - Change_leg_z
  - leg to change
  - blocking function
   ---------------------------------------------------------------------------*/
void change_leg_z(int leg,float z)
{
  set_site(leg,KEEP,KEEP,z);
  wait_all_reach();
}
/*
  - JC
  - move_cg
  - leg to change
  - blocking function
  - move the body of the robot away from the leg that is going to be lifted
   ---------------------------------------------------------------------------*/
void move_cg(int leg)
{
  // reducing right leg and increasing left leg X moves right
  // reducing front leg and increasing back leg Y moves forward
  float temp_move_speed = move_speed;
  move_speed = body_move_speed;
  
  //site_now contains current leg position - remember this
  for (int i = 0; i <4; i++)
  {
    for (int j=0; j < 3; j++)
    {
      site_cg[i][j] = site_now[i][j];
    }
    if (capture)
    {
      Serial.println("CG, "+String(i)+", "+String(site_cg[i][0])+", "+String(site_cg[i][1])+", "+String(site_cg[i][2]));
    }
  }
 
  switch(leg)
  {
    case 0:  // front right leg - move body weight back and left
    set_site(0, site_cg[0][0] + cg_shift, site_cg[0][1] + cg_shift, site_cg[0][2]);
    set_site(1, site_cg[1][0] + cg_shift, site_cg[1][1] - cg_shift, site_cg[1][2]);
    set_site(2, site_cg[2][0] - cg_shift, site_cg[2][1] + cg_shift, site_cg[2][2]);
    set_site(3, site_cg[3][0] - cg_shift, site_cg[3][1] - cg_shift, site_cg[3][2]);
    break;
      
    case 1:  // back right leg - move body weight forward and left
    set_site(0, site_cg[0][0] + cg_shift, site_cg[0][1] - cg_shift, site_cg[0][2]);
    set_site(1, site_cg[1][0] + cg_shift, site_cg[1][1] + cg_shift, site_cg[1][2]);
    set_site(2, site_cg[2][0] - cg_shift, site_cg[2][1] - cg_shift, site_cg[2][2]);
    set_site(3, site_cg[3][0] - cg_shift, site_cg[3][1] + cg_shift, site_cg[3][2]);
    break;
      
    case 2:  // front left leg - move body weight back and right
    set_site(0, site_cg[0][0] - cg_shift, site_cg[0][1] + cg_shift, site_cg[0][2]);
    set_site(1, site_cg[1][0] - cg_shift, site_cg[1][1] - cg_shift, site_cg[1][2]);
    set_site(2, site_cg[2][0] + cg_shift, site_cg[2][1] + cg_shift, site_cg[2][2]);
    set_site(3, site_cg[3][0] + cg_shift, site_cg[3][1] - cg_shift, site_cg[3][2]);
    break;
      
    case 3:  // back left leg - move body weight forward and right
    set_site(0, site_cg[0][0] - cg_shift, site_cg[0][1] - cg_shift, site_cg[0][2]);
    set_site(1, site_cg[1][0] - cg_shift, site_cg[1][1] + cg_shift, site_cg[1][2]);
    set_site(2, site_cg[2][0] + cg_shift, site_cg[2][1] - cg_shift, site_cg[2][2]);
    set_site(3, site_cg[3][0] + cg_shift, site_cg[3][1] + cg_shift, site_cg[3][2]);
    break;
  } 
  wait_all_reach();
  move_speed = temp_move_speed;
  
}
/*
  - JC
  - return_cg
  - leg to change
  - blocking function
  - move the body of the robot back towards lifted leg
   ---------------------------------------------------------------------------*/
void return_cg(int leg)
{
  float temp_move_speed = move_speed;
  move_speed = body_move_speed;
  for (int i=0; i<4; i++)
  {
    if (i != leg)
    {
      set_site(i, site_cg[i][0], site_cg[i][1], site_cg[i][2]);
    }
  }

  move_speed = temp_move_speed;
  
}
// -- end of JC added move routines --

/*
  - sit
  - blocking function
   ---------------------------------------------------------------------------*/
void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}


/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_left(unsigned int step)
{
  walk_legs(); // JC - get legs in correct starting position
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_right(unsigned int step)
{
  walk_legs(); // JC - get legs in correct starting position
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_forward(unsigned int step)
{
  walk_legs(); // JC - get legs in correct starting position
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_back(unsigned int step)
{
  walk_legs(); // JC - get legs in correct starting position
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

// add by RegisHsu

void body_left(int i)
{
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void body_right(int i)
{
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}



/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/


/*
  - set one of end points' expect site
  - this founction will set temp_speed[4][3] at same time
  - non - blocking function
   ---------------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;
  if (capture)
  {
    Serial.println("P, "+String(leg)+", "+String(x)+", "+String(y)+", "+String(z)); // log the set leg position
  }
  
  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

//  Serial.println("A\t "+String(i)+"\t"+String(temp_speed[leg][0])+"\t"+String(temp_speed[leg][1])+"\t"+String(temp_speed[leg][2]));

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_reach(int leg)
{
  delay(70);
  return;//for faster movement
  while (1)
    if (abs(angle_expect[leg][0] - servo_get_angle(leg, 0)) < angle_treshold)
      if (abs(angle_expect[leg][1] - servo_get_angle(leg, 1)) < angle_treshold)
        if (abs(angle_expect[leg][2] - servo_get_angle(leg, 2)) < angle_treshold)
          break;
}

/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}

/*
  - trans site from cartesian to polar
  - mathematical model 2/2
   ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //calculate w-z degree
  //Serial.println("C\t \t"+String(x)+"\t"+String(y)+"\t"+String(z));
  float v, w, temp_alpha, temp_beta;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;

  temp_alpha = (pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2));
  temp_alpha = constrain(temp_alpha, -1, 1);
  alpha = atan2(z, v) + acos(temp_alpha);
  temp_beta = (pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b;
  temp_beta = constrain(temp_beta, -1, 1);
  beta = acos(temp_beta);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;

  if(false) {
    Serial.println("\nx: "+String(x)+" y: "+String(y)+" z: "+String(z));
    Serial.println("w: "+String(w)+" v: "+String(v));
    Serial.println("alpha = "+String(atan2(z,v))+" + acos(("+String(length_a)+"^2 - "+String(length_b)+"^2 + "+String(z)+"^2 + "+String(v)+"^2) / 2 / "+String(length_a)+" / sqrt("+String(v)+"^2 +" +String(z)+"^2) *180/pi= "+String(alpha));
    Serial.println("beta = acos(("+String(length_a)+"^2 + "+String(length_b)+"^2 - "+String(v)+"^2 - "+String(z)+"^2)/ 2 / "+String(length_a)+"/ "+String(length_b)+" * 180/pi= "+ String(beta));
    Serial.println("gamma = atan2("+String(y)+", "+String(x)+") = "+String(gamma));
  }
}

servo_channel_t servo_to_channel(int servo, int option){
  servo_channel_t channel;
  switch(option) {
    case 0:
      switch(servo){
        case 0:
          channel = CHANNEL_2;
          break;
        case 1:
          channel = CHANNEL_1;
          break;
        case 2:
          channel = CHANNEL_3;
          break;
      }
      break;
    case 1:
      switch(servo){
        case 0:
          channel = CHANNEL_7;
          break;
        case 1:
          channel = CHANNEL_8;
          break;
        case 2:
          channel = CHANNEL_6;
          break;
      }
      break;
  }
  return channel;
}

void servo_write(int leg, int servo, int angle){
  //s1 Channel 1 is alpha
  servo_write(leg, servo, angle, 100);
}

void servo_write(int leg, int servo, int angle, int speed){
  //s1 Channel 1 is alpha
  
  switch(leg){
    case 0:
      servo1.angle(servo_to_channel(servo, 0), angle, speed);
      break;
    case 1:
      servo2.angle(servo_to_channel(servo, 0), angle, speed);
      break;
    case 2:
      servo1.angle(servo_to_channel(servo, 1), angle, speed);
      break;
    case 3:
      servo2.angle(servo_to_channel(servo, 1), angle, speed);
      break;
  }
}

int servo_get_angle(int leg, int servo){
  //s1 Channel 1 is alpha
  int angle = 0;
  switch(leg){
    case 0:
      angle = servo1.getAngle(servo_to_channel(servo, 0));
      break;
    case 1:
      angle = servo2.getAngle(servo_to_channel(servo, 0));
      break;
    case 2:
      angle = servo1.getAngle(servo_to_channel(servo, 1));
      break;
    case 3:
      angle = servo2.getAngle(servo_to_channel(servo, 1));
      break;
  }
  return angle;
}

/*
  - trans site from polar to microservos
  - mathematical model map to fact
  - the errors saved in eeprom will be add
   ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  alpha += servo_error[leg][0];
  beta += servo_error[leg][1];
  gamma += servo_error[leg][2];
  
  if (leg == 0)
  {
    alpha = 90 + alpha;
    beta = 180 - beta;
    gamma = 90 - gamma; // I changed to 0, to avoid angle reach 140+
  }
  else if (leg == 1)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  else if (leg == 2)
  {
    alpha = 90  - alpha;
    beta = beta;
    gamma += 90;
  }
  else if (leg == 3)
  {
    alpha = 90 + alpha;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }

  alpha = constrain(alpha, 0, 180);
  beta = constrain(beta, 0, 180);
  gamma = constrain(gamma, 0, 180);

  if (leg == 0)
  {
    //Serial.println("A\t "+String(leg)+"\t"+String(alpha)+"\t"+String(beta)+"\t"+String(gamma));
  }

  angle_expect[leg][0] = alpha;
  angle_expect[leg][1] = beta;
  angle_expect[leg][2] = gamma;
  

  servo_write(leg, 0, alpha, move_speed);
  servo_write(leg, 1, beta, move_speed);
  servo_write(leg, 2, gamma, move_speed);
}
/*
  - check lipo voltage to see if it is below minimum
  - blocking if voltage less than minimum
  - JC
  */
void voltage_check_service(void)
{
  float voltage_A0 = analogRead(A0)*vcc/1023; // 1C
  float voltage_A1 = analogRead(A1)*2*vcc/1023; // 2C combined voltage
  float voltage_A2 = analogRead(A2)*3*vcc/1023; // 3C combined voltage
  float cell[3] = {voltage_A0, voltage_A1 - voltage_A0, voltage_A2 - voltage_A1};  // workout individual cell voltage
  bool cell_under_voltage = FALSE;
  
  int cells_in_use = 3;
  if (voltage_A2 < 1)
  {
    cells_in_use = 2;
    Serial.println("Battery : 2C Cell 1 = " + String(cell[0]) + " Cell 2 = " + String(cell[1]));
  }
  else
  {
    Serial.println("Battery : 3C Cell 1 = " + String(cell[0]) + " Cell 2 = " + String(cell[1]) + " Cell 3 =  " + String(cell[2]));
  }

  // check each cell in use for under voltage. if found increment undervoltage count and stop robot if threshold is reached.
  for (int i =0 ; i< cells_in_use; i++)
  {
    if( cell[i] < minimum_voltage)  // if any cell less than the minimum detach servos and block return
    {
      cell_under_voltage=TRUE;
      if (count_on_under_voltage++ >= consecutive_under_voltage_readings) // x consecutive under voltage readings cause robot to stop
      {
        //servo_detach();
        while(1)
        {
          delay(10000);
        }
      }
    }
  }
  if (!cell_under_voltage) // if we get low readings then a normal reading then reset count
  {
    count_on_under_voltage = 0;
  } 
}

void servo_service(void)
{
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }
    //Serial.println("S\t "+String(i)+"\t"+String(site_expect[i][0])+"\t"+String(site_expect[i][1])+"\t"+String(site_expect[i][2]));
    cartesian_to_polar(alpha, beta, gamma, site_expect[i][0], site_expect[i][1], site_expect[i][2]);
    //Serial.println("A\t "+String(i)+"\t"+String(alpha)+"\t"+String(beta)+"\t"+String(gamma));
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
  if (( rest_counter % 500 == 0) && (monitor_voltage) && false)
  {
    //voltage_check_service();
  }
}
