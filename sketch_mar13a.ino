///////////////////////////////////////////////////
// Control 3 servo motors using 3 potentiometers //
///////////////////////////////////////////////////
#include <Servo.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 13
// Control pins
int Joint1ControlPin = A1;
int Joint2ControlPin = A2;
int Joint3ControlPin = A3;
// Control values
int Joint1Control = 512; // middle value between 0 and 1024
int Joint2Control = 512; // middle value between 0 and 1024
int Joint3Control = 512; // middle value between 0 and 1024
//
float controlx;
float controly;
float controlz;
//coordinates

float L2=9.8;
float L3=16.8;

// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;
// Gripper Values
/*
 * 30: 3.2cm
 * 60: 2.9cm
 * 90: 2.4cm
 * 120: 1.4cm
 * 150: 0.2cm
 * 180: 0cm
 */

// Gripper Values
int GripperOpen = 30; // Open gripper
int GripperClosed = 120; // Close gripper

// Joint Angle Offsets
int Joint1Offset = -10; // Your value may be different
int Joint2Offset = 5; // Your value may be different
int Joint3Offset = 45; // Your value may be different

// Given an xyz point in cartesian space, return an array of length 3 of the angles each joint has to take
// Eg. [theeta1, theeta2. theeta3]
double* get_joint_angles(double x, double y, double z) {
  z = -z;
  double alpha= atan2(z,sqrt(pow(x,2)+pow(y,2)))*(180/PI);
  double costheta2=(pow(L2,2)+pow(x,2)+pow(y,2)+pow(z,2)-pow(L3,2))/(2*L2*sqrt(pow(x,2)+pow(y,2)+pow(z,2)));
  double sintheta2=sqrt(1-pow(costheta2,2));
  double costheta3=(pow(L2,2)+pow(L3,2)-pow(x,2)-pow(y,2)-pow(z,2))/(2*L2*L3);
  double sintheta3=sqrt(1-pow(costheta3,2));

  double theeta1 = atan2(y , x)*(180/PI);
  double theeta2 = alpha + atan2(sintheta2,costheta2)*(180/PI);
  double theeta3 = 180-atan2(sintheta3,costheta3)*(180/PI);
    
  static double angles[3]; // make an array of size 3
  angles[0] = theeta1 + Joint1Offset;
  angles[1] = theeta2 + Joint2Offset;
  angles[2] = 180 - (theeta3 + Joint3Offset);

  return angles; // return a pointer to the angles array
}

// Get the point on an arbitrary axis the end effector should be at for a given time according to a cubic trajectory
// Part of trajectory planning
double get_cartesian_position(double u_start, double u_end, double total_time, double t) {
  double a0 = u_start; // constant term
  double a2 = (3/pow(total_time,2))*(u_end - u_start); // t^2 coefficient
  double a3 = (-2/pow(total_time,3))*(u_end - u_start); // t^3 coefficient

  double pos = a0 + (a2*pow(t,2)) + (a3*pow(t,3)); // the cubic
  return pos;
}

// move the end effector from (x0, y0, z0) to (x1, y1, z1) over tf millisecond
void movePoint(double x_start, double y_start, double z_start, double x_end, double y_end, double z_end, int tf) {

  // This for loop generates a array of length 50 called cartesian_positions which stores all the xyz positions the end effector should be at
  // Each point is represented as an array of length 3: [x, y z]
  static double cartesian_positions[50][3];
  for (int i = 0; i < 50; i++) {
    double t = i*100;
    cartesian_positions[i][0] = get_cartesian_position(x_start, x_end, tf, t);
    cartesian_positions[i][1] = get_cartesian_position(y_start, y_end, tf, t);
    cartesian_positions[i][2] = get_cartesian_position(z_start, z_end, tf, t);
  }

  // This for loop generates a array of length 50 called joint_angles which stores all the angles each joint should be at
  // Each point is represented as an array of length 3: [theeta1, theeta2, theeta3]
  static double joint_angles[50][3];
  for (int i = 0; i < 50; i++) {
    // for each cartesian position get a pointer to an array containing the join angles for that position
     double *p_angles;
     p_angles = get_joint_angles(
      cartesian_positions[i][0],
      cartesian_positions[i][1],
      cartesian_positions[i][2]
     );

     // store the join angles for joint 1, 2 and 3 in the i-th position in in the joint_angles array 
     joint_angles[i][0] = *p_angles; // theeta1
     joint_angles[i][1] = *(p_angles+1); // theeta2
     joint_angles[i][2] = *(p_angles+2); // theeta3
  }

  // For each joint angle from the start to the position move the joints to the intermediary angles and pause between them
  for (int i = 0; i < 50; i++) {
    Joint1.write(joint_angles[i][0]);
    Joint2.write(joint_angles[i][1]);
    Joint3.write(joint_angles[i][2]);
    delay(tf/50);
  }
  
}

void openGripper()
{
  Gripper.write(GripperOpen);
}

void closeGripper()
{
  Gripper.write(GripperClosed);
}

// Set the gripper to have distance cms between the claws
void setGripper(double distance)
{
  double gripperValue = 80*acos(distance/3.5) + 30;
  Gripper.write(gripperValue);
}

void setup()
{
  Serial.begin(9600);
  Joint1.attach(Joint1Pin);
  Joint2.attach(Joint2Pin);
  Joint3.attach(Joint3Pin);
  Gripper.attach(GripperPin);

  // the robot 'home' position
  float x = 0;
  float y = L3;
  float z = -L2;

  double *p_angles;
  p_angles = get_joint_angles(x, y, z);
  Joint1.write(*p_angles);
  Joint2.write(*(p_angles+1));
  Joint3.write(*(p_angles+2));
  openGripper();
  delay(1000); // 1 seconds before robot starts loop
}

void loop()
{

  // Home Position
  float x0 = 0;
  float y0 = L3;
  float z0 = -L2;

  // Object Position
  double obj_x = 3.5;
  double obj_y = L3+3.5;
  double obj_z = 7;

  // Above Object
  double x1 = obj_x;
  double y1 = obj_y;
  double z1 = -L2;

  movePoint(x0, y0, z0, x1, y1, z1, 5000); // go to above object
  delay(1000);
  
  movePoint(x1, y1, z1, obj_x, obj_y, obj_z, 5000); // go to object
  closeGripper(); // pick up object
  delay(1000);

  movePoint(obj_x, obj_y, obj_z, x0, y0, z0, 5000); // return to start
  delay(1000);
  openGripper(); // drop object
  delay(100000); 
  
}

  
  
  

  
  // Read Potentiometer Values
  //Joint1Control = analogRead(Joint1ControlPin);
  //Joint2Control = analogRead(Joint2ControlPin);
  //Joint3Control = analogRead(Joint3ControlPin);
  //
  //controlx=map(Joint1Control,0,1023,-26.6,26.6);
  //controly=map(Joint2Control,0,1023,-26.6,26.6);
  //controlz=map(Joint3Control,0,1023,-16.8,26.6);
  //
//  controlx = 0;
//  controly = 16.8;
//  controlz = 9.8;
//  double alpha= atan2(controlz,sqrt(pow(controlx,2)+pow(controly,2)))*(180/PI);
//  double costheta2=(pow(L2,2)+pow(controlx,2)+pow(controly,2)+pow(controlz,2)-pow(L3,2))/(2*L2*sqrt(pow(controlx,2)+pow(controly,2)+pow(controlz,2)));
//  double sintheta2=sqrt(1-pow(costheta2,2));
//  double costheta3=(pow(L2,2)+pow(L3,2)-pow(controlx,2)-pow(controly,2)-pow(controlz,2))/(2*L2*L3);
//  double sintheta3=sqrt(1-pow(costheta3,2));
//  int Joint1Angle = atan2(controly , controlx)*(180/PI);
//  int Joint2Angle = alpha + atan2(sintheta2,costheta2)*(180/PI);
//  int Joint3Angle = 180-atan2(sintheta3,costheta3)*(180/PI);
//  //
//  
//  //// Map Analog-Digital-Converted Values into Angles
//  //Joint1Angle = map(Joint1Control,0,1023,0,180);
//  //Joint2Angle = map(Joint2Control,0,1023,0,180);
//  //Joint3Angle = map(Joint3Control,0,1023,0,180);
//  Serial.print("Joint 1: ");
//  Serial.print(Joint1Angle);
//  Serial.print(", Joint 2: ");
//  Serial.print(Joint2Angle);
//  Serial.print(", Joint 3: ");
//  Serial.println(Joint3Angle);
//  Joint1.write(Joint1Angle+Joint1Offset);
//  Joint2.write(Joint2Angle+Joint2Offset);
//  Joint3.write(180-(Joint3Angle+Joint3Offset));
//  delay(10);
  
