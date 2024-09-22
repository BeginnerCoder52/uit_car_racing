#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/camera.h>
#include <webots/supervisor.h>
#define TIME_STEP 8

// 8 IR ground color sensors
#define NB_GROUND_SENS 8
#define NB_LEDS 5

// IR Ground Sensors
WbDeviceTag gs[NB_GROUND_SENS];

// LEDs 
WbDeviceTag led[NB_LEDS];

// Motors
WbDeviceTag left_motor, right_motor;

// Signals for the car
#define NOP  -1
#define MID   0
#define LEFT  2
#define RIGHT 2
#define a 0b00011111
#define noline 0b00000000
#define FULL_SIGNAL 3
#define BLANK_SIGNAL 4
#define STOP_SIGNAL 5

// Speed adjustment
#define MAX_SPEED 100
#define MIN_SPEED -10

// Sensor thresholds
unsigned short threshold[NB_GROUND_SENS] = { 300 , 300 , 300 , 300 , 300 , 300 , 300 , 300 };
unsigned int filted[8] = {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0};

// Speed ratio for motors
double left_ratio = 0.0;
double right_ratio = 0.0;



// Function to constrain values within a range
void constrain(double *value, double min, double max) {
  if (*value > max) *value = max;
  if (*value < min) *value = min;
}

// Function to read sensor values
void ReadSensors() {
  unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
  for(int i = 0; i < NB_GROUND_SENS; i++) {
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    // Compare gs_value with threshold -> binary conversion
    if (gs_value[i] < threshold[i])
      filted[i] = 1;
    else filted[i] = 0;
  }
}

// Function to determine the position of the line
int Position() {

    if (( filted[2] == 0 && filted[3] == 1 && filted[4] == 1 && filted[5] == 1 && filted[6] == 1 && filted[7] == 1 )) {
   
   return a;
   }
  // Center position (robot is aligned with the line)
  else if ((filted[3] == 1 && filted[4] == 1)) {
  printf("\tthang\t");
    return MID; 
  }
  // Turn left (left sensors detect the line)
  else if (filted[0] == 1 || filted[1] == 1 || filted[2] == 1) {
  printf("\ttrai\t ");
    return LEFT;
  }
  // Turn right (right sensors detect the line)
  else if (filted[5] == 1 || filted[6] == 1 || filted[7] == 1) {
  printf("phai");
    return RIGHT;
  }
   else if ( filted[6] == 1 && filted[7] == 1) {
   printf(" phai90");
    return a;
  }
  
  return NOP; 
}

// Function to move the robot straight
void GoStraight() {
  left_ratio = 0.2;
  right_ratio = 0.2;
  printf("Toc do trai: %.2f, Toc do phai: %.2f\n", left_ratio, right_ratio);
}

// Function to turn the robot left
void TurnLeft() {

  left_ratio = 0.05;  // Slow down the left motor
  right_ratio = 0.4; // Full speed on the right motor
     // Quay trong một khoảng thời gian đủ để đạt góc 90 độ
  
}

// Function to turn the robot right
void TurnRight() {
  left_ratio = 3;   // Bánh trái quay nhanh hơn
  right_ratio = -2;  // Bánh phải quay chậm hơn (góc cong nhẹ)
printf("Rẽ trái 45 độ - Tốc độ trái: %.2f, Tốc độ phải: %.2f\n", left_ratio, right_ratio);
  // Duy trì tốc độ này trong một khoảng thời gian phù hợp để rẽ 45 độ
  for (int i = 0; i < 15; i++) {
    wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
    wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
    wb_robot_step(TIME_STEP);  // Chờ một khoảng thời gian ngắn giữa mỗi bước
  printf("Rẽ trái 45 độ - Tốc độ trái: %.2f, Tốc độ phải: %.2f\n", left_ratio, right_ratio);
  }
  }

void righta() {
left_ratio = 2.1;  // Full speed on the left motor
  right_ratio = 2;

for ( int i=0;i<2;i++)
{
   // Cập nhật tốc độ
    wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
    wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);

  
    // In thông tin tốc độ để kiểm tra
    printf("Quay phải - Toc do trai: %.2f, Toc do phai: %.2f\n", left_ratio, right_ratio);
// Tạm dừng 1 giây để quay
    wb_robot_step(TIME_STEP);
  }

left_ratio = 3;  // Full speed on the left motor
  right_ratio = -2;
  
for ( int i=0;i<3;i++)
{
   // Cập nhật tốc độ
    wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
    wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);

  
    // In thông tin tốc độ để kiểm tra
    printf("Quay phải - Toc do trai: %.2f, Toc do phai: %.2f\n", left_ratio, right_ratio);
// Tạm dừng 1 giây để quay
    wb_robot_step(TIME_STEP);
  }
  }
  
int main() {
  
  // Initialize the robot
  wb_robot_init();    

  /* get and enable the camera and accelerometer */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 64);

  /* Initialization */
  char name[20];
  for (int i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }

  for (int i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name);
    wb_led_set(led[i], 1);
  }  
  
  // Motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
    
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    ReadSensors();   
    
    // Print sensor values for debugging
    printf("\n\t\tPosition : 0b");
    for (int i = 0; i < 8; i++) {
      printf("%u", filted[i]);
    }
    
    // Control logic based on sensor readings
    int position = Position();
    
    if (position == MID) {
      GoStraight();
    } else if (position == LEFT) {
      TurnLeft();
    } else if (position == RIGHT) {
      TurnRight();   
    } 
    
     else if ( position == a){
      righta(25);
      
       }
      

    // Set motor speeds based on ratio
    wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
    wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
  }
   
  wb_robot_cleanup();
  return 0;
  }
