#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>

#define TIME_STEP 8
#define NB_GROUND_SENS 8
#define NB_LEDS 5

// IR Ground Sensors
WbDeviceTag gs[NB_GROUND_SENS];
// LEDs
WbDeviceTag led[NB_LEDS];
// Motors
WbDeviceTag left_motor, right_motor;

#define NOP -1
#define MID 0
#define RIGHT30 4
#define RIGHT45 5
#define RIGHT60 6
#define LEFT30 1
#define LEFT45 2
#define LEFT60 3
#define LEFT90 7
#define STOP_SIGNAL 11

#define MAX_SPEED 8

unsigned short threshold[NB_GROUND_SENS] = {200, 200, 300, 300, 300, 300, 200, 200};
unsigned int filted[8] = {0, 0, 0, 0, 0, 0, 0, 0};

double left_ratio = 0.0;
double right_ratio = 0.0;

void constrain(double *value, double min, double max) {
  if (*value > max) *value = max;
  if (*value < min) *value = min;
}

void ReadSensors() {
  unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < NB_GROUND_SENS; i++) {
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    filted[i] = (gs_value[i] < threshold[i]) ? 1 : 0;
  }
}

int Position() {
  // Adjusting based on sensor combinations to be more precise
  if (filted[3] == 1 && filted[4] == 1) return MID;
  if (filted[4] == 1 && filted[5] == 1) return LEFT30;
  if (filted[5] == 1 && filted[6] == 1) return LEFT45;
  if (filted[6] == 1 && filted[7] == 1) return LEFT60;
  if (filted[2] == 1 && filted[3] == 1) return RIGHT30;
  if (filted[1] == 1 && filted[2] == 1) return RIGHT45;
  if (filted[0] == 1 && filted[1] == 1) return RIGHT60;
  if (filted[0] == 0 && filted[1] == 0 && filted[2] == 0 && filted[3] == 0 && filted[4] == 0 && filted[5] == 0 && filted[6] == 0 && filted[7] == 0) return MID;
  if (filted[0] == 1 && filted[1] == 1 && filted[2] == 1 && filted[3] == 1 && filted[4] == 1) return LEFT90;
  return NOP;
}

void GoStraight() { left_ratio = 4.0; right_ratio = 4.0; }
void GoLEFT30() { left_ratio = 3.5; right_ratio = 1.0; }  // Smoother turns
void GoLEFT45() { left_ratio = 4.0; right_ratio = 1.0; }
void GoLEFT60() { left_ratio = 5.0; right_ratio = 1.0; }
void GoRIGHT30() { left_ratio = 1.0; right_ratio = 3.5; }
void GoRIGHT45() { left_ratio = 1.0; right_ratio = 4.0; }
void GoRIGHT60() { left_ratio = 1.0; right_ratio = 5.0; }
void GoLEFT90() { left_ratio = 10.0; right_ratio = 0.5; }  // More gradual turn
void Stop() { left_ratio = 0; right_ratio = 0; }

int main() {
  wb_robot_init();

  // Initialize sensors
  char name[20];
  for (int i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }

  // Initialize LEDs
  for (int i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name);
    wb_led_set(led[i], 1);
  }

  // Initialize motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    ReadSensors();

    // Gracefully starting movement
    if (wb_robot_get_time() < 0.5) {
      GoStraight();
    }

    // Display filtered sensor values
    printf("\n\t\tPosition : 0b");
    for (int i = 0; i < 8; i++) {
      printf("%u", filted[i]);
    }

    // Handle different positions based on sensor input
    switch (Position()) {
      case MID: GoStraight(); break;
      case LEFT30: GoLEFT30(); break;
      case LEFT45: GoLEFT45(); break;
      case LEFT60: GoLEFT60(); break;
      case LEFT90: GoLEFT90(); break;
      case RIGHT30: GoRIGHT30(); break;
      case RIGHT45: GoRIGHT45(); break;
      case RIGHT60: GoRIGHT60(); break;
      default: Stop(); break;
    }

    // Set motor velocities based on the ratio calculations
    wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
    wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
  }

  wb_robot_cleanup();
  return 0;
}
