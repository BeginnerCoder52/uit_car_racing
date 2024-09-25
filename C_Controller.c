#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>

#define TIME_STEP 8

// Số lượng cảm biến mặt đất
#define NB_GROUND_SENS 8
#define NB_LEDS 5

// Cảm biến IR trên mặt đất
WbDeviceTag gs[NB_GROUND_SENS];

// Đèn LED
WbDeviceTag led[NB_LEDS];

// Động cơ
WbDeviceTag left_motor, right_motor;

// Tốc độ tối đa và tối thiểu
#define MAX_SPEED 20
#define MIN_SPEED -10

// Ngưỡng cảm biến
unsigned short threshold[NB_GROUND_SENS] = { 300, 300, 300, 300, 300, 300, 300, 300 };
unsigned int filted[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Biến điều khiển PID
double Kp = 0.1;  // Hệ số tỷ lệ (Proportional)
double Ki = 0.0;  // Hệ số tích phân (Integral)
double Kd = 0.1;  // Hệ số đạo hàm (Derivative)
double previous_error = 0.0;
double integral = 0.0;
double derivative = 0.0;
double error = 0.0;

// Hàm giới hạn giá trị trong một khoảng cho trước
void constrain(double *value, double min, double max) {
  if (*value > max) *value = max;
  if (*value < min) *value = min;
}

// Hàm đọc giá trị cảm biến
void ReadSensors() {
  unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < NB_GROUND_SENS; i++) {
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    // So sánh giá trị cảm biến với ngưỡng và chuyển thành nhị phân
    if (gs_value[i] < threshold[i]) {
      filted[i] = 1;
    } else {
      filted[i] = 0;
    }
  }
}


float GetLinePositionError() {
  int weighted_sum = 0;
  int sum = 0;
  
  // Tính tổng trọng số để tính sai số dựa trên vị trí cảm biến
  for (int i = 0; i < NB_GROUND_SENS; i++) {
    weighted_sum += filted[i] * (i - (NB_GROUND_SENS / 2 - 0.5));
  // -3 tới +3 nếu có 8 cảm biến
    sum += filted[i];
  }
  
  // Nếu không phát hiện đường, trả về 0
  if (sum == 0) return 0.0f;
  
  // Sai số là giá trị trung bình trọng số của các cảm biến (dạng số thực)
  return (float)weighted_sum / sum;
}


// Hàm điều khiển PID để điều chỉnh động cơ
void PIDControl() {
  // Lấy giá trị sai số từ cảm biến
  error = GetLinePositionError();

  // Tính toán tích phân và đạo hàm của sai số
  integral += error;
  derivative = error - previous_error;

  // Tính toán tín hiệu điều khiển PID
  double adjustment = Kp * error + Ki * integral + Kd * derivative;

  // Điều chỉnh tốc độ cho các động cơ dựa trên sai số
  double left_ratio = 0.2 - adjustment;
  double right_ratio = 0.2 + adjustment;

  // Tăng tốc độ khi rẽ
 // Điều chỉnh tốc độ khi rẽ dựa trên sai số
double base_speed = 1;  // Tốc độ cơ bản cho cả hai bánh
double speed_difference = 3;  // Hiệu tốc độ mong muốn khi sai số lớn

// Nếu sai số > 0, robot cần rẽ phải
if (error == 1  ) {
  right_ratio = base_speed - 5;  // Giữ tốc độ bánh phải cố định
  left_ratio = base_speed + speed_difference * (error / (NB_GROUND_SENS / 2));
    // Tăng tốc độ bánh trái dựa trên sai số
   // printf("Error: %.2f, Left ratio: %.2f, Right ratio: %.2f\n", error, left_ratio, right_ratio);
} 
// Nếu sai số < 0, robot cần rẽ trái
else if (error == -1) {
  left_ratio = base_speed - 5;  // Giữ tốc độ bánh trái cố định
  right_ratio = base_speed + speed_difference * (-error / (NB_GROUND_SENS / 2));
    // Tăng tốc độ bánh phải dựa trên sai số
   // printf("Error: %.2f, Left ratio: %.2f, Right ratio: %.2f\n", error, left_ratio, right_ratio);
}
else if (error == 2) {
  left_ratio = base_speed +5;  // Giữ tốc độ bánh trái cố định
  right_ratio = base_speed-1 ;
    // Tăng tốc độ bánh phải dựa trên sai số
 //   printf("Error: %.2f, Left ratio: %.2f, Right ratio: %.2f\n", error, left_ratio, right_ratio);

}

else if (error == -2) {
  left_ratio = base_speed -1;  // Giữ tốc độ bánh trái cố định
  right_ratio = base_speed + 5 ;
    // Tăng tốc độ bánh phải dựa trên sai số
   // printf("Error: %.2f, Left ratio: %.2f, Right ratio: %.2f\n", error, left_ratio, right_ratio);
}
else if (error == -0.5) {
  left_ratio = base_speed -1;  // Giữ tốc độ bánh trái cố định
  right_ratio = base_speed + 5 ;
  }
  
else if (error == 0.5) {
  left_ratio = base_speed +1;  // Giữ tốc độ bánh trái cố định
  right_ratio = base_speed -1  ;
  }

printf("Error: %.2f, Left ratio: %.2f, Right ratio: %.2f\n", error, left_ratio, right_ratio);
  // Giới hạn giá trị để đảm bảo tốc độ không vượt quá mức cho phép
  constrain(&left_ratio, MIN_SPEED, MAX_SPEED);
  constrain(&right_ratio, MIN_SPEED, MAX_SPEED);

  // Cập nhật sai số trước đó
  previous_error = error;

  // Đặt vận tốc cho động cơ
  wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
  wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
}

int main() {
  
  // Khởi tạo robot
  wb_robot_init();    

  // Khởi tạo các cảm biến
  char name[20];
  for (int i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name);  
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }

  // Khởi tạo đèn LED
  for (int i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name);
    wb_led_set(led[i], 1);
  }  
  
  // Khởi tạo động cơ
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
    
  // Vòng lặp chính
  while (wb_robot_step(TIME_STEP) != -1) {
    // Đọc giá trị cảm biến
    ReadSensors();
    // In trạng thái của các cảm biến
    printf("Sensor state: ");
    for (int i = 0; i < NB_GROUND_SENS; i++) {
      printf("%d", filted[i]);  // In 0 hoặc 1 cho từng cảm biến
    }
    printf("\n");
    

    PIDControl();

    
  //  printf("Error: %.2f, Left ratio: %.2f, Right ratio: %.2f\n", error, left_ratio, right_ratio);
  }
   
  // Dọn dẹp sau khi chương trình kết thúc
  wb_robot_cleanup();
  return 0;
}
