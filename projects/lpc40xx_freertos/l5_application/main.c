#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "ff.h"
#include "acceleration.h"
#include "board_io.h"
#include "common_macros.h"
#include "gpio.h"
#include "periodic_scheduler.h"
#include "sj2_cli.h"
#include "event_groups.h"
#include "uart_lab.h"
#include "flight_control.h"

#define BIT_1 (1 << 1) // battery monitor task 
#define BIT_2 (1 << 2) // flight controller task
#define BIT_3 (1 << 3) // data logging task
#define BIT_4 (1 << 4) // sensor communication task

static EventGroupHandle_t xEventGroup;
static EventBits_t uxBits;
static QueueHandle_t stateSpaceQueue;

void battery_monitor(void *params);
void quadcopter_flight_controller(void *params);
void data_logging(void *params);
void sensor_values(void *params);
void write_file_using_fatfs_pi(acceleration__axis_data_s *sensor_value);

int main(void) {

  uart_lab__init(UART_2, 96000000, 115200);
  gpio__construct_with_function(GPIO__PORT_2, 8, GPIO__FUNCTION_2);
  gpio__construct_with_function(GPIO__PORT_2, 9, GPIO__FUNCTION_2);
  // TODO: implement uart__enable_receive_interrupt(0);
  TaskHandle_t battery_monitor, quadcopter_flight_controller, data_logging, sensor_values, cli, watchdog;
  xTaskCreate(battery_monitor, "battery_monitor", 2048 / (void *), NULL, 6, &battery_monitor);
  xTaskCreate(quadcopter_flight_controller, "quadcopter_flight_controller", 4096 / (void *), NULL, 10, &battery_monitor);
  xTaskCreate(data_logging, "data_logging", 2048 / (void *), NULL, 6, &data_logging);
  xTaskCreate(sensor_values, "sensor_values", 2048 / (void *), NULL, 8, &sensor_values);
  xTaskCreate(sj2_cli__init, "cli", 2048 / sizeof(void *), NULL, 2, &cli);
  xTaskCreate(watchdog_task, "watchdog", 2048 / sizeof(void *), NULL, 10, &watchdog);

  xEventGroup = xEventGroupCreate();
  stateSpaceQueue = xQueueCreate(10, sizeof(state_space));

  vTaskStartScheduler(); // This function never returns unless RTOS scheduler
                         // runs out of memory and fails

  return 0;
}

void battery_monitor(void *params){
  while(1) {
    
    uxBits = xEventGroupSetBits(xEventGroup, BIT_1);
  }
}

void quadcopter_flight_controller(void *params) {

  while(1) {
    uxBits = xEventGroupSetBits(xEventGroup, BIT_2);
  }
}
void data_logging(void *params) {

  // write to SD card
  
  while(1) {
    
    // TODO: what types of information needs to be logged?
    acceleration__axis_data_s sensor_value;
    write_file_using_fatfs_pi(&sensor_value);
    uxBits = xEventGroupSetBits(xEventGroup, BIT_3);
  }
}

void write_file_using_fatfs_pi(acceleration__axis_data_s *sensor_value) {
  const char *filename = "sensor.txt";
  FIL file; // File handle
  UINT bytes_written = 0;
  FRESULT result = f_open(&file, filename, (FA_OPEN_APPEND | FA_WRITE));

  if (FR_OK == result) {
    char string[64];
    sprintf(string, "%li, %i, %i, %i\n", xTaskGetTickCount(), sensor_value->x, sensor_value->y, sensor_value->z);
    if (FR_OK == f_write(&file, string, strlen(string), &bytes_written)) {
      // printf("sent");
    } else {
      printf("ERROR: Failed to write data to file\n");
    }
    f_close(&file);
  } else {
    printf("ERROR: Failed to open: %s because %d\n", filename, result);
  }
}

void sensor_values(void *params) {

  // No markers being sent
  // 15-state vector [x, y, z, roll, pitch, yaw, x', y', z', roll', pitch', yaw', x'', y'', z'']
  // each state has two bytes, MSB sent first
  // 30 bytes total

  while (1) {
    char number_as_string[32] = {0};
    int counter = 0;
    char byte = 0;
    uart_lab__get_char_from_queue(&byte, 100000);
    printf("Received: %c\n", byte);
    
    
    if ('\0' == byte) {
      number_as_string[counter] = '\0';
      state_space *current;
      current->x = number_as_string[0] << 8 & number_as_string[1] << 0;
      current->y = number_as_string[2] << 8 & number_as_string[3] << 0;
      current->z = number_as_string[4] << 8 & number_as_string[5] << 0;
      current->roll = number_as_string[6] << 8 & number_as_string[7] << 0;
      current->pitch = number_as_string[8] << 8 & number_as_string[9] << 0;
      current->yaw = number_as_string[10] << 8 & number_as_string[11] << 0;
      current->x_vel = number_as_string[12] << 8 & number_as_string[13] << 0;
      current->y_vel = number_as_string[14] << 8 & number_as_string[15] << 0;
      current->z_vel = number_as_string[16] << 8 & number_as_string[17] << 0;
      current->d_roll = number_as_string[18] << 8 & number_as_string[19] << 0;
      current->d_pitch = number_as_string[20] << 8 & number_as_string[21] << 0;
      current->d_yaw = number_as_string[22] << 8 & number_as_string[23] << 0;
      current->x_acc = number_as_string[24] << 8 & number_as_string[25] << 0;
      current->y_acc = number_as_string[26] << 8 & number_as_string[27] << 0;
      current->z_acc = number_as_string[28] << 8 & number_as_string[29] << 0;
      xQueueSend(stateSpaceQueue, current, portMAX_DELAY);
      counter = 0;
      printf("Received this number from the other board: %s\n", number_as_string);
    /**} else if ('p' == byte) { // denotes position

    } else if ('e' == byte) { // denotes euler angles
    
    } else if ('v' == byte) { // denotes velocity
    
    } else if ('d' == byte) { // denotes d_euler angles
    
    } else if ('a' == byte) { // denotes acceleration**/
    
    } else {
      // We have not yet received the NULL '\0' char, so buffer the data
      number_as_string[counter] = byte;
      if (counter < 32) {
        counter++;
      } else {
        counter = 0;
      }
    }
    uxBits = xEventGroupSetBits(xEventGroup, BIT_4);
  }
}


void watchdog_task(void *params) {
  fprintf(stderr, "In\n");
  while (1) {
    
    vTaskDelay(500);
    uxBits = xEventGroupWaitBits(xEventGroup, BIT_1 | BIT_2 | BIT_3 | BIT_4, pdTRUE, pdFALSE, 1000 / portTICK_PERIOD_MS);
    if ((uxBits & (BIT_1 | BIT_2 | BIT_3 | BIT_4)) == (BIT_1 | BIT_2 | BIT_3 | BIT_4)) {
      // xEventGroupWaitBits() returned because both bits were set.
      fprintf(stderr, "all tasks are working\n");

    } else {
      if ((uxBits & BIT_1) != (BIT_1)) {
        fprintf(stderr, "battery monitor task is not working/suspended\n");
      }
      if ((uxBits & BIT_2) != (BIT_2)) {
        fprintf(stderr, "flight controller task is not working/suspended\n");
      }
      if ((uxBits & BIT_3) != (BIT_3)) {
        fprintf(stderr, "data logging task is not working/suspended\n");
      }
      if ((uxBits & BIT_4) != (BIT_4)) {
        fprintf(stderr, "sensor value communication is not working/suspended\n");
      }
    }
    printf("uxBits: %ld\n", uxBits);
  }
}
