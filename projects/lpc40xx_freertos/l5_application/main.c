#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "board_io.h"
#include "common_macros.h"
#include "gpio.h"
#include "periodic_scheduler.h"
#include "sj2_cli.h"

void battery_monitor(void *params);
void quadcopter_flight_controller(void *params);
void data_logging(void *params);
void sensor_values(void *params);

int main(void) {

  TaskHandle_t battery_monitor, quadcopter_flight_controller, data_logging, sensor_values, cli;
  xTaskCreate(battery_monitor, "battery_monitor", 2048 / (* void), NULL, 6, &battery_monitor);
  xTaskCreate(quadcopter_flight_controller, "quadcopter_flight_controller", 4096 / (* void), NULL, 10, &battery_monitor);
  xTaskCreate(data_logging, "data_logging", 2048 / (* void), NULL, 6, &data_logging);
  xTaskCreate(sensor_values, "sensor_values", 2048 / (* void), NULL, 8, &sensor_values);
  xTaskCreate(sj2_cli__init, "cli", 2048 / sizeof(void *), NULL, 2, &cli);


  vTaskStartScheduler(); // This function never returns unless RTOS scheduler
                         // runs out of memory and fails

  return 0;
}

void battery_monitor(void *params){
  while(1) {
    
  }
}

void quadcopter_flight_controller(void *params) {

  while(1) {
    
  }
}
void data_logging(void *params) {

  // write to SD card
  
  while(1) {
    
  }
}
void sensor_values(void *params) {

  // implement I2C lab here

  while(1) {
    
  }
}