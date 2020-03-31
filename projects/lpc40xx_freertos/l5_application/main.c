#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "board_io.h"
#include "common_macros.h"
#include "gpio.h"
#include "periodic_scheduler.h"
#include "sj2_cli.h"

#define BIT_1 (1 << 1) // battery monitor task 
#define BIT_2 (1 << 2) // flight controller task
#define BIT_3 (1 << 3) // data logging task
#define BIT_4 (1 << 4) // sensor communication task

static EventGroupHandle_t xEventGroup;
static EventBits_t uxBits;

void battery_monitor(void *params);
void quadcopter_flight_controller(void *params);
void data_logging(void *params);
void sensor_values(void *params);

int main(void) {

  TaskHandle_t battery_monitor, quadcopter_flight_controller, data_logging, sensor_values, cli, watchdog;
  xTaskCreate(battery_monitor, "battery_monitor", 2048 / (* void), NULL, 6, &battery_monitor);
  xTaskCreate(quadcopter_flight_controller, "quadcopter_flight_controller", 4096 / (* void), NULL, 10, &battery_monitor);
  xTaskCreate(data_logging, "data_logging", 2048 / (* void), NULL, 6, &data_logging);
  xTaskCreate(sensor_values, "sensor_values", 2048 / (* void), NULL, 8, &sensor_values);
  xTaskCreate(sj2_cli__init, "cli", 2048 / sizeof(void *), NULL, 2, &cli);
  xTaskCreate(watchdog_task, "watchdog", 2048 / sizeof(void *), NULL, 10, &watchdog);

  xEventGroup = xEventGroupCreate();


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
    uxBits = xEventGroupSetBits(xEventGroup, BIT_3);
  }
}
void sensor_values(void *params) {

  // implement I2C lab here

  while(1) {
    uxBits = xEventGroupSetBits(xEventGroup, BIT_4);
  }
}

void watchdog_task(void *params) {
  fprintf(stderr, "In\n");
  while (1) {
    // ...
    // vTaskDelay(200);
    // We either should vTaskDelay, but for better robustness, we should
    // block on xEventGroupWaitBits() for slightly more than 100ms because
    // of the expected production rate of the producer() task and its check-in
    // fprintf(stderr, "In while\n");
    /**if (xEventGroupWaitBits(xEventGroup, BIT_1 | BIT_2, pdTRUE, pdFALSE,
                            205 / portTICK_PERIOD_MS)) { // TODO: read up on port tick
      // if at least a bit is set
      // TODO
      fprintf(stderr, "At least one bit is set\n");
      fprintf(stderr, "uxBits: %d\n", uxBits);

    }**/
    vTaskDelay(500);
    uxBits = xEventGroupWaitBits(xEventGroup, BIT_1 | BIT_2 | BIT_3 | BIT_4, pdTRUE, pdFALSE, 205 / portTICK_PERIOD_MS);
    if ((uxBits & (BIT_1 | BIT_2)) == (BIT_1 | BIT_2)) {
      // xEventGroupWaitBits() returned because both bits were set.
      fprintf(stderr, "all tasks are working\n");

    } else {
      if ((uxBits & BIT_1) != (BIT_1)) {
        fprintf(stderr, "battery monitor task is not working/suspended\n");
      }
      if ((uxBits & BIT_2) != (BIT_2)) {
        fprintf(stderr, "flight controller task is not working/suspended\n");
      }
      if ((uxBits & BIT_2) != (BIT_2)) {
        fprintf(stderr, "data logging task is not working/suspended\n");
      }
      if ((uxBits & BIT_2) != (BIT_2)) {
        fprintf(stderr, "sensor value communication is not working/suspended\n");
      }
    }
    printf("uxBits: %ld\n", uxBits);
  }
}