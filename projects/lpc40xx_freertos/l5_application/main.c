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

  uart_lab__init(UART_2, 96000000, 9600);
  gpio__construct_with_function(GPIO__PORT_2, 8, GPIO__FUNCTION_2);
  gpio__construct_with_function(GPIO__PORT_2, 9, GPIO__FUNCTION_2);
  // TODO: implement uart__enable_receive_interrupt(0);
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

  // TODO: implement reciever task here

  while(1) {
    uxBits = xEventGroupSetBits(xEventGroup, BIT_4);
  }
}

// TODO: implement this task or (code) on RPi side
void rpi_sender_task(void *p) {
  char number_as_string[16] = {0};
  while (true) {
    const int number = rand();
    sprintf(number_as_string, "%i", number);
    // Send one char at a time to the other board including terminating
    NULL char
    for (int i = 0; i <= strlen(number_as_string); i++) {
      uart_lab__polled_put(0, number_as_string[i]);
      printf("Sent: %c\n", number_as_string[i]);
    }
    printf("Sent: %i over UART to the other board\n", number);
    vTaskDelay(3000);
  }
}


void uart__init(uart_number_e uart, uint32_t peripheral_clock, uint32_t baud_rate) {
  // Refer to LPC User manual and setup the register bits correctly
  // The first page of the UART chapter has good instructions
  // a) Power on Peripheral
  // b) Setup DLL, DLM, FDR, LCR registers
  // using UART2
  // LPC_SC->PCONP |= (1 << 23);
  lpc_peripheral__turn_on_power_to(LPC_PERIPHERAL__UART2);
  uint16_t div = peripheral_clock / (16 * baud_rate);
  LPC_UART2->LCR |= (1 << 7);
  LPC_UART2->DLM |= (div >> 8 & 0xFF);
  LPC_UART2->DLL |= (div >> 0 & 0xFF);
  LPC_UART2->FDR = (1 << 4);
  LPC_UART2->LCR &= ~(1 << 7);
  LPC_UART2->LCR &= ~(1 << 3);
  LPC_UART2->LCR |= (3 << 0); // WLS
  LPC_UART2->LCR &= ~(1 << 2); // two stop bits
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
