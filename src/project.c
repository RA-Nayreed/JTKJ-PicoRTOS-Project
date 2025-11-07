#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include "tkjhat/sdk.h"
// Copied and pasted these from main.c

// This file to be used for the project itself. Main.c was exercise session 2.
// Although the tasks are missing from it because it was never pushed to the repository.

typedef enum {
    WAITING,
    DISPLAYING,
    DATA_READY,
    RECORDING
} programState;

char current_character;
char current_str[]; // string of all the characters received so far (TIER 2)
programState state = WAITING;

// Prototypes
void displayOutput(char current_char);
void detectMovement();
void button_interrupt();
void sensor_task();
void button_task();



// I'm guessing we should make a statemachine task that uses switch statements
// to determine which state we're in?




void detectMovement() {
    /*
    Detect movement from the device and save the corresponding character to current_character.
    */
    int64_t ax, ay, az, gx, gy, gz, t;
      if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {

        if (az < -0.8) {
            current_character = '-';
        }
        else if (ay < -0.8) {
            current_character = '.';
        }
      }
    await(100); // wait for 100 ms, prevent flooding
}


void button_interrupt_space(int input) {
    /*
    Button interruption function
    */
    current_character = ' ';
}

void button_interrupt_record(int input) {
    /*
    Button interruption function for button 2
    */
    if (state == RECORDING)
        state = WAITING;
    else if (state == WAITING)
        state = RECORDING;
}


// Terminal communication
void displayOutput(char current_char) {
    /*
    Displays the current morse code character.
    */
    printf("%s\n", current_char);
    current_character = "\0"; // Reset global character
    state = WAITING;
}


// Task functions
void sensor_task(void *arg) {
    /*
    Detects movement and conerts it to dot or dash
        - detectMovement()
    */
   while(1) {
    detectMovement();
    state = DATA_READY;
    vTaskdelay(1000);
   }

}

void button_task(void *arg) {
    /*
    Prints the current character on the console
        - displayOutput()
        - button interrupt, 
    */
   while(1) {
    displayOutput(current_character);
    vTaskdelay(1000);

   }

}


// Main function for initializing everything
int main() {
    stdio_init_all();
    init_hat_sdk();
    sleep_ms(300);

    // Task Creation
    xTaskCreate(sensor_task, "Sensor Task", 256, NULL, 1, NULL);
    xTaskCreate(button_task, "Button Task", 256, NULL, 1, NULL);

    vTaskStartScheduler(); // Start FreeRTOS
    return 0;
}
