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
void displayOutput(char current_str[]);
void detectMovement();



// I'm guessing we should make a statemachine task that uses switch statements
// to determine which state we're in?




void detectMovement() {
    /*
    Detect movement from the device and save the corresponding character to current_character.
    */
   // blah blah blah
}


void button_interrupt() {
    /*
    
    */
}


// Terminal communication
void displayOutput(char current_char) {
    /*
    Displays the current morse code string and resets it.
    */
    printf("%s\n", current_char);
    current_char = NULL; // Reset string after displaying
    state = WAITING;
}


// Task functions
void sensor_task() {
    /*
    Detects movement and conerts it to dot or dash
        - detectMovement()
    */

}

void button_task() {
    /*
    Prints the current character on the console
        - displayOutput()
        - button interrupt, 
    */
   while(1) {
    displayOutput();
    vTaskdelay(1000);

   }

}


// Main function for initializing everything
int main() {
    stdio_init_all();
    init_hat_sdk();
    sleep_ms(300);

    switch(state) {
        case WAITING:
            break;
        case DISPLAYING:
            break;
        case DATA_READY:
            break;
        case RECORDING:
            break;
    }
    vTaskStartScheduler(); // Start FreeRTOS
    return 0;
}
