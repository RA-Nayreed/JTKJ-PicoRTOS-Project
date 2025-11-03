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
    DATA_READY
} programState;

char current_character;
char current_str[]; // string of all the characters received so far
programState state = WAITING;

// I'm guessing we should make a statemachine task that uses switch statements
// to determine which state we're in?




// Terminal communication
void displayOutput(char current_str[]) {
    /*
    Displays the current morse code string and resets it.
    */
    printf("%s\n", current_str);
    current_str[0] = "\0"; // Reset string after displaying
    state = WAITING;
}
