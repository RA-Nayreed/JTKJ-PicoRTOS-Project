/*
 * JTKJ Module2 Final Project - Tier 2
 * Authors: Rezwan Ahmad Nayreed, Joni Lahtinen and Aati Samuel Strömmer
 * * Description:
 * This firmware implements a Morse Code communicator using the JTKJ Hat.
 * Features:
 * - IMU-based input: Tilt the device to input Dots (.) and Dashes (-).
 * - Button input: Button 1 acts as 'Space' (end of character), Button 2 controls State.
 * - USB CDC: Sends translated text to PC and receives text from PC.
 * - Feedback: OLED display updates and Buzzer tones for user feedback.
 * - RTOS: Uses FreeRTOS to manage concurrent tasks (IMU, USB, Display, Audio).
*/

#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <tusb.h>
#include <string.h>
#include <math.h> 
#include "tkjhat/sdk.h"
#include "usbSerialDebug/helper.h"

// ==========================================
// --- Configuration & Constants ---
// ==========================================

// USB Configuration
#define CDC_ITF_TX 1  // The specific TinyUSB interface index for data transmission
#define QUEUE_LEN  32 // Max items in Morse/Event queues
#define MSG_BUFFER_SIZE 256 // Max length of a text message string

// Timing Constants (in Milliseconds)
#define DEBOUNCE_MS     50   // Filter out switch noise; ignore presses shorter than this
#define LONG_PRESS_MS   800  // Threshold to distinguish between "Toggle Mode" and "Send Message"
#define BTN1_DEBOUNCE   250  // Higher debounce for Button 1 to prevent accidental double-spaces

// ==========================================
// --- Data Structures & Enums ---
// ==========================================

// System State Machine
typedef enum {
    STATE_IDLE,       // Waiting for user input to start recording
    STATE_RECORDING,  // Actively reading IMU/Button inputs for Morse
    STATE_SENDING,    // Transmitting the finalized buffer via USB
    STATE_RECEIVING,  // Processing incoming USB data
    STATE_DISPLAYING  // Showing status/messages on the OLED
} SystemState;

// Message Buffer Wrapper
typedef struct {
    char buffer[MSG_BUFFER_SIZE];
    uint16_t length;
    bool complete;    // Flag to indicate message is ready to send
} MorseMessage;

// ==========================================
// --- Global Variables ---
// ==========================================

static volatile SystemState g_state = STATE_IDLE;
static MorseMessage g_tx_message = {0}; // Outgoing message buffer
static MorseMessage g_rx_message = {0}; // Incoming message buffer

// Button Timing Variables (volatile because modified in ISR)
static volatile uint32_t btn2_press_time = 0;
static volatile uint32_t btn1_last_press_time = 0;

// --- Morse Translation Maps ---
// Maps Morse patterns to their corresponding alphanumeric character
const char *morse_map[36] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-",
    ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-",
    ".--", "-..-", "-.--", "--..", ".----", "..---", "...--", "....-", ".....",
    "-....", "--...", "---..", "----.", "-----"
};
const char alphabet_map[36] = {
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
    'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
    'U', 'V', 'W', 'X', 'Y', 'Z', '1', '2', '3', '4',
    '5', '6', '7', '8', '9', '0'
};

// --- FreeRTOS Handles ---
static QueueHandle_t xMorseTxQueue;   // Events from IMU/Buttons to TX Task
static QueueHandle_t xMorseRxQueue;   // Events from USB to feedback systems
static QueueHandle_t xDisplayQueue;   // Strings to be printed on LCD
static QueueHandle_t xBuzzerQueue;    // Frequency/Duration commands for buzzer
static SemaphoreHandle_t xStateMutex; // Protects g_state variable
static SemaphoreHandle_t xTxMsgMutex; // Protects g_tx_message buffer

// --- Function Prototypes ---
static void imu_task(void *arg);
static void usb_tx_task(void *arg);
static void usb_rx_task(void *arg);
static void display_task(void *arg);
static void buzzer_task(void *arg);
static void state_manager_task(void *arg);
static void tinyusb_task(void *arg); 
static void gpio_callback(uint gpio, uint32_t events);

// ==========================================
// --- Helper Functions ---
// ==========================================

/**
 * Thread-safe state transition function.
 * Updates the global state and sends a status update to the LCD.
 */
static void change_state(SystemState new_state) {
    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_state = new_state;
        xSemaphoreGive(xStateMutex);
        
        // Update LCD with new state name
        char msg[32];
        switch(new_state) {
            case STATE_IDLE:       snprintf(msg, 32, "IDLE"); break;
            case STATE_RECORDING:  snprintf(msg, 32, "RECORDING"); break;
            case STATE_SENDING:    snprintf(msg, 32, "SENDING..."); break;
            case STATE_RECEIVING:  snprintf(msg, 32, "RECEIVING"); break;
            case STATE_DISPLAYING: snprintf(msg, 32, "DISPLAY"); break;
        }
        xQueueSend(xDisplayQueue, msg, 0);
    }
}

/**
 * Thread-safe append to the outgoing message buffer.
 */
static void append_to_tx_message(char c) {
    if (xSemaphoreTake(xTxMsgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (g_tx_message.length < MSG_BUFFER_SIZE - 1) {
            g_tx_message.buffer[g_tx_message.length++] = c;
            g_tx_message.buffer[g_tx_message.length] = '\0';
        }
        xSemaphoreGive(xTxMsgMutex);
    }
}

/**
 * Marks the current TX message as complete and ready to send.
 * Appends a newline character as a terminator.
 */
static void send_tx_message(void) {
    if (xSemaphoreTake(xTxMsgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (g_tx_message.length > 0) {
            if (g_tx_message.length < MSG_BUFFER_SIZE - 2) {
                g_tx_message.buffer[g_tx_message.length++] = '\n';
                g_tx_message.buffer[g_tx_message.length] = '\0';
                g_tx_message.complete = true;
            }
            else{
                g_tx_message.buffer[g_tx_message.length - 2] = '\n';
                g_tx_message.buffer[g_tx_message.length - 1] = '\0';
                g_tx_message.complete = true;
            }
        }
        else{
            g_tx_message.buffer[g_tx_message.length++] = '\n';
            g_tx_message.buffer[g_tx_message.length] = '\0';
            g_tx_message.complete = true;
        }
        xSemaphoreGive(xTxMsgMutex);
    }
}

/**
 * Resets the TX message buffer.
 */
static void clear_tx_message(void) {
    if (xSemaphoreTake(xTxMsgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(g_tx_message.buffer, 0, MSG_BUFFER_SIZE);
        g_tx_message.length = 0;
        g_tx_message.complete = false;
        xSemaphoreGive(xTxMsgMutex);
    }
}

/**
 * Queues a buzzer command. 
 * Packing freq and duration into a single uint32_t to save queue space.
 */
static void play_feedback_tone(uint16_t freq_hz, uint16_t duration_ms) {
    uint32_t cmd = (freq_hz << 16) | duration_ms;
    xQueueSend(xBuzzerQueue, &cmd, 0);
}

static void display_text(const char* text) {
    if (text && strlen(text) > 0) {
        xQueueSend(xDisplayQueue, text, 0);
    }
}

// ==========================================
// --- Interrupt Handler (ISR) ---
// ==========================================

/**
 * Handles GPIO interrupts for buttons.
 * NOTE: Keep logic minimal here. Use queues to offload work to tasks.
 */
static void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t hpw = pdFALSE; // High Priority Task Woken flag
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // --- Button 1: Space / Next Character ---
    if (gpio == BUTTON1 && (events & GPIO_IRQ_EDGE_RISE)) {
        // DEBOUNCE: Ignore if pressed too soon after last press
        if (now - btn1_last_press_time > BTN1_DEBOUNCE) {
            btn1_last_press_time = now;
            
            // Only process input if we are in recording mode
            if (g_state == STATE_RECORDING) {
                char space = ' ';
                xQueueSendFromISR(xMorseTxQueue, &space, &hpw);
            }
        }
    }
    // --- Button 2: State Toggle / Send ---
    else if (gpio == BUTTON2) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Record time when button was pressed down
            btn2_press_time = now;
        }
        else if (events & GPIO_IRQ_EDGE_RISE) {
            // Calculate how long the button was held
            uint32_t press_duration = now - btn2_press_time;
            
            if (press_duration >= LONG_PRESS_MS) {
                // LONG PRESS: User wants to send the message
                if (g_state == STATE_RECORDING) {
                    char end_msg = '\n';  
                    xQueueSendFromISR(xMorseTxQueue, &end_msg, &hpw);
                }
            } else if (press_duration >= DEBOUNCE_MS) {
                // SHORT PRESS: Toggle between IDLE and RECORDING
                char toggle = 'T';  
                xQueueSendFromISR(xMorseTxQueue, &toggle, &hpw);
            }
            // Context switch if a higher priority task was woken by the queue send
            portYIELD_FROM_ISR(hpw);
        }
    }
}

// ==========================================
// --- FreeRTOS Tasks ---
// ==========================================

/**
 * Task: IMU Reader
 * Reads accelerometer data and detects orientation for Morse input.
 * Z-Axis down = Flat = Dot (.)
 * Y-Axis down = Tilted Right = Dash (-)
 */
static void imu_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, t;
    
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for sensor power-up settling

    if (init_ICM42670() == 0) {
        ICM42670_start_with_default_values();
        usb_serial_print("IMU OK.\n");
    } else {
        usb_serial_print("IMU FAIL.\n");
        vTaskDelete(NULL); // Kill task if hardware fails
        return;
    }
    
    for (;;) {
        if (g_state == STATE_RECORDING) {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                
                // Logic: Check absolute gravity direction (1g ~ 1.0f)
                
                // DETECT DOT: Device is Flat (Z is approx -1g)
                if (az < -0.8f && fabsf(ax) < 0.3f && fabsf(ay) < 0.3f) {
                    char dot = '.';
                    xQueueSend(xMorseTxQueue, &dot, 0);
                    play_feedback_tone(1000, 100);  
                    // Delay prevents rapid-fire triggering; user must return to neutral
                    vTaskDelay(pdMS_TO_TICKS(800)); 
                }
                // DETECT DASH: Device tilted 90 deg (Y is approx -1g)
                else if (ay < -0.8f && fabsf(ax) < 0.3f) {
                    char dash = '-';
                    xQueueSend(xMorseTxQueue, &dash, 0);
                    play_feedback_tone(1000, 300);  
                    vTaskDelay(pdMS_TO_TICKS(800)); 
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));  // Sampling rate
    }
}

/**
 * Task: USB TX & Logic Coordinator
 * Receives raw inputs (buttons/IMU), translates Morse to text,
 * and handles sending data over USB.
 */
static void usb_tx_task(void *arg) {
    (void)arg;
    char symbol;
    
    char current_morse[8] = {0}; // Buffer for current character (e.g., "...-")
    uint8_t morse_len = 0;

    for (;;) {
        // Block until a symbol is received from ISR or IMU task
        if (xQueueReceive(xMorseTxQueue, &symbol, portMAX_DELAY) == pdTRUE) {
            
            // --- 1. Command Processing (State Control) ---
            if (symbol == 'T') { // Toggle Command
                if (g_state == STATE_IDLE) {
                    clear_tx_message();
                    change_state(STATE_RECORDING);
                    play_feedback_tone(2000, 100); // Start sound
                    
                    // Reset local Morse buffer
                    morse_len = 0;
                    memset(current_morse, 0, sizeof(current_morse));
                } else if (g_state == STATE_RECORDING) {
                    change_state(STATE_IDLE);
                    play_feedback_tone(1500, 100); // Stop sound
                }
                continue; // Skip further processing for this loop
            }
            else if (symbol == '\n') { // Send Message Command
                // Reset buffer and trigger send state
                morse_len = 0;
                memset(current_morse, 0, sizeof(current_morse));
                
                send_tx_message();
                change_state(STATE_SENDING);
                
            }
            
            // --- 2. Morse Character Building ---
            if (g_state == STATE_RECORDING) {
                
                if (symbol == '.' || symbol == '-') {
                    if (morse_len < 7) {
                        current_morse[morse_len++] = symbol;
                        current_morse[morse_len] = '\0';
                        
                        // Print current Morse sequence to USB debug
                        usb_serial_print(current_morse); 
                        usb_serial_print("\n");
                    }
                    else {
                        morse_len = 0;
                        memset(current_morse, 0, sizeof(current_morse));
                    }
                } 
                else if (symbol == ' ') {
                    // Space Button Pressed: End of current letter
                    if (morse_len == 0) {
                        append_to_tx_message(' ');
                        // Handle actual spaces between words
                    } 
                    else {
                        // --- Translation Logic ---
                        char translated_char = '?'; 
                        // Scan lookup table
                        for (int i = 0; i < 36; i++) {
                            if (strcmp(current_morse, morse_map[i]) == 0) {
                                translated_char = alphabet_map[i];
                                break;
                            }
                        }
                        append_to_tx_message(translated_char);
                        
                        // Debug Output
                        char debug_buf[10];
                        snprintf(debug_buf, 10, " -> %c\n", translated_char);
                        usb_serial_print(debug_buf);
                        
                        // Show the translated char on LCD
                        char lcd_buf[2] = {translated_char, '\0'};
                        display_text(lcd_buf);
                    }
                    // Reset Morse buffer for next character
                    morse_len = 0;
                    memset(current_morse, 0, sizeof(current_morse));
                }
            }
        }
        
        // --- 3. Sending Logic ---
        if (g_state == STATE_SENDING && g_tx_message.complete) {
            usb_serial_print("\n[SENDING MESSAGE]\n");
            if (tud_cdc_n_connected(CDC_ITF_TX)) {
                // Write buffer to USB CDC
                tud_cdc_n_write(CDC_ITF_TX, g_tx_message.buffer, g_tx_message.length);
                tud_cdc_n_write_flush(CDC_ITF_TX);
                
                play_feedback_tone(2500, 200); // Success tone
                usb_serial_print("\n[MSG SENT]\n");
            }
            clear_tx_message();
            change_state(STATE_IDLE);
        }
    }
}

/**
 * Task: USB RX
 * Polls USB for incoming data from PC.
 */
static void usb_rx_task(void *arg) {
    (void)arg;
    char rx_buf[64];
    for (;;) {
        if (tud_cdc_n_connected(CDC_ITF_TX) && tud_cdc_n_available(CDC_ITF_TX)) {
            uint32_t count = tud_cdc_n_read(CDC_ITF_TX, rx_buf, sizeof(rx_buf) - 1);
            
            if (count > 0) {
                rx_buf[count] = '\0'; // Null-terminate
                change_state(STATE_RECEIVING);
                
                for (uint32_t i = 0; i < count; i++) {
                    char c = rx_buf[i];
                    xQueueSend(xMorseRxQueue, &c, 0); // Trigger sound effect per char
                    
                    // Buffer incoming message
                    if (g_rx_message.length < MSG_BUFFER_SIZE - 1) {
                        g_rx_message.buffer[g_rx_message.length++] = c;
                        g_rx_message.buffer[g_rx_message.length] = '\0';
                    }
                    
                    // End of line detected: Show full message
                    if (c == '\n') {
                        change_state(STATE_DISPLAYING);
                        display_text(g_rx_message.buffer);
                        
                        usb_serial_print("\n[RX]: ");
                        usb_serial_print(g_rx_message.buffer);
                        
                        // Reset RX buffer
                        memset(g_rx_message.buffer, 0, MSG_BUFFER_SIZE);
                        g_rx_message.length = 0;
                        
                        vTaskDelay(pdMS_TO_TICKS(2000)); // Hold display
                        change_state(STATE_IDLE);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Polling interval
    }
}

/**
 * Task: Display
 * Controls the OLED screen based on messages received in queue.
 */
static void display_task(void *arg) {
    (void)arg;
    char msg_buf[64];
    init_display();
    clear_display();
    write_text("Ready!");
    
    for (;;) {
        // Block until a message arrives in queue
        if (xQueueReceive(xDisplayQueue, msg_buf, portMAX_DELAY) == pdTRUE) {
            clear_display();
            write_text(msg_buf);
        }
    }
}

/**
 * Task: Buzzer
 * Plays tones based on commands or raw characters.
 */
static void buzzer_task(void *arg) {
    (void)arg;
    uint32_t cmd;
    char morse_symbol;
    init_buzzer();
    
    for (;;) {
        // Priority 1: Explicit frequency commands
        if (xQueueReceive(xBuzzerQueue, &cmd, 0) == pdTRUE) {
            uint16_t freq = (cmd >> 16) & 0xFFFF; // Unpack freq
            uint16_t duration = cmd & 0xFFFF;     // Unpack duration
            buzzer_play_tone(freq, duration);
        }
        // Priority 2: Audio feedback for Morse playback
        else if (xQueueReceive(xMorseRxQueue, &morse_symbol, 0) == pdTRUE) {
            if (morse_symbol == '.') buzzer_play_tone(800, 150);
            else if (morse_symbol == '-') buzzer_play_tone(800, 450);
            else if (morse_symbol == ' ') vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * Task: TinyUSB Background
 * Required by TinyUSB stack to process USB events.
 */
static void tinyusb_task(void *arg) {
    (void)arg;
    for (;;) {
        tud_task();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * Task: State Manager
 * Periodic housekeeping (Watchdog style or debug printing).
 */
static void state_manager_task(void *arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(2000));
    usb_serial_print("\n=== System Ready ===\n");
    
    for (;;) {
        // Auto-revert from displaying state after timeout
        if (g_state == STATE_DISPLAYING) {
            vTaskDelay(pdMS_TO_TICKS(3000));
            if (g_state == STATE_DISPLAYING) change_state(STATE_IDLE);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ==========================================
// --- Main Entry Point ---
// ==========================================

int main(void) {
    // 1. Hardware Initialization
    init_hat_sdk();
    sleep_ms(300);
    init_button1();
    init_button2();
    
    // 2. OS Resource Creation
    xMorseTxQueue = xQueueCreate(QUEUE_LEN, sizeof(char));
    xMorseRxQueue = xQueueCreate(QUEUE_LEN, sizeof(char));
    xDisplayQueue = xQueueCreate(8, 64 * sizeof(char));
    xBuzzerQueue = xQueueCreate(8, sizeof(uint32_t));
    xStateMutex = xSemaphoreCreateMutex();
    xTxMsgMutex = xSemaphoreCreateMutex();
    
    // 3. Interrupt Configuration
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    
    // 4. Task Creation
    xTaskCreate(imu_task, "IMU", 2048, NULL, 2, NULL);
    xTaskCreate(usb_tx_task, "TX", 2048, NULL, 2, NULL);
    xTaskCreate(usb_rx_task, "RX", 2048, NULL, 2, NULL);
    xTaskCreate(display_task, "LCD", 2048, NULL, 1, NULL);
    xTaskCreate(buzzer_task, "Buz", 1024, NULL, 1, NULL);
    xTaskCreate(state_manager_task, "Mgr", 1024, NULL, 3, NULL);
    
    // Create USB task (often pinned to a specific core if multicore is enabled)
    TaskHandle_t hUsb;
    xTaskCreate(tinyusb_task, "USB", 2048, NULL, 3, &hUsb);
    #if (configNUMBER_OF_CORES > 1)
    vTaskCoreAffinitySet(hUsb, 1u << 0);
    #endif
    
    // 5. Start USB and Scheduler
    tusb_init();
    usb_serial_init();
    
    vTaskStartScheduler(); // Should never return
    return 0;
}
