/*
 * JTKJ Module2 Final Project - Tier 2
 * Authors: Rezwan Ahmad Nayreed, Joni Lahtinen and Aati Samuel Strömmer	
 * Date: 2025-11-17
 * Hardware: RP2040 + TKJ HAT (ICM42670, LCD, Buzzer)
 * Summary: IMU-based Morse transceiver with full-duplex communication
 * 
 * Changes from Tier 1:
 * - Fixed protocol compliance (proper spacing and message termination)
 * - Added state machine with 5 states
 * - Added RX task for receiving morse from workstation
 * - Added LCD display task
 * - Added buzzer feedback
 * - Added message buffer management
 * - Added mutex synchronization
 */

#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <tusb.h>
#include <string.h>
#include <math.h>  // Add this for fabs()
#include "tkjhat/sdk.h"
#include "usbSerialDebug/helper.h"

// Configuration
#define CDC_ITF_TX 0
#define QUEUE_LEN  32
#define MSG_BUFFER_SIZE 256
#define DEBOUNCE_MS 300
#define LONG_PRESS_MS 1000

// State Machine Definition
typedef enum {
    STATE_IDLE,          // Waiting for user input
    STATE_RECORDING,     // Capturing morse symbols
    STATE_SENDING,       // Transmitting message
    STATE_RECEIVING,     // Receiving from workstation
    STATE_DISPLAYING     // Showing received message
} SystemState;

// Message structure
typedef struct {
    char buffer[MSG_BUFFER_SIZE];
    uint16_t length;
    bool complete;
} MorseMessage;

// Global variables
static volatile SystemState g_state = STATE_IDLE;
static MorseMessage g_tx_message = {0};  // Message being composed
static MorseMessage g_rx_message = {0};  // Message being received

// FreeRTOS objects
static QueueHandle_t xMorseTxQueue;      // Symbols to transmit
static QueueHandle_t xMorseRxQueue;      // Symbols received
static QueueHandle_t xDisplayQueue;      // LCD display commands
static QueueHandle_t xBuzzerQueue;       // Buzzer commands
static SemaphoreHandle_t xStateMutex;    // State protection
static SemaphoreHandle_t xTxMsgMutex;    // TX message buffer protection

// Button timing
static volatile uint32_t button2_press_time = 0;

// Task prototypes
static void imu_task(void *arg);
static void usb_tx_task(void *arg);
static void usb_rx_task(void *arg);
static void display_task(void *arg);
static void buzzer_task(void *arg);
static void state_manager_task(void *arg);

// Helper prototypes
static void gpio_callback(uint gpio, uint32_t events);
static void change_state(SystemState new_state);
static void append_to_tx_message(char c);
static void send_tx_message(void);
static void clear_tx_message(void);
static void play_feedback_tone(uint16_t freq_hz, uint16_t duration_ms);
static void display_text(const char* text);

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static void change_state(SystemState new_state) {
    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_state = new_state;
        xSemaphoreGive(xStateMutex);
        
        // Visual feedback
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

static void append_to_tx_message(char c) {
    if (xSemaphoreTake(xTxMsgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (g_tx_message.length < MSG_BUFFER_SIZE - 1) {
            g_tx_message.buffer[g_tx_message.length++] = c;
            g_tx_message.buffer[g_tx_message.length] = '\0';
        }
        xSemaphoreGive(xTxMsgMutex);
    }
}

static void send_tx_message(void) {
    if (xSemaphoreTake(xTxMsgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (g_tx_message.length > 0) {
            // Ensure proper termination: two spaces + newline
            if (g_tx_message.length < MSG_BUFFER_SIZE - 3) {
                g_tx_message.buffer[g_tx_message.length++] = ' ';
                g_tx_message.buffer[g_tx_message.length++] = ' ';
                g_tx_message.buffer[g_tx_message.length++] = '\n';
                g_tx_message.buffer[g_tx_message.length] = '\0';
                g_tx_message.complete = true;
            }
        }
        xSemaphoreGive(xTxMsgMutex);
    }
}

static void clear_tx_message(void) {
    if (xSemaphoreTake(xTxMsgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(g_tx_message.buffer, 0, MSG_BUFFER_SIZE);
        g_tx_message.length = 0;
        g_tx_message.complete = false;
        xSemaphoreGive(xTxMsgMutex);
    }
}

static void play_feedback_tone(uint16_t freq_hz, uint16_t duration_ms) {
    uint32_t cmd = (freq_hz << 16) | duration_ms;
    xQueueSend(xBuzzerQueue, &cmd, 0);
}

static void display_text(const char* text) {
    if (text && strlen(text) > 0) {
        xQueueSend(xDisplayQueue, text, 0);
    }
}

// ============================================================================
// GPIO INTERRUPT HANDLER
// ============================================================================

static void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t hpw = pdFALSE;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    if (gpio == BUTTON1 && (events & GPIO_IRQ_EDGE_RISE)) {
        // BUTTON1: Add space between characters
        if (g_state == STATE_RECORDING) {
            char space = ' ';
            xQueueSendFromISR(xMorseTxQueue, &space, &hpw);
        }
        portYIELD_FROM_ISR(hpw);
    }
    else if (gpio == BUTTON2) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Button pressed - record time
            button2_press_time = now;
        }
        else if (events & GPIO_IRQ_EDGE_RISE) {
            // Button released - check duration
            uint32_t press_duration = now - button2_press_time;
            
            if (press_duration >= LONG_PRESS_MS) {
                // LONG PRESS: Send message (complete and transmit)
                if (g_state == STATE_RECORDING) {
                    // Signal to send message
                    char end_msg = '\n';  // Special signal
                    xQueueSendFromISR(xMorseTxQueue, &end_msg, &hpw);
                }
            } else if (press_duration >= DEBOUNCE_MS) {
                // SHORT PRESS: Toggle recording mode
                // State change handled in state manager task
                char toggle = 'T';  // Toggle signal
                xQueueSendFromISR(xMorseTxQueue, &toggle, &hpw);
            }
            portYIELD_FROM_ISR(hpw);
        }
    }
}

// ============================================================================
// TASKS
// ============================================================================

static void imu_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, t;
    
    if (init_ICM42670() == 0) {
        ICM42670_start_with_default_values();
        usb_serial_print("IMU Initialized!\n");
    } else {
        usb_serial_print("IMU FAILED!\n");
        vTaskDelete(NULL);
        return;
    }
    
    for (;;) {
        if (g_state == STATE_RECORDING) {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                
                // Position detection logic
                // Position 1: Device flat on table → DOT
                if (az < -0.8f && fabsf(ax) < 0.3f && fabsf(ay) < 0.3f) {  // Use fabsf for float
                    char dot = '.';
                    xQueueSend(xMorseTxQueue, &dot, 0);
                    play_feedback_tone(1000, 100);  // Short beep
                    vTaskDelay(pdMS_TO_TICKS(800));  // Prevent flooding
                }
                // Position 2: Device tilted 90° on side → DASH
                else if (ay < -0.8f && fabsf(ax) < 0.3f) {  // Use fabsf for float
                    char dash = '-';
                    xQueueSend(xMorseTxQueue, &dash, 0);
                    play_feedback_tone(1000, 300);  // Long beep
                    vTaskDelay(pdMS_TO_TICKS(800));  // Prevent flooding
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));  // Poll rate
    }
}

static void usb_tx_task(void *arg) {
    (void)arg;
    char symbol;
    
    for (;;) {
        if (xQueueReceive(xMorseTxQueue, &symbol, portMAX_DELAY) == pdTRUE) {
            
            // Handle special commands
            if (symbol == 'T') {
                // Toggle recording state
                if (g_state == STATE_IDLE) {
                    clear_tx_message();
                    change_state(STATE_RECORDING);
                    play_feedback_tone(2000, 100);
                } else if (g_state == STATE_RECORDING) {
                    change_state(STATE_IDLE);
                    play_feedback_tone(1500, 100);
                }
                continue;
            }
            else if (symbol == '\n') {
                // Send complete message
                if (g_state == STATE_RECORDING) {
                    send_tx_message();
                    change_state(STATE_SENDING);
                }
                continue;
            }
            
            // Append morse symbols to message buffer
            if (g_state == STATE_RECORDING) {
                append_to_tx_message(symbol);
                
                // Echo to debug
                char dbg[2] = {symbol, '\0'};
                usb_serial_print(dbg);
            }
        }
        
        // Check if message ready to send
        if (g_state == STATE_SENDING && g_tx_message.complete) {
            if (tud_cdc_n_connected(CDC_ITF_TX)) {
                tud_cdc_n_write(CDC_ITF_TX, g_tx_message.buffer, g_tx_message.length);
                tud_cdc_n_write_flush(CDC_ITF_TX);
                
                // Success feedback
                play_feedback_tone(2500, 200);
                display_text("SENT!");
                
                usb_serial_print("\n[MSG SENT]\n");
            }
            
            clear_tx_message();
            change_state(STATE_IDLE);
        }
    }
}

static void usb_rx_task(void *arg) {
    (void)arg;
    char rx_buf[64];
    
    for (;;) {
        if (tud_cdc_n_connected(CDC_ITF_TX) && tud_cdc_n_available(CDC_ITF_TX)) {
            uint32_t count = tud_cdc_n_read(CDC_ITF_TX, rx_buf, sizeof(rx_buf) - 1);
            
            if (count > 0) {
                rx_buf[count] = '\0';
                
                // Process received symbols
                change_state(STATE_RECEIVING);
                
                for (uint32_t i = 0; i < count; i++) {
                    char c = rx_buf[i];
                    
                    // Add to RX queue for buzzer/display
                    xQueueSend(xMorseRxQueue, &c, 0);
                    
                    // Build received message
                    if (g_rx_message.length < MSG_BUFFER_SIZE - 1) {
                        g_rx_message.buffer[g_rx_message.length++] = c;
                        g_rx_message.buffer[g_rx_message.length] = '\0';
                    }
                    
                    // Check for message end
                    if (c == '\n') {
                        change_state(STATE_DISPLAYING);
                        display_text(g_rx_message.buffer);
                        
                        usb_serial_print("\n[RX]: ");
                        usb_serial_print(g_rx_message.buffer);
                        
                        // Clear RX buffer
                        memset(g_rx_message.buffer, 0, MSG_BUFFER_SIZE);
                        g_rx_message.length = 0;
                        
                        vTaskDelay(pdMS_TO_TICKS(2000));  // Display duration
                        change_state(STATE_IDLE);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // RX poll rate
    }
}

static void display_task(void *arg) {
    (void)arg;
    char msg_buf[64];
    
    // Initialize display using correct SDK function
    init_display();
    clear_display();
    write_text("Ready!");
    
    for (;;) {
        if (xQueueReceive(xDisplayQueue, msg_buf, portMAX_DELAY) == pdTRUE) {
            clear_display();
            write_text(msg_buf);
        }
    }
}

static void buzzer_task(void *arg) {
    (void)arg;
    uint32_t cmd;
    char morse_symbol;
    
    init_buzzer();
    
    for (;;) {
        // Priority 1: Direct buzzer commands (from feedback)
        if (xQueueReceive(xBuzzerQueue, &cmd, 0) == pdTRUE) {
            uint16_t freq = (cmd >> 16) & 0xFFFF;
            uint16_t duration = cmd & 0xFFFF;
            
            // Use SDK function buzzer_play_tone()
            buzzer_play_tone(freq, duration);
        }
        // Priority 2: Morse symbol playback (from RX)
        else if (xQueueReceive(xMorseRxQueue, &morse_symbol, 0) == pdTRUE) {
            if (morse_symbol == '.') {
                // Short beep for dot
                buzzer_play_tone(800, 150);
            }
            else if (morse_symbol == '-') {
                // Long beep for dash
                buzzer_play_tone(800, 450);
            }
            else if (morse_symbol == ' ') {
                // Silence for space
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void state_manager_task(void *arg) {
    (void)arg;
    
    for (;;) {
        // Monitor system state and handle timeouts
        if (g_state == STATE_DISPLAYING) {
            // Auto-return to idle after display timeout
            vTaskDelay(pdMS_TO_TICKS(3000));
            if (g_state == STATE_DISPLAYING) {
                change_state(STATE_IDLE);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Add wrapper function for TinyUSB task
static void tinyusb_task(void *arg) {
    (void)arg;
    for (;;) {
        tud_task();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main(void) {
    // Hardware initialization
    init_hat_sdk();
    sleep_ms(300);
    
    init_button1();
    init_button2();
    
    // GPIO interrupts with callback
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    
    // Create synchronization objects
    xMorseTxQueue = xQueueCreate(QUEUE_LEN, sizeof(char));
    xMorseRxQueue = xQueueCreate(QUEUE_LEN, sizeof(char));
    xDisplayQueue = xQueueCreate(8, 64 * sizeof(char));
    xBuzzerQueue = xQueueCreate(8, sizeof(uint32_t));
    
    xStateMutex = xSemaphoreCreateMutex();
    xTxMsgMutex = xSemaphoreCreateMutex();
    
    // Create tasks
    xTaskCreate(imu_task,          "IMU",        2048, NULL, 2, NULL);
    xTaskCreate(usb_tx_task,       "USB_TX",     2048, NULL, 2, NULL);
    xTaskCreate(usb_rx_task,       "USB_RX",     2048, NULL, 2, NULL);
    xTaskCreate(display_task,      "Display",    2048, NULL, 1, NULL);
    xTaskCreate(buzzer_task,       "Buzzer",     1024, NULL, 1, NULL);
    xTaskCreate(state_manager_task,"StateMgr",   1024, NULL, 3, NULL);
    
    // USB task on core 0 - Use wrapper function
    TaskHandle_t hUsb;
    xTaskCreate(tinyusb_task, "TinyUSB", 2048, NULL, 3, &hUsb);
#if (configNUMBER_OF_CORES > 1)
    vTaskCoreAffinitySet(hUsb, 1u << 0);
#endif
    
    // Initialize USB
    tusb_init();
    usb_serial_init();
    
    usb_serial_print("\n=== Morse Transceiver Ready ===\n");
    usb_serial_print("BUTTON2 short: Toggle recording\n");
    usb_serial_print("BUTTON2 long:  Send message\n");
    usb_serial_print("BUTTON1:       Character space\n");
    
    vTaskStartScheduler();
    return 0;
}