#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <gui/view_dispatcher.h>
#include <gui/scenes/generic_scene.h>
#include <gui/view_stack.h>
#include <gui/modules/submenu.h>
#include <gui/modules/text_input.h>
#include <gui/modules/popup.h>
#include <gui/modules/menu.h>
#include <notification/notification_messages.h>

#include <furi_hal_uart.h>
#include <furi_hal_gpio.h>

#define FLIPPFLOP_APP_NAME "Flippflop"
#define TAG "Flippflop"

// UART Configuration
#define UART_CHANNEL FuriHalUartChannelUSART1
#define UART_BAUD 115200

typedef enum {
    SceneMainMenu,
    SceneCC1101,
    SceneCC1101Status,
    SceneCC1101Frequency,
    SceneCC1101Power,
    SceneCC1101Sweep,
    SceneSI5351,
    SceneSI5351Frequency,
    SceneSI5351FreqConv,
    SceneSI5351LO,
    SceneSI5351Impair,
    SceneAntenna,
    SceneAbout,
} SceneEnum;

typedef struct {
    ViewDispatcher* view_dispatcher;
    Gui* gui;
    Submenu* submenu;
    TextInput* text_input;
    Popup* popup;
    
    FuriThread* uart_thread;
    FuriStreamBuffer* uart_rx_buffer;
    
    // State
    char frequency_input[16];
    char power_input[3];
    char rf_frequency[16];
    char lo_frequency[16];
    char noise_level[3];
    
    // Status variables
    float current_freq;
    int8_t current_rssi;
    uint8_t current_power;
    uint8_t current_antenna;
    bool cc1101_present;
    bool si5351_present;
    char status_text[256];
} FlippflopApp;

// ============================================================================
// UART RX HANDLING
// ============================================================================

static int32_t flippflop_uart_worker(void* context) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    while (true) {
        size_t available = furi_stream_buffer_spaces_available(app->uart_rx_buffer);
        if (available > 0) {
            uint8_t data[64];
            size_t read = furi_hal_uart_receive(UART_CHANNEL, data, sizeof(data), 100);
            
            if (read > 0) {
                furi_stream_buffer_send(app->uart_rx_buffer, data, read);
            }
        }
        furi_delay_ms(10);
    }
    
    return 0;
}

static void flippflop_uart_init(FlippflopApp* app) {
    furi_hal_uart_set_br(UART_CHANNEL, UART_BAUD);
    app->uart_rx_buffer = furi_stream_buffer_alloc(256, 1);
    app->uart_thread = furi_thread_alloc();
    furi_thread_set_name(app->uart_thread, "FlippflopUART");
    furi_thread_set_stack_size(app->uart_thread, 1024);
    furi_thread_set_callback(app->uart_thread, flippflop_uart_worker);
    furi_thread_set_context(app->uart_thread, app);
    furi_thread_start(app->uart_thread);
}

static void flippflop_uart_send_command(FlippflopApp* app, const char* command) {
    furi_hal_uart_tx(UART_CHANNEL, (const uint8_t*)command, strlen(command));
    furi_hal_uart_tx(UART_CHANNEL, (const uint8_t*)"\n", 1);
}

// ============================================================================
// MAIN MENU
// ============================================================================

static void flippflop_submenu_callback(void* context, uint32_t index) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    switch (index) {
        case 0: // CC1101 Control
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneCC1101);
            break;
        case 1: // SI5351 Control
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneSI5351);
            break;
        case 2: // Antenna
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneAntenna);
            break;
        case 3: // Status
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneCC1101Status);
            break;
        case 4: // About
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneAbout);
            break;
    }
}

// ============================================================================
// CC1101 MENU
// ============================================================================

static void flippflop_cc1101_menu_callback(void* context, uint32_t index) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    switch (index) {
        case 0: // Set Frequency
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneCC1101Frequency);
            break;
        case 1: // Set Power
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneCC1101Power);
            break;
        case 2: // Send Ping
            flippflop_uart_send_command(app, "SEND");
            snprintf(app->status_text, sizeof(app->status_text), "Ping sent!");
            break;
        case 3: // Read RSSI
            flippflop_uart_send_command(app, "RSSI");
            snprintf(app->status_text, sizeof(app->status_text), "Reading RSSI...");
            break;
        case 4: // Sweep
            flippflop_uart_send_command(app, "SWEEP");
            snprintf(app->status_text, sizeof(app->status_text), "Sweeping...");
            break;
        case 5: // Carrier ON
            flippflop_uart_send_command(app, "CARRIER:ON");
            snprintf(app->status_text, sizeof(app->status_text), "Carrier ON");
            break;
        case 6: // Carrier OFF
            flippflop_uart_send_command(app, "CARRIER:OFF");
            snprintf(app->status_text, sizeof(app->status_text), "Carrier OFF");
            break;
    }
}

// ============================================================================
// SI5351 MENU
// ============================================================================

static void flippflop_si5351_menu_callback(void* context, uint32_t index) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    switch (index) {
        case 0: // Set Frequency
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneSI5351Frequency);
            break;
        case 1: // Freq Conversion
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneSI5351FreqConv);
            break;
        case 2: // LO Substitution
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneSI5351LO);
            break;
        case 3: // Impairment
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneSI5351Impair);
            break;
        case 4: // Output ON
            flippflop_uart_send_command(app, "SI5351ON");
            snprintf(app->status_text, sizeof(app->status_text), "SI5351 Output ON");
            break;
        case 5: // Output OFF
            flippflop_uart_send_command(app, "SI5351OFF");
            snprintf(app->status_text, sizeof(app->status_text), "SI5351 Output OFF");
            break;
    }
}

// ============================================================================
// TEXT INPUT CALLBACKS
// ============================================================================

static void flippflop_frequency_input_callback(void* context) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    if (strlen(app->frequency_input) > 0) {
        char command[64];
        snprintf(command, sizeof(command), "TUNE:%s", app->frequency_input);
        flippflop_uart_send_command(app, command);
        snprintf(app->status_text, sizeof(app->status_text), "Set to %.2f MHz", atof(app->frequency_input));
    }
}

static void flippflop_power_input_callback(void* context) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    if (strlen(app->power_input) > 0) {
        char command[64];
        snprintf(command, sizeof(command), "POWER:%s", app->power_input);
        flippflop_uart_send_command(app, command);
        snprintf(app->status_text, sizeof(app->status_text), "Power set to %d", atoi(app->power_input));
    }
}

static void flippflop_si5351_freq_input_callback(void* context) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    if (strlen(app->frequency_input) > 0) {
        char command[64];
        snprintf(command, sizeof(command), "SI5351FREQ:%s", app->frequency_input);
        flippflop_uart_send_command(app, command);
        snprintf(app->status_text, sizeof(app->status_text), "SI5351 set to %s Hz", app->frequency_input);
    }
}

static void flippflop_freqconv_callback(void* context) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    if (strlen(app->rf_frequency) > 0 && strlen(app->lo_frequency) > 0) {
        char command[128];
        snprintf(command, sizeof(command), "FREQCONV:%s:%s", app->rf_frequency, app->lo_frequency);
        flippflop_uart_send_command(app, command);
        snprintf(app->status_text, sizeof(app->status_text), "RF:%s LO:%s", app->rf_frequency, app->lo_frequency);
    }
}

// ============================================================================
// ANTENNA MENU
// ============================================================================

static void flippflop_antenna_menu_callback(void* context, uint32_t index) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    if (index < 4) {
        char command[32];
        snprintf(command, sizeof(command), "ANTENNA:%d", index);
        flippflop_uart_send_command(app, command);
        
        const char* names[] = {"Internal", "External-1", "External-2", "Dipole"};
        snprintf(app->status_text, sizeof(app->status_text), "Antenna: %s", names[index]);
    }
}

// ============================================================================
// STATUS SCREEN DRAW
// ============================================================================

static void flippflop_status_draw_callback(Canvas* canvas, void* context) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    canvas_clear(canvas);
    
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 0, 10, "Flippflop Status");
    
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 128, 20, AlignRight, AlignTop, app->status_text);
    
    // CC1101 Status
    canvas_draw_str(canvas, 0, 35, "CC1101:");
    canvas_draw_str(canvas, 50, 35, app->cc1101_present ? "OK" : "NOT FOUND");
    
    canvas_draw_str(canvas, 0, 45, "SI5351:");
    canvas_draw_str(canvas, 50, 45, app->si5351_present ? "OK" : "NOT FOUND");
    
    char freq_str[32];
    snprintf(freq_str, sizeof(freq_str), "Freq: %.2f MHz", app->current_freq);
    canvas_draw_str(canvas, 0, 55, freq_str);
    
    char rssi_str[32];
    snprintf(rssi_str, sizeof(rssi_str), "RSSI: %d dBm", app->current_rssi);
    canvas_draw_str(canvas, 0, 65, rssi_str);
}

// ============================================================================
// MAIN VIEW DRAW
// ============================================================================

static void flippflop_main_draw_callback(Canvas* canvas, void* context) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    canvas_clear(canvas);
    
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 5, AlignCenter, AlignTop, "Flippflop");
    canvas_draw_str_aligned(canvas, 64, 20, AlignCenter, AlignTop, "RF Test Tool");
    
    canvas_set_font(canvas, FontSecondary);
    
    // Version
    canvas_draw_str_aligned(canvas, 64, 40, AlignCenter, AlignTop, "v2.0 - Advanced RF");
    
    // Features
    canvas_draw_str_aligned(canvas, 64, 55, AlignCenter, AlignTop, "CC1101 + SI5351");
    canvas_draw_str_aligned(canvas, 64, 65, AlignCenter, AlignTop, "Press OK to continue");
}

// ============================================================================
// SCENE MANAGEMENT
// ============================================================================

static bool flippflop_custom_event_callback(void* context, uint32_t event) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    switch (event) {
        case SceneCC1101:
            // Build CC1101 menu
            submenu_reset(app->submenu);
            submenu_add_item(app->submenu, "Set Frequency", 0, flippflop_cc1101_menu_callback, app);
            submenu_add_item(app->submenu, "Set Power (0-7)", 1, flippflop_cc1101_menu_callback, app);
            submenu_add_item(app->submenu, "Send Ping", 2, flippflop_cc1101_menu_callback, app);
            submenu_add_item(app->submenu, "Read RSSI", 3, flippflop_cc1101_menu_callback, app);
            submenu_add_item(app->submenu, "Sweep Band", 4, flippflop_cc1101_menu_callback, app);
            submenu_add_item(app->submenu, "Carrier ON", 5, flippflop_cc1101_menu_callback, app);
            submenu_add_item(app->submenu, "Carrier OFF", 6, flippflop_cc1101_menu_callback, app);
            view_dispatcher_switch_to_view(app->view_dispatcher, 1);
            break;
            
        case SceneSI5351:
            // Build SI5351 menu
            submenu_reset(app->submenu);
            submenu_add_item(app->submenu, "Set Frequency", 0, flippflop_si5351_menu_callback, app);
            submenu_add_item(app->submenu, "Frequency Conversion", 1, flippflop_si5351_menu_callback, app);
            submenu_add_item(app->submenu, "LO Substitution", 2, flippflop_si5351_menu_callback, app);
            submenu_add_item(app->submenu, "Impairment Setup", 3, flippflop_si5351_menu_callback, app);
            submenu_add_item(app->submenu, "Output ON", 4, flippflop_si5351_menu_callback, app);
            submenu_add_item(app->submenu, "Output OFF", 5, flippflop_si5351_menu_callback, app);
            view_dispatcher_switch_to_view(app->view_dispatcher, 1);
            break;
            
        case SceneAntenna:
            // Build Antenna menu
            submenu_reset(app->submenu);
            submenu_add_item(app->submenu, "Internal", 0, flippflop_antenna_menu_callback, app);
            submenu_add_item(app->submenu, "External-1", 1, flippflop_antenna_menu_callback, app);
            submenu_add_item(app->submenu, "External-2", 2, flippflop_antenna_menu_callback, app);
            submenu_add_item(app->submenu, "Dipole", 3, flippflop_antenna_menu_callback, app);
            view_dispatcher_switch_to_view(app->view_dispatcher, 1);
            break;
            
        case SceneCC1101Frequency:
            // Frequency input screen
            text_input_reset(app->text_input);
            text_input_set_header_text(app->text_input, "Set Frequency (MHz)");
            text_input_set_result_callback(app->text_input, flippflop_frequency_input_callback, app, app->frequency_input, sizeof(app->frequency_input), true);
            view_dispatcher_switch_to_view(app->view_dispatcher, 2);
            break;
            
        case SceneCC1101Power:
            // Power input screen
            text_input_reset(app->text_input);
            text_input_set_header_text(app->text_input, "Set Power (0-7)");
            text_input_set_result_callback(app->text_input, flippflop_power_input_callback, app, app->power_input, sizeof(app->power_input), true);
            view_dispatcher_switch_to_view(app->view_dispatcher, 2);
            break;
            
        case SceneSI5351Frequency:
            // SI5351 frequency input
            text_input_reset(app->text_input);
            text_input_set_header_text(app->text_input, "Set Freq (Hz)");
            text_input_set_result_callback(app->text_input, flippflop_si5351_freq_input_callback, app, app->frequency_input, sizeof(app->frequency_input), true);
            view_dispatcher_switch_to_view(app->view_dispatcher, 2);
            break;
    }
    
    return true;
}

// ============================================================================
// APP INITIALIZATION
// ============================================================================

static FlippflopApp* flippflop_app_alloc(void) {
    FlippflopApp* app = malloc(sizeof(FlippflopApp));
    
    app->gui = furi_record_open(RECORD_GUI);
    app->view_dispatcher = view_dispatcher_alloc();
    
    app->submenu = submenu_alloc();
    app->text_input = text_input_alloc();
    
    view_dispatcher_add_view(app->view_dispatcher, 1, submenu_get_view(app->submenu));
    view_dispatcher_add_view(app->view_dispatcher, 2, text_input_get_view(app->text_input));
    
    view_dispatcher_attach_to_gui(app->view_dispatcher, app->gui, ViewDispatcherTypeFullscreen);
    view_dispatcher_set_event_callback_context(app->view_dispatcher, app);
    view_dispatcher_set_custom_event_callback(app->view_dispatcher, flippflop_custom_event_callback);
    
    // Initialize main menu
    submenu_add_item(app->submenu, "CC1101 Control", 0, flippflop_submenu_callback, app);
    submenu_add_item(app->submenu, "SI5351 (8kHz-160MHz)", 1, flippflop_submenu_callback, app);
    submenu_add_item(app->submenu, "Antenna Select", 2, flippflop_submenu_callback, app);
    submenu_add_item(app->submenu, "Status", 3, flippflop_submenu_callback, app);
    submenu_add_item(app->submenu, "About", 4, flippflop_submenu_callback, app);
    
    // Initialize UART
    flippflop_uart_init(app);
    
    // Initialize state
    app->current_freq = 433.92f;
    app->current_rssi = -128;
    app->current_power = 7;
    app->current_antenna = 0;
    app->cc1101_present = false;
    app->si5351_present = false;
    
    memset(app->status_text, 0, sizeof(app->status_text));
    snprintf(app->status_text, sizeof(app->status_text), "Ready");
    
    return app;
}

static void flippflop_app_free(FlippflopApp* app) {
    if (app->uart_thread) {
        furi_thread_join(app->uart_thread);
        furi_thread_free(app->uart_thread);
    }
    
    if (app->uart_rx_buffer) {
        furi_stream_buffer_free(app->uart_rx_buffer);
    }
    
    view_dispatcher_remove_view(app->view_dispatcher, 1);
    view_dispatcher_remove_view(app->view_dispatcher, 2);
    
    submenu_free(app->submenu);
    text_input_free(app->text_input);
    view_dispatcher_free(app->view_dispatcher);
    
    furi_record_close(RECORD_GUI);
    
    free(app);
}

// ============================================================================
// MAIN APP ENTRY
// ============================================================================

int32_t flippflop_app(void* p) {
    UNUSED(p);
    
    FlippflopApp* app = flippflop_app_alloc();
    
    // Start with main menu
    view_dispatcher_switch_to_view(app->view_dispatcher, 1);
    
    // Main loop
    view_dispatcher_run(app->view_dispatcher);
    
    flippflop_app_free(app);
    
    return 0;
}
