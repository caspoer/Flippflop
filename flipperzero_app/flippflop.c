#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <gui/view.h>
#include <gui/view_dispatcher.h>
#include <gui/view_stack.h>
#include <gui/modules/submenu.h>
#include <gui/modules/text_input.h>
#include <gui/modules/popup.h>
#include <gui/modules/menu.h>
#include <notification/notification_messages.h>

#include <furi_hal_serial.h>
#include <furi_hal_serial_control.h>

#define FLIPPFLOP_APP_NAME "Flippflop"
#define TAG "Flippflop"

// UART Configuration
#define UART_CHANNEL FuriHalSerialIdUsart
#define UART_BAUD 115200

typedef enum {
    SceneMainMenu,
    SceneCC1101,
    SceneCC1101Status,
    SceneCC1101Frequency,
    SceneCC1101Power,
    SceneCC1101Sweep,
    SceneADF4351,
    SceneADF4351Frequency,
    SceneADF4351Power,
    SceneAntenna,
    SceneAbout,
} SceneEnum;

typedef struct {
    ViewDispatcher* view_dispatcher;
    Gui* gui;
    Submenu* submenu;
    TextInput* text_input;
    Popup* popup;
    View* status_view;
    View* about_view;
    SceneEnum current_scene;

    FuriHalSerialHandle* uart_handle;
    FuriStreamBuffer* uart_rx_buffer;

    // State
    char frequency_input[16];
    char power_input[3];

    // Status variables
    float current_freq;
    int8_t current_rssi;
    uint8_t current_power;
    uint8_t current_antenna;
    bool cc1101_present;
    bool adf4351_present;
    char status_text[256];
} FlippflopApp;

// ============================================================================
// UART RX HANDLING
// ============================================================================

static void flippflop_uart_rx_callback(
    FuriHalSerialHandle* handle,
    FuriHalSerialRxEvent event,
    void* context) {
    FlippflopApp* app = (FlippflopApp*)context;

    if (event & FuriHalSerialRxEventData) {
        while (furi_hal_serial_async_rx_available(handle)) {
            uint8_t data = furi_hal_serial_async_rx(handle);
            furi_stream_buffer_send(app->uart_rx_buffer, &data, 1, 0);
        }
    }
}

static void flippflop_uart_init(FlippflopApp* app) {
    app->uart_rx_buffer = furi_stream_buffer_alloc(256, 1);
    app->uart_handle = furi_hal_serial_control_acquire(UART_CHANNEL);
    furi_hal_serial_init(app->uart_handle, UART_BAUD);
    furi_hal_serial_async_rx_start(app->uart_handle, flippflop_uart_rx_callback, app, false);
}

static void flippflop_uart_send_command(FlippflopApp* app, const char* command) {
    furi_hal_serial_tx(app->uart_handle, (const uint8_t*)command, strlen(command));
    furi_hal_serial_tx(app->uart_handle, (const uint8_t*)"\n", 1);
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
        case 1: // ADF4351 Control
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneADF4351);
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
// ADF4351 MENU
// ============================================================================

static void flippflop_adf4351_menu_callback(void* context, uint32_t index) {
    FlippflopApp* app = (FlippflopApp*)context;

    switch (index) {
        case 0: // Set Frequency
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneADF4351Frequency);
            break;
        case 1: // Set Power
            view_dispatcher_send_custom_event(app->view_dispatcher, SceneADF4351Power);
            break;
        case 2: // RF Output ON
            flippflop_uart_send_command(app, "RFOUT:ON");
            snprintf(app->status_text, sizeof(app->status_text), "RF Output ON");
            break;
        case 3: // RF Output OFF
            flippflop_uart_send_command(app, "RFOUT:OFF");
            snprintf(app->status_text, sizeof(app->status_text), "RF Output OFF");
            break;
        case 4: // Sweep
            flippflop_uart_send_command(app, "SWEEP");
            snprintf(app->status_text, sizeof(app->status_text), "Sweeping...");
            break;
        case 5: // Lock Status
            flippflop_uart_send_command(app, "LOCK");
            snprintf(app->status_text, sizeof(app->status_text), "Reading lock status...");
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
        snprintf(app->status_text, sizeof(app->status_text), "Set to %.2f MHz", (double)strtof(app->frequency_input, NULL));
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

static void flippflop_adf4351_freq_input_callback(void* context) {
    FlippflopApp* app = (FlippflopApp*)context;

    if (strlen(app->frequency_input) > 0) {
        char command[64];
        snprintf(command, sizeof(command), "TUNE:%s", app->frequency_input);
        flippflop_uart_send_command(app, command);
        snprintf(app->status_text, sizeof(app->status_text), "Set to %.2f MHz", (double)strtof(app->frequency_input, NULL));
    }
}

static void flippflop_adf4351_power_input_callback(void* context) {
    FlippflopApp* app = (FlippflopApp*)context;

    if (strlen(app->power_input) > 0) {
        char command[64];
        snprintf(command, sizeof(command), "POWER:%s", app->power_input);
        flippflop_uart_send_command(app, command);
        snprintf(app->status_text, sizeof(app->status_text), "Power idx set to %d", atoi(app->power_input));
    }
}

// ============================================================================
// ANTENNA MENU
// ============================================================================

static void flippflop_antenna_menu_callback(void* context, uint32_t index) {
    FlippflopApp* app = (FlippflopApp*)context;
    
    if (index < 4) {
        char command[32];
        snprintf(command, sizeof(command), "ANTENNA:%d", (int)index);
        flippflop_uart_send_command(app, command);
        
        const char* names[] = {"Internal", "External-1", "External-2", "Dipole"};
        snprintf(app->status_text, sizeof(app->status_text), "Antenna: %s", names[index]);
    }
}

// ============================================================================
// STATUS SCREEN DRAW
// ============================================================================

static void flippflop_status_draw_callback(Canvas* canvas, void* model) {
    FlippflopApp* app = *(FlippflopApp**)model;

    canvas_clear(canvas);
    
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 0, 10, "Flippflop Status");
    
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 128, 20, AlignRight, AlignTop, app->status_text);
    
    // CC1101 Status
    canvas_draw_str(canvas, 0, 35, "CC1101:");
    canvas_draw_str(canvas, 50, 35, app->cc1101_present ? "OK" : "NOT FOUND");
    
    canvas_draw_str(canvas, 0, 45, "ADF4351:");
    canvas_draw_str(canvas, 50, 45, app->adf4351_present ? "OK" : "NOT FOUND");
    
    char freq_str[32];
    snprintf(freq_str, sizeof(freq_str), "Freq:%.1f Scene:%d", (double)app->current_freq, (int)app->current_scene);
    canvas_draw_str(canvas, 0, 55, freq_str);

    char rssi_str[32];
    snprintf(rssi_str, sizeof(rssi_str), "RSSI: %d dBm", app->current_rssi);
    canvas_draw_str(canvas, 0, 63, rssi_str);
}

// ============================================================================
// MAIN VIEW DRAW
// ============================================================================

static void flippflop_main_draw_callback(Canvas* canvas, void* model) {
    UNUSED(model);

    canvas_clear(canvas);
    
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 5, AlignCenter, AlignTop, "Flippflop");
    canvas_draw_str_aligned(canvas, 64, 20, AlignCenter, AlignTop, "RF Test Tool");
    
    canvas_set_font(canvas, FontSecondary);
    
    // Version
    canvas_draw_str_aligned(canvas, 64, 40, AlignCenter, AlignTop, "v2.0 - Advanced RF");
    
    // Features
    canvas_draw_str_aligned(canvas, 64, 55, AlignCenter, AlignTop, "CC1101 + ADF4351");
    canvas_draw_str_aligned(canvas, 64, 65, AlignCenter, AlignTop, "Press OK to continue");
}

// ============================================================================
// MENU BUILDERS
// ============================================================================
// Each of these (re)builds the shared submenu (view 1) for one logical scene
// and records it in app->current_scene, so both forward navigation (menu
// selection, below) and backward navigation (flippflop_navigation_event_callback)
// can rebuild the correct menu from a single place.

static void flippflop_show_main_menu(FlippflopApp* app) {
    submenu_reset(app->submenu);
    submenu_add_item(app->submenu, "CC1101 Control", 0, flippflop_submenu_callback, app);
    submenu_add_item(app->submenu, "ADF4351 (35-4400MHz)", 1, flippflop_submenu_callback, app);
    submenu_add_item(app->submenu, "Antenna Select", 2, flippflop_submenu_callback, app);
    submenu_add_item(app->submenu, "Status", 3, flippflop_submenu_callback, app);
    submenu_add_item(app->submenu, "About", 4, flippflop_submenu_callback, app);
    app->current_scene = SceneMainMenu;
    view_dispatcher_switch_to_view(app->view_dispatcher, 1);
}

static void flippflop_show_cc1101_menu(FlippflopApp* app) {
    submenu_reset(app->submenu);
    submenu_add_item(app->submenu, "Set Frequency", 0, flippflop_cc1101_menu_callback, app);
    submenu_add_item(app->submenu, "Set Power (0-7)", 1, flippflop_cc1101_menu_callback, app);
    submenu_add_item(app->submenu, "Send Ping", 2, flippflop_cc1101_menu_callback, app);
    submenu_add_item(app->submenu, "Read RSSI", 3, flippflop_cc1101_menu_callback, app);
    submenu_add_item(app->submenu, "Sweep Band", 4, flippflop_cc1101_menu_callback, app);
    submenu_add_item(app->submenu, "Carrier ON", 5, flippflop_cc1101_menu_callback, app);
    submenu_add_item(app->submenu, "Carrier OFF", 6, flippflop_cc1101_menu_callback, app);
    app->current_scene = SceneCC1101;
    view_dispatcher_switch_to_view(app->view_dispatcher, 1);
}

static void flippflop_show_adf4351_menu(FlippflopApp* app) {
    submenu_reset(app->submenu);
    submenu_add_item(app->submenu, "Set Frequency", 0, flippflop_adf4351_menu_callback, app);
    submenu_add_item(app->submenu, "Set Power (0-3)", 1, flippflop_adf4351_menu_callback, app);
    submenu_add_item(app->submenu, "RF Output ON", 2, flippflop_adf4351_menu_callback, app);
    submenu_add_item(app->submenu, "RF Output OFF", 3, flippflop_adf4351_menu_callback, app);
    submenu_add_item(app->submenu, "Sweep Band", 4, flippflop_adf4351_menu_callback, app);
    submenu_add_item(app->submenu, "Lock Status", 5, flippflop_adf4351_menu_callback, app);
    app->current_scene = SceneADF4351;
    view_dispatcher_switch_to_view(app->view_dispatcher, 1);
}

static void flippflop_show_antenna_menu(FlippflopApp* app) {
    submenu_reset(app->submenu);
    submenu_add_item(app->submenu, "Internal", 0, flippflop_antenna_menu_callback, app);
    submenu_add_item(app->submenu, "External-1", 1, flippflop_antenna_menu_callback, app);
    submenu_add_item(app->submenu, "External-2", 2, flippflop_antenna_menu_callback, app);
    submenu_add_item(app->submenu, "Dipole", 3, flippflop_antenna_menu_callback, app);
    app->current_scene = SceneAntenna;
    view_dispatcher_switch_to_view(app->view_dispatcher, 1);
}

// ============================================================================
// SCENE MANAGEMENT
// ============================================================================

static bool flippflop_custom_event_callback(void* context, uint32_t event) {
    FlippflopApp* app = (FlippflopApp*)context;
    // Record which scene we're entering so the back-button handler
    // (flippflop_navigation_event_callback) knows where "back" should go.
    app->current_scene = (SceneEnum)event;

    switch (event) {
        case SceneCC1101:
            flippflop_show_cc1101_menu(app);
            break;

        case SceneADF4351:
            flippflop_show_adf4351_menu(app);
            break;

        case SceneAntenna:
            flippflop_show_antenna_menu(app);
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
            
        case SceneADF4351Frequency:
            // ADF4351 frequency input
            text_input_reset(app->text_input);
            text_input_set_header_text(app->text_input, "Set Frequency (35-4400 MHz)");
            text_input_set_result_callback(app->text_input, flippflop_adf4351_freq_input_callback, app, app->frequency_input, sizeof(app->frequency_input), true);
            view_dispatcher_switch_to_view(app->view_dispatcher, 2);
            break;

        case SceneADF4351Power:
            // ADF4351 power input
            text_input_reset(app->text_input);
            text_input_set_header_text(app->text_input, "Set Power (0-3)");
            text_input_set_result_callback(app->text_input, flippflop_adf4351_power_input_callback, app, app->power_input, sizeof(app->power_input), true);
            view_dispatcher_switch_to_view(app->view_dispatcher, 2);
            break;

        case SceneCC1101Status:
            view_dispatcher_switch_to_view(app->view_dispatcher, 3);
            break;

        case SceneAbout:
            view_dispatcher_switch_to_view(app->view_dispatcher, 4);
            break;
    }

    return true;
}

// Handles the hardware Back button. There's no SceneManager here (the app
// hand-rolls its own scene switching above), so this is the only place
// Back is handled at all - without it, Back does nothing on any screen.
static bool flippflop_navigation_event_callback(void* context) {
    FlippflopApp* app = (FlippflopApp*)context;

    switch (app->current_scene) {
        case SceneCC1101:
        case SceneADF4351:
        case SceneAntenna:
        case SceneCC1101Status:
        case SceneAbout:
            // Back from any top-level submenu/screen -> main menu
            flippflop_show_main_menu(app);
            return true;

        case SceneCC1101Frequency:
        case SceneCC1101Power:
            // Back from a CC1101 input screen -> CC1101 menu
            flippflop_show_cc1101_menu(app);
            return true;

        case SceneADF4351Frequency:
        case SceneADF4351Power:
            // Back from an ADF4351 input screen -> ADF4351 menu
            flippflop_show_adf4351_menu(app);
            return true;

        case SceneMainMenu:
        default:
            // Already at the root menu -> exit the app
            view_dispatcher_stop(app->view_dispatcher);
            return true;
    }
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

    app->status_view = view_alloc();
    view_allocate_model(app->status_view, ViewModelTypeLockFree, sizeof(FlippflopApp*));
    with_view_model(
        app->status_view,
        FlippflopApp** model,
        { *model = app; },
        false);
    view_set_draw_callback(app->status_view, flippflop_status_draw_callback);

    app->about_view = view_alloc();
    view_set_draw_callback(app->about_view, flippflop_main_draw_callback);

    view_dispatcher_add_view(app->view_dispatcher, 1, submenu_get_view(app->submenu));
    view_dispatcher_add_view(app->view_dispatcher, 2, text_input_get_view(app->text_input));
    view_dispatcher_add_view(app->view_dispatcher, 3, app->status_view);
    view_dispatcher_add_view(app->view_dispatcher, 4, app->about_view);

    view_dispatcher_attach_to_gui(app->view_dispatcher, app->gui, ViewDispatcherTypeFullscreen);
    view_dispatcher_set_event_callback_context(app->view_dispatcher, app);
    view_dispatcher_set_custom_event_callback(app->view_dispatcher, flippflop_custom_event_callback);
    view_dispatcher_set_navigation_event_callback(
        app->view_dispatcher, flippflop_navigation_event_callback);

    // Initialize main menu
    flippflop_show_main_menu(app);

    // Initialize UART
    flippflop_uart_init(app);
    
    // Initialize state
    app->current_freq = 433.92f;
    app->current_rssi = -128;
    app->current_power = 7;
    app->current_antenna = 0;
    app->cc1101_present = false;
    app->adf4351_present = false;
    
    memset(app->status_text, 0, sizeof(app->status_text));
    snprintf(app->status_text, sizeof(app->status_text), "Ready");
    memset(app->frequency_input, 0, sizeof(app->frequency_input));
    memset(app->power_input, 0, sizeof(app->power_input));

    return app;
}

static void flippflop_app_free(FlippflopApp* app) {
    if (app->uart_handle) {
        furi_hal_serial_async_rx_stop(app->uart_handle);
        furi_hal_serial_deinit(app->uart_handle);
        furi_hal_serial_control_release(app->uart_handle);
    }

    if (app->uart_rx_buffer) {
        furi_stream_buffer_free(app->uart_rx_buffer);
    }
    
    view_dispatcher_remove_view(app->view_dispatcher, 1);
    view_dispatcher_remove_view(app->view_dispatcher, 2);
    view_dispatcher_remove_view(app->view_dispatcher, 3);
    view_dispatcher_remove_view(app->view_dispatcher, 4);

    submenu_free(app->submenu);
    text_input_free(app->text_input);
    view_free_model(app->status_view);
    view_free(app->status_view);
    view_free(app->about_view);
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
