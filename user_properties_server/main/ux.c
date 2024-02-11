#include "ux.h"
#include "board.h"
#include "digital_input.h"

void ux_attention() {
    board_rgb_led_control(INTENSIVE_WHITE);
}

void ux_signal_unprovisioned() {
    board_rgb_led_control(RED);
}

void ux_signal_provisioned() {
    board_rgb_led_control(GREEN);
}

void ux_signal_provisioning_state(bool isProvisioned) {
    if (isProvisioned) {        
        ux_signal_provisioned();
    } else {
        ux_signal_unprovisioned();
    }
}

void ux_signal_reset_initiative_started() {
    board_rgb_led_control(ORANGE);
}

void ux_init(input_signal_f press_callback) {
    board_init(press_callback);
}