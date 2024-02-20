#pragma once

#include <stdbool.h>
#include "digital_input.h"

void ux_attention();

void ux_signal_unprovisioned();

void ux_signal_provisioned();

void ux_signal_provisioning_state(bool isProvisioned);

void ux_signal_reset_initiative_started();

void ux_init(input_signal_f press_callback);