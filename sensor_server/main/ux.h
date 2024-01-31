#pragma once

#include <stdbool.h>

void ux_attention();

void ux_signal_unprovisioned();

void ux_signal_provisioned();

void ux_signal_provisioning_state(bool isProvisioned);

void ux_signal_reset_initiative_started();