#pragma once

typedef void (*input_signal_f)(int);

void digital_input_init(input_signal_f func);