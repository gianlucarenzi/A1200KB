#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <stdint.h>

// --- Definizioni per la Matrice della Tastiera ---
#define MATRIX_ROWS 6
#define MATRIX_COLS 22

extern const uint8_t scancode_lut[MATRIX_ROWS][MATRIX_COLS];

#endif // KEYBOARD_H
