#pragma once

#include "daisy_patch.h"

// Screen utility functions for fixed-width display text
// Prevents text overlap by ensuring consistent character widths

// Format a string with fixed width (padding/truncating as needed)
void FormatFixedWidth(char* dest, size_t destSize, const char* src, int width);

// Write a fixed-width string to display at specified position
// Parameters: hw, x, y, width, font, str
void WriteFixedString(daisy::DaisyPatch& hw, int x, int y, int width, FontDef font, const char* str);

// Write a formatted fixed-width string to display (printf-style)
// Parameters: hw, x, y, width, font, format, ...
void WriteFixedStringF(daisy::DaisyPatch& hw, int x, int y, int width, FontDef font, const char* format, ...);

