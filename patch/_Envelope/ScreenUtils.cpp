#include "ScreenUtils.h"
#include <cstdarg>
#include <cstdio>

using namespace daisy;

// Helper function to format strings with fixed width to prevent display overlap
// Pads short strings with spaces, truncates long strings
void FormatFixedWidth(char* dest, size_t destSize, const char* src, int width)
{
    snprintf(dest, destSize, "%-*s", width, src);
    // Ensure null termination in case of truncation
    dest[destSize - 1] = '\0';
}

// Write a fixed-width string to display at specified position
// Prevents overlap by padding/truncating to exact width
void WriteFixedString(DaisyPatch& hw, int x, int y, int width, FontDef font, const char* str)
{
    char buffer[128];
    size_t bufferSize = (width + 1 < 128) ? width + 1 : 128;
    FormatFixedWidth(buffer, bufferSize, str, width);
    hw.display.SetCursor(x, y);
    hw.display.WriteString(buffer, font, true);
}

// Write a formatted fixed-width string to display (printf-style)
void WriteFixedStringF(DaisyPatch& hw, int x, int y, int width, FontDef font, const char* format, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    char fixedBuffer[128];
    size_t bufferSize = (width + 1 < 128) ? width + 1 : 128;
    FormatFixedWidth(fixedBuffer, bufferSize, buffer, width);
    hw.display.SetCursor(x, y);
    hw.display.WriteString(fixedBuffer, font, true);
}

