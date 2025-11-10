#pragma once

#include <Wire.h>
#include <U8g2lib.h>

class OLEDScreen {
public:
    // Constructor with default pins (clock=6, data=5)
    OLEDScreen();
    
    // Constructor with custom pins
    OLEDScreen(uint8_t clockPin, uint8_t dataPin);
    
    // Initialize the display
    void begin();
    
    // Set contrast (0-255)
    void setContrast(uint8_t contrast);
    
    // Set bus clock speed
    void setBusClock(uint32_t clockSpeed);
    
    // Set font
    void setFont(const uint8_t* font);
    
    // Clear the display
    void clear();
    
    // Update the display with current buffer
    void update();
    
    // Draw text at current cursor position
    void print(const char* text);
    void printf(const char* format, ...);
    
    // Set cursor position (relative to screen offset)
    void setCursor(int16_t x, int16_t y);
    
    // Draw basic shapes
    void drawFrame(int16_t x, int16_t y, int16_t w, int16_t h);
    void drawBox(int16_t x, int16_t y, int16_t w, int16_t h);
    void drawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
    
    // Get screen dimensions
    int16_t getWidth() const { return ScreenWidth; }
    int16_t getHeight() const { return ScreenHeight; }
    
    // Get buffer dimensions
    int16_t getBufferWidth() const { return BufferWidth; }
    int16_t getBufferHeight() const { return BufferHeight; }

    // Scrolling functionality
    void enableHorizontalScroll(bool enable);
    void setScrollOffset(int16_t x, int16_t y);
    void scrollTextHorizontal(const char* text, int lineNumber);
    void updateAllLines();
    void clearScrollBuffer();

private:
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
    
    // Screen dimensions
    static const unsigned int BufferWidth = 132;
    static const unsigned int BufferHeight = 64;
    static const unsigned int ScreenWidth = 72;
    static const unsigned int ScreenHeight = 40;
    static const unsigned int xOffset = (BufferWidth - ScreenWidth) / 2;
    static const unsigned int yOffset = (BufferHeight - ScreenHeight) / 2;

    // Scrolling state
    bool horizontalScrollEnabled;
    int16_t scrollX;
    static const int MAX_SCROLL_LINES = 4;  // Maximum number of scrollable lines
    static const int MAX_TEXT_LENGTH = 200;  // Maximum length for scrollable text
    char scrollBuffer[MAX_SCROLL_LINES][MAX_TEXT_LENGTH];  // Buffer to store scrollable text
    int scrollPositions[MAX_SCROLL_LINES];    // Current scroll position for each line
    bool lineActive[MAX_SCROLL_LINES];        // Whether each line is still scrolling
}; 