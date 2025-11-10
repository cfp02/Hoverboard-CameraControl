#include "OLEDScreen.h"

OLEDScreen::OLEDScreen() : u8g2(U8G2_R2, U8X8_PIN_NONE, 6, 5) {
    horizontalScrollEnabled = false;
    scrollX = 0;
    for (int i = 0; i < MAX_SCROLL_LINES; i++) {
        scrollPositions[i] = 0;
        lineActive[i] = false;
        scrollBuffer[i][0] = '\0';
    }
}

OLEDScreen::OLEDScreen(uint8_t clockPin, uint8_t dataPin) 
    : u8g2(U8G2_R2, U8X8_PIN_NONE, clockPin, dataPin) {
    horizontalScrollEnabled = false;
    scrollX = 0;
    for (int i = 0; i < MAX_SCROLL_LINES; i++) {
        scrollPositions[i] = 0;
        lineActive[i] = false;
        scrollBuffer[i][0] = '\0';
    }
}

void OLEDScreen::begin() {
    u8g2.begin();
    u8g2.setContrast(255);
    u8g2.setBusClock(400000);
}

void OLEDScreen::setContrast(uint8_t contrast) {
    u8g2.setContrast(contrast);
}

void OLEDScreen::setBusClock(uint32_t clockSpeed) {
    u8g2.setBusClock(clockSpeed);
}

void OLEDScreen::setFont(const uint8_t* font) {
    u8g2.setFont(font);
}

void OLEDScreen::clear() {
    u8g2.clearBuffer();
}

void OLEDScreen::update() {
    u8g2.sendBuffer();
}

void OLEDScreen::print(const char* text) {
    u8g2.print(text);
}

void OLEDScreen::printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    u8g2.printf(format, args);
    va_end(args);
}

void OLEDScreen::setCursor(int16_t x, int16_t y) {
    u8g2.setCursor(xOffset + x, yOffset + y);
}

void OLEDScreen::drawFrame(int16_t x, int16_t y, int16_t w, int16_t h) {
    u8g2.drawFrame(xOffset + x, yOffset + y, w, h);
}

void OLEDScreen::drawBox(int16_t x, int16_t y, int16_t w, int16_t h) {
    u8g2.drawBox(xOffset + x, yOffset + y, w, h);
}

void OLEDScreen::drawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
    u8g2.drawLine(xOffset + x1, yOffset + y1, xOffset + x2, yOffset + y2);
}

void OLEDScreen::enableHorizontalScroll(bool enable) {
    horizontalScrollEnabled = enable;
    if (!enable) {
        scrollX = 0;
        for (int i = 0; i < MAX_SCROLL_LINES; i++) {
            scrollPositions[i] = 0;
            lineActive[i] = false;
        }
    }
}

void OLEDScreen::setScrollOffset(int16_t x, int16_t y) {
    scrollX = x;
}

void OLEDScreen::scrollTextHorizontal(const char* text, int lineNumber) {
    if (!horizontalScrollEnabled || lineNumber < 0 || lineNumber >= MAX_SCROLL_LINES) return;
    
    // Store the text in the buffer, ensuring null termination
    strncpy(scrollBuffer[lineNumber], text, MAX_TEXT_LENGTH - 1);
    scrollBuffer[lineNumber][MAX_TEXT_LENGTH - 1] = '\0';
    
    // Reset scroll position and activate the line
    scrollPositions[lineNumber] = 0;
    lineActive[lineNumber] = true;
}

void OLEDScreen::updateAllLines() {
    if (!horizontalScrollEnabled) return;
    
    // Clear display
    clear();
    
    // Draw all active lines
    for (int i = 0; i < MAX_SCROLL_LINES; i++) {
        if (lineActive[i]) {
            // Calculate text width (6 pixels per character)
            int textWidth = strlen(scrollBuffer[i]) * 6;
            int scrollPosition = scrollPositions[i];
            
            // Calculate vertical position (starting at 7, then 17, 27, 37)
            int yPos = 7 + (i * 10);
            
            // Draw text at scroll position
            setCursor(-scrollPosition, yPos);
            print(scrollBuffer[i]);
            
            // If text is longer than screen, draw it again to create continuous scroll
            if (textWidth > ScreenWidth) {
                // Add 7 spaces (42 pixels) between text repetitions
                setCursor(-scrollPosition + textWidth + 42, yPos);
                print(scrollBuffer[i]);
            }
            
            // Update scroll position
            scrollPositions[i]++;
            
            // Reset position when we reach the end of the text plus the space gap
            if (scrollPositions[i] >= textWidth + 42) {
                scrollPositions[i] = 0;
            }
        }
    }
    
    update();
}

void OLEDScreen::clearScrollBuffer() {
    for (int i = 0; i < MAX_SCROLL_LINES; i++) {
        scrollPositions[i] = 0;
        lineActive[i] = false;
        scrollBuffer[i][0] = '\0';
    }
} 