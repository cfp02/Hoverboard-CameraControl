#pragma once

#include <stddef.h>
#include <stdint.h>

//List of mac addresses with names
struct HoverboardMAC {
    const char* name;
    const uint8_t address[6];
};

static const HoverboardMAC HOVERBOARD_MACS[] = {
    {"H1", { 0xAC, 0x67, 0xB2, 0x53, 0x86, 0x28 }},
    {"H2", { 0xE8, 0xDB, 0x84, 0x03, 0xF1, 0x80 }}
};

static const size_t NUM_HOVERBOARDS = sizeof(HOVERBOARD_MACS) / sizeof(HOVERBOARD_MACS[0]);
