// Host-test stand-in for the Pico SDK's hardware/vreg.h: the codes core_vreg.h
// uses (the same on RP2040 and RP2350, hardware_vreg/include/hardware/vreg.h).
#pragma once
enum vreg_voltage {
    VREG_VOLTAGE_1_10 = 0b01011,
    VREG_VOLTAGE_1_15 = 0b01100,
    VREG_VOLTAGE_1_20 = 0b01101,
    VREG_VOLTAGE_1_25 = 0b01110,
    VREG_VOLTAGE_1_30 = 0b01111,
};
