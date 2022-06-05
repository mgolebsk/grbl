#include "grbl.h"

void sensor_init() {
    SENSOR_DDR &= ~(SENSOR_MASK); // Set as input pins
    SENSOR_PORT |= (SENSOR_MASK);  // Enable internal pull-up resistors. Normal high operation.
}

uint8_t sensors_get_state() {
    return (SENSOR_PIN & SENSOR_MASK); // READ VALUES
}