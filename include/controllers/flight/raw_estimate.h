#ifndef _BLUEJAY_FLIGHT_RAW_ESTIMATE_H
#define _BLUEJAY_FLIGHT_RAW_ESTIMATE_H

#include <stdint.h>

#define FLIGHT_RAW_CLOSE 0
#define FLIGHT_RAW_POSITION 1
#define FLIGHT_RAW_ORIENTATION 2
#define FLIGHT_RAW_GYRO 3
#define FLIGHT_RAW_ACC 4

struct flight_raw_estimate{
    uint8_t type;
    uint64_t time;
    float data[4];  
} __attribute__((packed));

#endif
