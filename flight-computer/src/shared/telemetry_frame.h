//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~ telemetry_frame.h - shared M7/M4: XBEE telemetry payload and frame ~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Frame format, health bit meanings, ground-station contract : README 4, 6.4
#pragma once
#include <stdint.h>
#include <stddef.h>

//~~~~~~~~~~~~~~~~~~~~~~~ XBEE telemetry payload ~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
struct __attribute__((packed)) PackedSensorData {
    uint8_t state = 0xBA;
    float   time = 0.0f;
    float   lat = 0.0f;
    float   lon = 0.0f;
    float   euler[3] = {0, 0, 0};
    float   accel[3] = {0, 0, 0};
    float   gyro[3]  = {0, 0, 0};
    float   pos[3]   = {0, 0, 0};
    float   altitude = 0.0f;
    float   pressure = 0.0f;
    float   kalman[4] = {0, 0, 0, 0};
    float   gnssAlt = 0.0f;   // GNSS hMSL, m - log/recovery ONLY, never control
    uint8_t health = 0;
};

static_assert(sizeof(PackedSensorData) == 90,
              "PackedSensorData must stay 90B - the ground station unpacks "
              "'<B'+'f'*22+'B'; a size change silently breaks the downlink");

//~~~~~~~~~~~~~~~~~~ health flags: set = good, 0xFF = nominal ~~~~~~~~~~~~~~~//
static const uint8_t TLM_IMU_FRESH   = 0x01;
static const uint8_t TLM_BARO_FRESH  = 0x02;
static const uint8_t TLM_IMU_ALIVE   = 0x04;
static const uint8_t TLM_BARO_ALIVE  = 0x08;
static const uint8_t TLM_GNSS_ALIVE  = 0x10;
static const uint8_t TLM_BARO_REF    = 0x20;
static const uint8_t TLM_VZ_TRUSTED  = 0x40;
static const uint8_t TLM_VZ_USABLE   = 0x80;   // trusted AND past the gap blanking
static const uint8_t TLM_HEALTH_ALL  = 0xFF;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~ XBEE wire format ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
static const uint8_t FRAME_START     = 0x7E;
static const uint8_t FRAME_END       = 0x0A;
static const size_t  FRAME_LEN       = 1 + sizeof(PackedSensorData) + 1;
static const uint8_t XBEE_LOG_PREFIX = 0xAA;
