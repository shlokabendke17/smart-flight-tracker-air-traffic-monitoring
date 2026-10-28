// Wokwi Custom Chip - For docs and examples see:
// https://docs.wokwi.com/chips-api/getting-started
//
// SPDX-License-Identifier: MIT
// Copyright 2023 Shloka Bendke

#include "wokwi-api.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  uart_dev_t uart;
  uint32_t latitude, longitude, fix;
  uint8_t buffer[240];
} chip_state_t;

static void nmea_coordinate(double value, int width,
                            char *out, size_t size) {
  double positive = fabs(value);
  int degrees = (int)positive;
  double minutes = (positive - degrees) * 60;

  if (minutes >= 59.99995) {
    degrees++;
    minutes = 0;
  }

  snprintf(out, size, "%0*d%07.4f", width, degrees, minutes);
}

static size_t sentence(char *out, size_t size,
                       const char *body) {
  unsigned char checksum = 0;

  for (const char *p = body; *p; p++) {
    checksum ^= (unsigned char)*p;
  }

  int length = snprintf(out, size, "$%s*%02X\r\n",
                        body, checksum);

  return length > 0 && (size_t)length < size
         ? (size_t)length : 0;
}

static void gps_tick(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;

  double lat = attr_read_float(chip->latitude);
  double lon = attr_read_float(chip->longitude);
  int fix = attr_read(chip->fix) != 0;

  char latitude[16], longitude[16], body[160], utc[12];

  nmea_coordinate(lat, 2, latitude, sizeof(latitude));
  nmea_coordinate(lon, 3, longitude, sizeof(longitude));

  unsigned int seconds =
      (get_sim_nanos() / 1000000000ULL) % 86400;

  snprintf(utc, sizeof(utc), "%02u%02u%02u.00",
           seconds / 3600, seconds / 60 % 60, seconds % 60);

  snprintf(body, sizeof(body),
           "GPRMC,%s,%c,%s,%c,%s,%c,0.0,0.0,061026,,,A",
           utc, fix ? 'A' : 'V',
           latitude, lat < 0 ? 'S' : 'N',
           longitude, lon < 0 ? 'W' : 'E');

  size_t length = sentence(
      (char *)chip->buffer, sizeof(chip->buffer), body);

  snprintf(body, sizeof(body),
           "GPGGA,%s,%s,%c,%s,%c,%d,%02d,1.0,20.0,M,0.0,M,,",
           utc,
           latitude, lat < 0 ? 'S' : 'N',
           longitude, lon < 0 ? 'W' : 'E',
           fix ? 1 : 0, fix ? 8 : 0);

  length += sentence(
      (char *)chip->buffer + length,
      sizeof(chip->buffer) - length, body);

  uart_write(chip->uart, chip->buffer, length);
}

void chip_init(void) {
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));

  chip->latitude = attr_init_float("latitude", 16.9902f);
  chip->longitude = attr_init_float("longitude", 73.3120f);
  chip->fix = attr_init("fix", 1);

  pin_init("VCC", INPUT);
  pin_init("GND", INPUT);

  uart_config_t uart = {
    .rx = pin_init("RX", INPUT),
    .tx = pin_init("TX", INPUT_PULLUP),
    .baud_rate = 9600,
    .user_data = chip
  };

  chip->uart = uart_init(&uart);

  timer_config_t timer = {
    .callback = gps_tick,
    .user_data = chip
  };

  timer_t timer_id = timer_init(&timer);
  timer_start(timer_id, 1000000, true);
}
