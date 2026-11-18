#pragma once
#include <Arduino.h>

const int MAX_TIMES = 8;
const int MAX_DAYS = 365;
const int NAME_LEN = 24;

struct Plan {
  uint8_t enabled;
  uint8_t count;
  uint16_t days;
  uint32_t startDay;
  uint16_t times[MAX_TIMES];
  char name[NAME_LEN + 1];
};

struct PlanStat {
  int state;
  long dayNo;
  long daysLeft;
  unsigned long nextEpoch;
};
