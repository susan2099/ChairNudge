#ifndef OCCUPANCY_TIMER
#define OCCUPANCY_TIMER

#include <Arduino.h>
#include "occupancy.h"

constexpr uint8_t BUZZER_PIN = 27;

const unsigned long 
	DEFAULT_SIT_TIME = 45*60*1000, 
	DEFAULT_STAND_TIME = 2*60*1000;

struct BuzzerPattern {
	uint8_t pin;
	uint16_t on_ms;
	uint16_t off_ms;
};

const BuzzerPattern ALARM_PATTERN{BUZZER_PIN, 250, 250};

class OccupancyTimer {
private:
	static OccupancyDetector *detector;
	static unsigned long last_tick;
	static bool sat_too_long_alarm;

	static unsigned long sit_time_limit;
	static unsigned long stand_time_limit;
	static unsigned long total_stand_time;
	static unsigned long total_sit_time;

	static const BuzzerPattern *alarm_pattern;
	static TaskHandle_t handle;

	static void start_alarm(TaskHandle_t &, const BuzzerPattern *);
	static void stop_alarm(TaskHandle_t &, const BuzzerPattern *);
public:
	static void Init(OccupancyDetector *, const BuzzerPattern * = NULL);
	static void Tick();
};

#endif