#include "../lib/OccupancyTimer.h"

OccupancyDetector *OccupancyTimer::detector = NULL;

unsigned long OccupancyTimer::last_tick = millis();
bool OccupancyTimer::sat_too_long_alarm = false;

unsigned long OccupancyTimer::sit_time_limit = DEFAULT_SIT_TIME;
unsigned long OccupancyTimer::stand_time_limit = DEFAULT_STAND_TIME;
unsigned long OccupancyTimer::total_sit_time = 0;
unsigned long OccupancyTimer::total_stand_time = 0;

const BuzzerPattern *OccupancyTimer::alarm_pattern = &ALARM_PATTERN;
TaskHandle_t OccupancyTimer::handle = NULL;

void alarm(void *arg) {
	BuzzerPattern *pattern = (BuzzerPattern *)arg;
	pinMode(pattern->pin, OUTPUT);

	bool buzzer_on = false;
	while(true) {
		if(buzzer_on) {
			noTone(pattern->pin);
			vTaskDelay(pdMS_TO_TICKS(pattern->off_ms));
		} else {
			tone(pattern->pin, 800);
			vTaskDelay(pdMS_TO_TICKS(pattern->on_ms));
		}
		buzzer_on = !buzzer_on;
	}
}

void OccupancyTimer::start_alarm(TaskHandle_t &handle, const BuzzerPattern *pattern) {
	if(handle == NULL) {
		xTaskCreate(alarm, "alarm", 2048, (void *)pattern, 1, &handle);
	}
}

void OccupancyTimer::stop_alarm(TaskHandle_t &handle, const BuzzerPattern *pattern) {
	if(handle != NULL) {
		vTaskDelete(handle);
		handle = NULL;
	
		noTone(pattern->pin);
	}
}

/* occupied() == is sitting */
void OccupancyTimer::Tick() {
	if(alarm_pattern != NULL) { /* cursed error handler */
		if(detector != NULL) {
			unsigned long now = millis();
			unsigned long delta = now - last_tick;
			last_tick = now;

			if(detector->occupied()) { /* user is sitting */
				total_sit_time += delta;
				
				if(!sat_too_long_alarm) {
					total_stand_time = 0; /* reset the standing time if the alarm is still going. must stand for the entire time */

					if(total_sit_time >= sit_time_limit) { /* turn on the alarm */
						sat_too_long_alarm = true;
					}
				}

				if(sat_too_long_alarm) { /* turn on the buzzer */
					start_alarm(handle, alarm_pattern);
				}
			} else { /* user is standing */
				total_stand_time += delta; /* stand time can accumulate if the alarm has not gone off (eg: stand for 1.5 minutes, then 0.5 minutes, this is sufficient to reset the sitting timer ) */
				stop_alarm(handle, alarm_pattern);

				if(total_stand_time >= stand_time_limit) { /* user stood for long enough */
					sat_too_long_alarm = false;
					total_sit_time = total_stand_time = 0;
				}
			}
		} else {
			Serial.println("OccupancyTimer needs OccupancyDetector, please call init!");
			alarm_pattern = NULL;
		}
	}
}

void OccupancyTimer::Init(OccupancyDetector *_detector, const BuzzerPattern *_pattern) {
	detector = _detector;
	alarm_pattern = (_pattern) ? _pattern : &ALARM_PATTERN;
}