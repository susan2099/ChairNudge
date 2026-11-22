#include "../lib/OccupancyTimer.h"

OccupancyDetector *OccupancyTimer::detector = NULL;

unsigned long OccupancyTimer::sit_begin_time = 0;
unsigned long OccupancyTimer::stand_begin_time = 0;
bool OccupancyTimer::sat_too_long_alarm = false;

unsigned long OccupancyTimer::sit_time = DEFAULT_SIT_TIME;
unsigned long OccupancyTimer::stand_time = DEFAULT_STAND_TIME;
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

void OccupancyTimer::UserSat() {
	unsigned long now = millis();

	if(sat_too_long_alarm) { /* user sat while alarm is still on, resume the buzzer */
		total_stand_time += (now - stand_begin_time);
		start_alarm(handle, alarm_pattern);
	} else { /* alarm is not on */
		sit_begin_time = now;
	}
}

void OccupancyTimer::UserStood() {
	unsigned long now = millis();

	if(sat_too_long_alarm) { /* alarm is on, turn off the buzzer */
		stop_alarm(handle, alarm_pattern);
		stand_begin_time = now;
	} else { /* alarm is not on */
		total_sit_time += (now - sit_begin_time);
	}
}

/* occupied() == is sitting */
void OccupancyTimer::Tick() {
	if(alarm_pattern != NULL) { /* cursed error handler */
		if(detector != NULL) {
			unsigned long now = millis();

			if(sat_too_long_alarm) { /* alarm is on */
				if( /* user stood for long enough, turn off alarm and buzzer */
					(total_stand_time >= stand_time) || 
					(!detector->occupied() && now - stand_begin_time + total_stand_time >= stand_time)
				) {
					sat_too_long_alarm = false;
					total_sit_time = total_stand_time = 0;
					sit_begin_time = now;
					stop_alarm(handle, alarm_pattern);
				}
			} else { /* alarm is not on */
				if( /* user sat too long, turn on alarm and buzzer */
					(total_sit_time >= sit_time) || 
					(detector->occupied() && now - sit_begin_time + total_sit_time >= sit_time)
				) {
					sat_too_long_alarm = true;
					total_stand_time = 0;
					stand_begin_time = now;

					if(detector->occupied()) { start_alarm(handle, alarm_pattern); }
				}
			}
		} else {
			Serial.println("OccupancyTimer needs OccupancyDetector, please call init!");
			alarm_pattern = NULL;
		}
	} 
}

void OccupancyTimer::Init(OccupancyDetector *_detector) {
	detector = _detector;
}