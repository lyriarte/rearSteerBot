/*
 * Copyright (c) 2015, Luc Yriarte
 * License: BSD <http://www.opensource.org/licenses/bsd-license.php>
 */

#include <Servo.h> 


/* **** **** **** **** **** ****
 * Build directives
 * **** **** **** **** **** ****/
#define LOG_DEBUG 0

/* **** **** **** **** **** ****
 * Constants
 * **** **** **** **** **** ****/

#ifdef LOG_DEBUG
char * decisionLevelName[] = {
	"AVOIDANCE",
	"TRAJECTORY",
	"FREE RUN"
};
#endif

/* 
 * gpio mappings
 */
#define STEERSERVO 7
#define ENGINERELAY 13
#define INLEFT 10
#define INRIGHT 11
#define INSTOP 12
#define INLEFTECHO 2
#define LEFTTRIGGER 3
#define INRIGHTECHO 4
#define RIGHTTRIGGER 5

/* 
 * control loop frequency
 */
#define POLL 50

/* 
 * action
 */
#define STOP 0
#define STRAIGHT 90
#define LEFT 140
#define RIGHT 40
/* STRAIGHT + STEERDELTA * MAXDELTARANGE = LEFT */
#define STEERDELTA 1
/* STRAIGHT + STEERRANGE = LEFT / STRAIGHT - STEERRANGE = RIGHT */
#define STEERRANGE 50

/* 
 * perception
 */
#define MAXRANGE 200
#define MINRANGE  50
#define STOPRANGE 20
#define ECHO_TIMEOUT 20000
#define ECHO2CM(x) (x/60) 
#define CMBUFSZ 20
#define MAXDELTARANGE 50
#define MINDELTARANGE 2
#define MISSED_PERCEPTION_INCREMENT 5

/* 
 * internal state
 */
#define DELTA_STEER 5
#define DELTA_DELAY 2

/* **** **** **** **** **** ****
 * Global variables
 * **** **** **** **** **** ****/

/* 
 * action
 */
Servo steerServo;
int steer;
int speed;

/* 
 * perception
 */
/* immediate perception */
int cmLeft;
int cmRight;
/* persistant perception */
int cmLeftBuf[CMBUFSZ];
int cmRightBuf[CMBUFSZ];
int iCmBuf;
int cmDeltaLeft;
int cmDeltaRight;
int cmAvgLeft;
int cmAvgRight;

/* 
 * internal state
 */
int steerAdjust;
int speedDelay;
/* decision level */ 
int decisionLevel;


/* **** **** **** **** **** ****
 * Functions
 * **** **** **** **** **** ****/

/* 
 * setup
 */
void setup() {
	/* globals initialization */
	steerAdjust = 0;
	speedDelay = 0; 
	steer = STRAIGHT;
	speed = 0;
	for (iCmBuf=0; iCmBuf<CMBUFSZ; iCmBuf++) {
		cmLeftBuf[iCmBuf] = 0;
		cmRightBuf[iCmBuf] = 0;
	}
	cmLeft = 0;
	cmRight = 0;
	iCmBuf = 0;
	cmDeltaLeft = 0;
	cmDeltaRight = 0;
	cmAvgLeft = 0;
	cmAvgRight = 0;
	/* gpio mappings */
	pinMode(STEERSERVO, OUTPUT);
	pinMode(INLEFT, INPUT);
	pinMode(INRIGHT, INPUT);
	pinMode(ENGINERELAY, OUTPUT);
	pinMode(INSTOP, INPUT);
	pinMode(INLEFTECHO, INPUT);
	pinMode(LEFTTRIGGER, OUTPUT);
	pinMode(INRIGHTECHO, INPUT);
	pinMode(RIGHTTRIGGER, OUTPUT);
	/* subsystems setup */
	steerServo.attach(STEERSERVO);
#if LOG_DEBUG
	Serial.begin(9600);
#endif
	/* initial action state */
	steerServo.write(steer);
	digitalWrite(ENGINERELAY, LOW);
	digitalWrite(LEFTTRIGGER, LOW);
	digitalWrite(RIGHTTRIGGER, LOW);
}

/*
 * ultra sonic sensors perception function
 * 	gpio in:	INLEFTECHO, INRIGHTECHO
 * 	gpio out:	LEFTTRIGGER, RIGHTTRIGGER
 * 	perception vars out: cmLeft, cmRight
 */
void frontUltrasonicPerception() {
	unsigned long echoDuration;
	digitalWrite(LEFTTRIGGER, HIGH);
	delayMicroseconds(10);
	digitalWrite(LEFTTRIGGER, LOW);
	echoDuration = pulseIn(INLEFTECHO, HIGH, ECHO_TIMEOUT);
	cmLeft = echoDuration ? ECHO2CM(echoDuration) : min(cmLeft + MISSED_PERCEPTION_INCREMENT,MAXRANGE);
	digitalWrite(RIGHTTRIGGER, HIGH);
	delayMicroseconds(10);
	digitalWrite(RIGHTTRIGGER, LOW);
	echoDuration = pulseIn(INRIGHTECHO, HIGH, ECHO_TIMEOUT);
	cmRight = echoDuration ? ECHO2CM(echoDuration) : min(cmRight + MISSED_PERCEPTION_INCREMENT,MAXRANGE);
}

/*
 * stores 1 second (20 * 50 ms) of ultra sonic sensors data
 * to calculate the how fast the robots comes near obstacles.
 * sums buffer deltas for a smooth perception of the last second.
 * vars in: immediate perception
 * vars out: persistent perception
 */
void frontUltrasonicPerceptionMemory() {
	int prevIndex = iCmBuf > 0 ? (iCmBuf - 1) % CMBUFSZ : CMBUFSZ - 1;
	int nextIndex = (iCmBuf + 1) % CMBUFSZ;
	/*  add latest mesure to / substract oldest mesure from average */
	cmAvgLeft = cmAvgLeft + (cmLeft - cmLeftBuf[iCmBuf]) / CMBUFSZ;
	cmAvgRight = cmAvgRight + (cmRight - cmRightBuf[iCmBuf]) / CMBUFSZ;
	/* substract delta between oldest and second oldest mesure */
	cmDeltaLeft -= (cmLeftBuf[iCmBuf] - cmLeftBuf[nextIndex]);
	cmDeltaRight -= (cmRightBuf[iCmBuf] - cmRightBuf[nextIndex]);
	/* add delta between last saved and latest mesure */
	cmDeltaLeft += cmLeftBuf[prevIndex] - cmLeft;
	cmDeltaRight += cmRightBuf[prevIndex] - cmRight;
	/* store latest mesure */
	cmLeftBuf[iCmBuf] = cmLeft;
	cmRightBuf[iCmBuf] = cmRight;
	iCmBuf = nextIndex;
}

/* 
 * reactive avoidance decision function
 */
boolean avoidance() {
	if (cmLeft < STOPRANGE || cmRight < STOPRANGE) {
		steer = STRAIGHT;
		speed = 0;
		return true;
	}
	if (cmLeft < MINRANGE && cmRight < MINRANGE) {
		steer = (cmLeft > cmRight) ? LEFT : RIGHT;
		return true;
	}
	if (cmLeft < MINRANGE) {
		steer = RIGHT;
		return true;
	}
	if (cmRight < MINRANGE) {
		steer = LEFT;
		return true;
	}
	return false;
}

/* 
 * trajectory decision function
 */
boolean trajectory() {
	steer = STRAIGHT;
	speed = 1;
	/* straight ahead */
	if (cmRight >= MAXRANGE && cmLeft >= MAXRANGE)
		return false;
	/* steer away from the closest fast approaching obstacle */
	if (cmRight < cmLeft && cmDeltaRight > MINDELTARANGE)
		steer = steer + min(cmDeltaRight,MAXDELTARANGE) * STEERDELTA;
	if (cmLeft < cmRight && cmDeltaLeft > MINDELTARANGE)
		steer = steer - min(cmDeltaLeft,MAXDELTARANGE) * STEERDELTA;
	/* if going straight, steer slightly to the most open area */
	if (steer == STRAIGHT && cmLeft >= MAXRANGE && cmAvgLeft >= MAXRANGE)
		steer = steer + STEERRANGE / 4;
	if (steer == STRAIGHT && cmRight >= MAXRANGE && cmAvgRight >= MAXRANGE)
		steer = steer - STEERRANGE / 4;
	/* stop temporarily if approaching too fast on both sides */
	if (cmDeltaLeft > MAXDELTARANGE && cmDeltaRight > MAXDELTARANGE
		&& cmAvgLeft < MAXRANGE / 2 && cmAvgRight < MAXRANGE / 2)
		speed = 0;
	return true;
}

/* 
 * control loop
 */
void loop() {
	/* log message */
#if LOG_DEBUG
	String logMsg;
#endif
	/* control loop frequency */
	unsigned long timeLoopStart;
	unsigned long timeLoop;
	timeLoopStart = millis();
	/* perception */
	frontUltrasonicPerception();
	frontUltrasonicPerceptionMemory();
	/* inner state calibration */
	if (digitalRead(INLEFT) == HIGH)
		steerAdjust += DELTA_STEER;
  	if (digitalRead(INRIGHT) == HIGH)
		steerAdjust -= DELTA_STEER;
	if (digitalRead(INSTOP) == HIGH)
		speedDelay += DELTA_DELAY;
	/* inner state */
	if (speedDelay != 0) {
		digitalWrite(ENGINERELAY, LOW);
		delay(speedDelay);
	}
	/* decision */
	decisionLevel = 0;
	if (!avoidance()) {
		++decisionLevel;
		if (!trajectory()) {
			++decisionLevel;
		}
	}
#if LOG_DEBUG
	/* log message construction */
	logMsg = decisionLevelName[decisionLevel];
	logMsg += "\t[" + String(timeLoopStart) + "] ";
	logMsg += String(cmLeft) + "," + String(cmAvgLeft) + "," + String(cmDeltaLeft);
	logMsg += " | ";
	logMsg += String(cmRight) + "," + String(cmAvgRight) + "," + String(cmDeltaRight);
	logMsg += "\t -> " + String(steer) + " , " + String(speed);
	/* print log message */
	Serial.println(logMsg);
#endif
	/* action */
	digitalWrite(ENGINERELAY, speed ? HIGH : LOW);
	steerServo.write(steer+steerAdjust);
	/* control loop frequency */
	timeLoop = millis() - timeLoopStart;
	if (timeLoop < POLL)
		delay(POLL - timeLoop);
}

