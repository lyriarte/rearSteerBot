/*
 * Copyright (c) 2015, Luc Yriarte
 * License: BSD <http://www.opensource.org/licenses/bsd-license.php>
 */

#include <Servo.h> 


/* **** **** **** **** **** ****
 * Constants
 * **** **** **** **** **** ****/

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

/* 
 * perception
 */
#define MAXRANGE 200
#define MINRANGE  50
#define STOPRANGE 20
#define MAXRANGE_0 120
#define MINRANGE_0 20
#define STOPRANGE_0 5

/* 
 * internal state
 */
#define DELTA_STEER 5
#define DELTA_DELAY 10
#define RANGE_FACTOR 0.667


/* **** **** **** **** **** ****
 * Global variables
 * **** **** **** **** **** ****/

/* 
 * action
 */
Servo steerServo;

/* 
 * perception
 */
int cmLeft;
int cmRight;

/* 
 * internal state
 */
int steerAdjust;
int speedDelay;
int maxRange;
int minRange;
int stopRange;



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
	maxRange = MAXRANGE;
	minRange = MINRANGE;
	stopRange = STOPRANGE;
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
	Serial.begin(9600);
	/* initial action state */
	steerServo.write(STRAIGHT);
	digitalWrite(ENGINERELAY, LOW);
	digitalWrite(LEFTTRIGGER, LOW);
	digitalWrite(RIGHTTRIGGER, LOW);
}

/* 
 * reactive avoidance decision function
 *	perception x inner state -> action
 */
int getSteer() {
	if (cmLeft < stopRange || cmRight < stopRange)
		return STOP;
	if (cmLeft < minRange && cmRight < minRange)
		return (cmLeft > cmRight) ? LEFT : RIGHT;
	if (cmLeft < minRange)
		return RIGHT;
	if (cmRight < minRange)
		return LEFT;
	if (cmLeft > maxRange)
		cmLeft = maxRange;
	if (cmRight > maxRange)
		cmRight = maxRange;
	return STRAIGHT + ((cmLeft > cmRight) ? (maxRange - cmRight) : - (maxRange - cmLeft)) * (STRAIGHT - RIGHT) / maxRange;
}


/* 
 * control loop
 */
void loop() {
	int steer;
	/* perception */
	unsigned long echoDuration;
	digitalWrite(LEFTTRIGGER, HIGH);
	delayMicroseconds(10);
	digitalWrite(LEFTTRIGGER, LOW);
	echoDuration = pulseIn(INLEFTECHO, HIGH, 100000);
	cmLeft = echoDuration ? echoDuration / 60 : maxRange;
	digitalWrite(RIGHTTRIGGER, HIGH);
	delayMicroseconds(10);
	digitalWrite(RIGHTTRIGGER, LOW);
	echoDuration = pulseIn(INRIGHTECHO, HIGH, 100000);
	cmRight = echoDuration ? echoDuration / 60 : maxRange;
	/* inner state calibration */
	if (digitalRead(INLEFT) == HIGH)
		steerAdjust += DELTA_STEER;
  	if (digitalRead(INRIGHT) == HIGH)
		steerAdjust -= DELTA_STEER;
	if (digitalRead(INSTOP) == HIGH) {
		speedDelay += DELTA_DELAY;
		maxRange = max(MAXRANGE_0, MAXRANGE * RANGE_FACTOR);
		minRange = max(MINRANGE_0, MINRANGE * RANGE_FACTOR);
		stopRange = max(STOPRANGE_0, STOPRANGE * RANGE_FACTOR);
	}
	/* inner state */
	if (speedDelay != 0) {
		digitalWrite(ENGINERELAY, LOW);
		delay(speedDelay);
	}
	/* decision */
	steer = getSteer();
	/* communication */
	Serial.println(String(cmLeft) + " | " + String(cmRight) + " -> " + String(steer));
	/* action */
	if (steer == STOP) {
		digitalWrite(ENGINERELAY, LOW);
		steerServo.write(STRAIGHT+steerAdjust);
	}
  	else {
		digitalWrite(ENGINERELAY, HIGH);
		steerServo.write(steer+steerAdjust);
	}
	/* control loop frequency */
	delay(POLL);
}

