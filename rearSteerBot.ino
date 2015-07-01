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
#define ECHO_TIMEOUT 20000
#define ECHO2CM(x) (x/60) 

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
int cmLeft;
int cmRight;

/* 
 * internal state
 */
int steerAdjust;
int speedDelay;



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
	cmLeft = echoDuration ? ECHO2CM(echoDuration) : MAXRANGE;
	digitalWrite(RIGHTTRIGGER, HIGH);
	delayMicroseconds(10);
	digitalWrite(RIGHTTRIGGER, LOW);
	echoDuration = pulseIn(INRIGHTECHO, HIGH, ECHO_TIMEOUT);
	cmRight = echoDuration ? ECHO2CM(echoDuration) : MAXRANGE;
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
	return true;
}

/* 
 * control loop
 */
void loop() {
	/* control loop frequency */
	unsigned long timeLoopStart;
	unsigned long timeLoop;
	timeLoopStart = millis();
	/* perception */
	frontUltrasonicPerception();
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
	if (!avoidance())
		trajectory();
	/* communication */
	Serial.println("[" + String(millis()) + "] " + String(cmLeft) + " | " + String(cmRight) + " -> " + String(steer) + " , " + String(speed));
	/* action */
	digitalWrite(ENGINERELAY, speed ? HIGH : LOW);
	steerServo.write(steer+steerAdjust);
	/* control loop frequency */
	timeLoop = millis() - timeLoopStart;
	if (timeLoop < POLL)
		delay(POLL - timeLoop);
}

