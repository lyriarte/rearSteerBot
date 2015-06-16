#include <Servo.h> 

#define STEERSERVO 7
#define ENGINERELAY 13
#define INLEFT 10
#define INRIGHT 11
#define INSTOP 12
#define INLEFTECHO 2
#define LEFTTRIGGER 3
#define INRIGHTECHO 4
#define RIGHTTRIGGER 5

#define POLL 50

#define STOP 0
#define STRAIGHT 90
#define LEFT 135
#define RIGHT 45

#define MAXRANGE 200
#define MINRANGE  50
#define STOPRANGE 20
#define MAXRANGE_0 30
#define MINRANGE_0 10
#define STOPRANGE_0 5

Servo steerServo;
int cmLeft;
int cmRight;
int steerAdjust;
int speedDelay;
int maxRange;
int minRange;
int stopRange;

void setup() {
	steerAdjust = 0;
	speedDelay = 0; 
	maxRange = MAXRANGE;
	minRange = MINRANGE;
	stopRange = STOPRANGE;
	pinMode(STEERSERVO, OUTPUT);
	pinMode(INLEFT, INPUT);
	pinMode(INRIGHT, INPUT);
	pinMode(ENGINERELAY, OUTPUT);
	pinMode(INSTOP, INPUT);
	pinMode(INLEFTECHO, INPUT);
	pinMode(LEFTTRIGGER, OUTPUT);
	pinMode(INRIGHTECHO, INPUT);
	pinMode(RIGHTTRIGGER, OUTPUT);
	steerServo.attach(STEERSERVO);
	Serial.begin(9600);
	steerServo.write(STRAIGHT);
	digitalWrite(ENGINERELAY, LOW);
	digitalWrite(LEFTTRIGGER, LOW);
	digitalWrite(RIGHTTRIGGER, LOW);
}

int getSteer() {
	if (cmLeft < stopRange || cmRight < stopRange)
		return STOP;
	if (cmLeft < minRange && cmRight < minRange)
		return STOP;
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


void loop() {
	int steer;
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
	Serial.print(cmLeft);
	Serial.print(" : ");
	Serial.println(cmRight);
	if (digitalRead(INLEFT) == HIGH)
		steerAdjust += 5;
  	if (digitalRead(INRIGHT) == HIGH)
		steerAdjust -= 5;
	if (digitalRead(INSTOP) == HIGH) {
		speedDelay += 10;
		maxRange = max(MAXRANGE_0, MAXRANGE * 0.667);
		minRange = max(MINRANGE_0, MINRANGE * 0.667);
		stopRange = max(STOPRANGE_0, STOPRANGE * 0.667);
	}
	if (speedDelay != 0) {
		digitalWrite(ENGINERELAY, LOW);
		delay(speedDelay);
	}
	steer = getSteer();
	Serial.println(steer);
	if (steer == STOP) {
		digitalWrite(ENGINERELAY, LOW);
		steerServo.write(STRAIGHT+steerAdjust);
	}
  	else {
		digitalWrite(ENGINERELAY, HIGH);
		steerServo.write(steer+steerAdjust);
	}
	delay(POLL);
}

