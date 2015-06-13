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

#define MAXRANGE 100
#define MINRANGE  25
#define STOPRANGE 05

Servo steerServo;
int cmLeft;
int cmRight;

void setup() {
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
	if (digitalRead(INLEFT) == HIGH)
		return LEFT;
  	if (digitalRead(INRIGHT) == HIGH)
		return RIGHT;
	if (digitalRead(INSTOP) == HIGH)
		return STOP;
	if (cmLeft < STOPRANGE || cmRight < STOPRANGE)
		return STOP;
	if (cmLeft < MINRANGE && cmRight < MINRANGE)
		return STOP;
	if (cmLeft < MINRANGE)
		return RIGHT;
	if (cmRight < MINRANGE)
		return LEFT;
	if (cmLeft > MAXRANGE)
		cmLeft = MAXRANGE;
	if (cmRight > MAXRANGE)
		cmRight = MAXRANGE;
	return STRAIGHT + (cmLeft - cmRight) * (STRAIGHT - RIGHT) / MAXRANGE;
}


void loop() {
	int steer;
	unsigned long echoDuration;
	digitalWrite(LEFTTRIGGER, HIGH);
	digitalWrite(LEFTTRIGGER, LOW);
	echoDuration = pulseIn(INLEFTECHO, HIGH, 100000);
	cmLeft = echoDuration / 60;
	digitalWrite(RIGHTTRIGGER, HIGH);
	delayMicroseconds(10);
	digitalWrite(RIGHTTRIGGER, LOW);
	echoDuration = pulseIn(INRIGHTECHO, HIGH, 100000);
	cmRight = echoDuration / 60;
	Serial.print(cmLeft);
	Serial.print(" : ");
	Serial.println(cmRight);
	steer = getSteer();
	Serial.println(steer);
	if (steer == STOP) {
		digitalWrite(ENGINERELAY, LOW);
		steerServo.write(STRAIGHT);
	}
  	else {
		digitalWrite(ENGINERELAY, HIGH);
		steerServo.write(steer);
	}
	delay(POLL);
}

