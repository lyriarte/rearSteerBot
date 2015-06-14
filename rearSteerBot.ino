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

#define MAXRANGE 150
#define MINRANGE  30
#define STOPRANGE 15

Servo steerServo;
int cmLeft;
int cmRight;
int steerAdjust;
int speedDelay;

void setup() {
	steerAdjust = 0;
	speedDelay = 0; 
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
	return STRAIGHT + ((cmLeft > cmRight) ? (MAXRANGE - cmRight) : - (MAXRANGE - cmLeft)) * (STRAIGHT - RIGHT) / MAXRANGE;
}


void loop() {
	int steer;
	unsigned long echoDuration;
	digitalWrite(LEFTTRIGGER, HIGH);
	digitalWrite(LEFTTRIGGER, LOW);
	echoDuration = pulseIn(INLEFTECHO, HIGH, 100000);
	cmLeft = echoDuration ? echoDuration / 60 : MAXRANGE;
	digitalWrite(RIGHTTRIGGER, HIGH);
	delayMicroseconds(10);
	digitalWrite(RIGHTTRIGGER, LOW);
	echoDuration = pulseIn(INRIGHTECHO, HIGH, 100000);
	cmRight = echoDuration ? echoDuration / 60 : MAXRANGE;
	Serial.print(cmLeft);
	Serial.print(" : ");
	Serial.println(cmRight);
	if (digitalRead(INLEFT) == HIGH)
		steerAdjust += 5;
  	if (digitalRead(INRIGHT) == HIGH)
		steerAdjust -= 5;
	if (digitalRead(INSTOP) == HIGH)
		speedDelay += 10;
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

