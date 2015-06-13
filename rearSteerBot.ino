#include <Servo.h> 

#define STEERSERVO 7
#define ENGINERELAY 13
#define INLEFT 10
#define INRIGHT 11
#define INRUN 12
#define INLEFTECHO 2
#define LEFTTRIGGER 3
#define INRIGHTECHO 4
#define RIGHTTRIGGER 5

#define POLL 50

#define STOP 0
#define STRAIGHT 90
#define LEFT 120
#define RIGHT 60

Servo steerServo;

void setup() {
	pinMode(STEERSERVO, OUTPUT);
	pinMode(INLEFT, INPUT);
	pinMode(INRIGHT, INPUT);
	pinMode(ENGINERELAY, OUTPUT);
	pinMode(INRUN, INPUT);
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
	return STRAIGHT;
}


void loop() {
	unsigned long echoDuration;
	digitalWrite(LEFTTRIGGER, HIGH);
	digitalWrite(LEFTTRIGGER, LOW);
	Serial.print("Reading... ");
	echoDuration = pulseIn(INLEFTECHO, HIGH, 100000);
	Serial.print(echoDuration / 60);
	digitalWrite(RIGHTTRIGGER, HIGH);
	delayMicroseconds(10);
	digitalWrite(RIGHTTRIGGER, LOW);
	Serial.print("... ");
	echoDuration = pulseIn(INRIGHTECHO, HIGH, 100000);
	Serial.println(echoDuration / 60);
	steerServo.write(getSteer());
	if (digitalRead(INRUN) == HIGH) {
		digitalWrite(ENGINERELAY, HIGH);
	}
  	else {
		digitalWrite(ENGINERELAY, LOW);
	}
	delay(POLL);
}

