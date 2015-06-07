#include <Servo.h> 

#define STEERSERVO 7
#define ENGINERELAY 13
#define INLEFT 10
#define INRIGHT 11
#define INRUN 12

#define STRAIGHT 90
#define LEFT 120
#define RIGHT 60


Servo steerServo;
int pollms;

void setup() {
	pinMode(STEERSERVO, OUTPUT);
	pinMode(INLEFT, INPUT);
	pinMode(INRIGHT, INPUT);
	pinMode(ENGINERELAY, OUTPUT);
	pinMode(INRUN, INPUT);
	steerServo.attach(STEERSERVO);
	pollms=50;
	steerServo.write(STRAIGHT);
	digitalWrite(ENGINERELAY, LOW);
}

void loop() {
  	if (digitalRead(INLEFT) == HIGH) {
		steerServo.write(LEFT);
	}
  	else if (digitalRead(INRIGHT) == HIGH) {
		steerServo.write(RIGHT);
	}
  	else {
		steerServo.write(STRAIGHT);
	}
  	if (digitalRead(INRUN) == HIGH) {
		digitalWrite(ENGINERELAY, HIGH);
	}
  	else {
		digitalWrite(ENGINERELAY, LOW);
	}
	delay(pollms);
}
