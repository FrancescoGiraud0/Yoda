#include <Servo.h>

#define echoPin 5 // Echo Pin
#define trigPin 7 // Trigger Pin
#define LEDPin 13 // Onboard LED

#define DirB 13
#define PwmB 11
#define ENPinB 8

#define PwmA 3
#define DirA 12
#define ENPinA 9

int pos = 0;  // variable to store the servo position
int n = 0;
int dl1 = 80;
byte pse = 0;
int pon = 0;

/*Velocita' motori*/
byte velMotori = 200;


void setup() {
  // set the digital pin as output:
  pinMode(PwmA, OUTPUT);
  pinMode(PwmB, OUTPUT);
  pinMode(ENPinA, OUTPUT);
  pinMode(ENPinB, OUTPUT);
  pinMode(DirA, OUTPUT);
  pinMode(DirB, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(ENPinA, LOW);
  digitalWrite(ENPinB, LOW);
  
  Serial.begin(9600);

}

int cmd;
int speed = 100;

void indietro() {
  analogWrite(PwmA, velMotori);
  analogWrite(PwmB, velMotori);
  digitalWrite(DirA, HIGH);
  digitalWrite(DirB, LOW);
  digitalWrite(ENPinA, HIGH);
  digitalWrite(ENPinB, HIGH);
  delay(10);
  return;
}

void avanti() {
  analogWrite(PwmA, velMotori);
  analogWrite(PwmB, velMotori);
  digitalWrite(DirA, LOW);
  digitalWrite(DirB, HIGH);
  digitalWrite(ENPinA, HIGH);
  digitalWrite(ENPinB, HIGH);
  delay(10);
  return;
}
void destra() {
  analogWrite(PwmA, velMotori);
  analogWrite(PwmB, velMotori);
  digitalWrite(DirA, HIGH);
  digitalWrite(DirB, HIGH);
  digitalWrite(ENPinA, HIGH);
  digitalWrite(ENPinB, HIGH);
  delay(10);
  //stops();
}
void sinistra() {
  analogWrite(PwmA, velMotori);
  analogWrite(PwmB, velMotori);
  digitalWrite(DirA, LOW);
  digitalWrite(DirB, LOW);
  digitalWrite(ENPinA, HIGH);
  digitalWrite(ENPinB, HIGH);
  delay(10);
  //stops();
}

void stops() {
  digitalWrite(ENPinA, LOW);
  digitalWrite(ENPinB, LOW);
  digitalWrite(PwmA, LOW);
  digitalWrite(PwmB, LOW);
  digitalWrite(DirA, LOW);
  digitalWrite(DirB, LOW);
  delay(10);
}

void loop(){
  digitalWrite(ENPinA, LOW);
  digitalWrite(ENPinB, LOW);

  if (Serial.available()) {
    cmd = Serial.read();
    if (cmd == 'x') {
      indietro();
    }
    if (cmd == 'w') {
      avanti();
    }
    if (cmd == 'd') {
      destra();
    }
    if (cmd == 'a') {
      sinistra();
    }
    if (cmd == 's') {
      stops();
    }
    if (cmd == 'L') {
      digitalWrite(LEDPin, LOW);
    }
    if (cmd == 'l') {
      digitalWrite(LEDPin, HIGH);
    }
    if (cmd == 'i') {
      Serial.print("DAGU - ");
      Serial.println(n);
      n++;
    }
  }
}
