#include <Servo.h>

#define echoPin 5 // Echo Pin
#define trigPin 7 // Trigger Pin
#define LEDPin 13 // Onboard LED

/*Motor B*/
#define DirB 13 //direction
#define PwmB 11 //PWM
#define ENPinB 8  //!brake

/*Motor A*/
#define DirA 12 //direction
#define PwmA 3  //PWM
#define ENPinA 9  //!brake

int  pos = 0;  //variable to store the servo position
int  n = 0;
int  dl1 = 80;
byte pse = 0;
int pon = 0;

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
  Serial.println("bk\n");
  digitalWrite(PwmA, HIGH);
  digitalWrite(PwmB, HIGH);
  digitalWrite(DirA, HIGH);
  digitalWrite(DirB, LOW);
  digitalWrite(ENPinA, HIGH);
  digitalWrite(ENPinB, HIGH);
  delay(100);
  return;
}

void avanti() {
  Serial.println("av\n");
  digitalWrite(PwmA, HIGH);
  digitalWrite(PwmB, HIGH);
  digitalWrite(DirA, LOW);
  digitalWrite(DirB, HIGH);
  digitalWrite(ENPinA, HIGH);
  digitalWrite(ENPinB, HIGH);
  delay(100);
  return;
}

void destra() {
  Serial.println("dx\n");
  digitalWrite(PwmA, HIGH);
  digitalWrite(PwmB, HIGH);
  digitalWrite(DirA, HIGH);
  digitalWrite(DirB, HIGH);
  digitalWrite(ENPinA, HIGH);
  digitalWrite(ENPinB, HIGH);
  delay(100);
  return;
}

void sinistra() {
  Serial.println("sx\n");
  digitalWrite(PwmA, HIGH);
  digitalWrite(PwmB, HIGH);
  digitalWrite(DirA, LOW);
  digitalWrite(DirB, LOW);
  digitalWrite(ENPinA, HIGH);
  digitalWrite(ENPinB, HIGH);
  delay(100);
  return;
}

void stops() {
  Serial.println("stop\n");
  digitalWrite(ENPinA, LOW);
  digitalWrite(ENPinB, LOW);
  digitalWrite(PwmA, LOW);
  digitalWrite(PwmB, LOW);
  digitalWrite(DirA, LOW);
  digitalWrite(DirB, LOW);
  delay(100);
  return;
}

void loop(){
  digitalWrite(ENPinA, LOW);
  digitalWrite(ENPinB, LOW);

  if (Serial.available()){
    
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

