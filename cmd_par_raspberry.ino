#include <Servo.h>

char inData[2];
char inChar;
byte index = 0;

//code test
//definitions de structures
typedef struct{// recu
  int dir = 90;
  int pwr = 0;
}cmd;

typedef struct{
  Servo s;
  float k = 0.35;
  int off = 0;
  int ch;
}servo;

//structures
servo ServoDir;
servo ServoPwr;
cmd commande;

void setup() {

  Serial.begin(57600); // opens serial port, sets data rate to 57600 baud
  ServoDir.s.attach(2);// attacher le varaible de l'angle du servo-moteur à un pin
  ServoPwr.s.attach(3);
  ServoDir.s.write(0); // on place le servo à 0 degré au demarrage
  ServoPwr.s.write(0);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  ServoPwr.s.writeMicroseconds(1500);
}

void loop() {
  while (Serial.available() > 0) { // if any data available
    if (index < 2){
      char incomingByte = Serial.read(); // read byte
      inData[index]=incomingByte;
      index++;
      Serial.write(inData[index]); // send it back
    }
    index = 0;
    int r_cmd = min(max(-90,  inData[0]), 90);
    int r_pwr = min(max(1300, inData[1]), 1800);
    //ServoDir.ch = pulseIn(4,HIGH);
    //ServoPwr.ch = pulseIn(5,HIGH);


    ServoDir.s.write(ServoDir.k * r_cmd + ServoDir.off + 90);
    ServoPwr.s.writeMicroseconds(r_pwr);

  }

  //  ServoDir.s.write(ServoDir.k * -90 + ServoDir.off + 90); // On envoi la commande de l'angle au servo-moteur
  //  delay(1500);
  //  ServoDir.s.write(ServoDir.k * 90 + ServoDir.off + 90); // On envoi la commande de l'angle au servo-moteur
  //  delay(1500);
  //  ServoDir.s.write(ServoDir.k * 0 + ServoDir.off + 90); // On envoi la commande de l'angle au servo-moteur
  //  delay(100);
  //
  //  Serial.println("1500");
  //  ServoPwr.s.writeMicroseconds(1500);
  //  delay(2500);
  //  Serial.println("1300");
  //  ServoPwr.s.writeMicroseconds(1300); //arriere
  //  delay(2500);
  //  Serial.println("1500");
  //  ServoPwr.s.writeMicroseconds(1500);
  //  delay(2500);
  //  Serial.println("1700");
  //  ServoPwr.s.writeMicroseconds(1700); //avant
  //  delay(2500);
  //
  //  ServoDir.ch = pulseIn(4,HIGH);
  //  ServoPwr.ch = pulseIn(5,HIGH);
  //
  //  Serial.print(ServoDir.ch);
  //  Serial.print(",");
  //  Serial.println(ServoPwr.ch);
  //
  //  delay(100);
}
