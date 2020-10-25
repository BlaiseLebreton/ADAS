#include <Servo.h>
#include <string.h>

char inData[2];
char inChar;
byte index = 0;

//code test
//definitions de structures
typedef struct{ // recu
  int dir = 90;
  int pwr = 1500;
}commande;

typedef struct{
  Servo s;
  float k = 0.35;
  int off = 0;
  int ch;
}servo;

//structures
servo ServoDir;
servo ServoPwr;
commande cmd;

void setup() {

  Serial.begin(230400); // opens serial port, sets data rate to 57600 baud
  ServoDir.s.attach(2);// attacher le varaible de l'angle du servo-moteur à un pin
  ServoPwr.s.attach(3);
  ServoDir.s.write(0); // on place le servo à 0 degré au demarrage
  ServoPwr.s.write(0);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  ServoPwr.s.writeMicroseconds(1500);
}

String incomingByte;
char test[25];
void loop() {
  if (Serial.available() > 0) { // if any data available
    // incomingByte = char(Serial.read()); // read byte
    incomingByte = "";
    while (Serial.available()) {
      delay(2);  //delay to allow byte to arrive in input buffer
      char c = Serial.read();
      incomingByte += c;
    }
    // Serial.write(incomingByte); // send it back


    int passed=0,ci=0,pi=0;
    char sdir[5]; char spwr[5];
    for (int i = 0; i<incomingByte.length(); i++) {
    // for (int i = 0; i<strlen(incomingByte); i++) {
      if (incomingByte[i] == '_') {
        passed = 1;
      }
      else{
        if (passed == 0) {
          sdir[ci] = incomingByte[i];
          ci++;
        }
        else{
          spwr[pi] = incomingByte[i];
          pi++;
        }
      }
    }
    sdir[ci] = '\0';
    spwr[pi] = '\0';
    sprintf(test, "%s_%s", sdir, spwr);
    Serial.write(test); // send it back
  }
}
