#include <Servo.h>
#include <string.h>

char inData[2];
char inChar;
int lastdir = 90;
byte index = 0;

//code test
//definitions de structures
typedef struct{ // recu
  int dir = 90;
  int pwr = 1500;
}commande;

typedef struct{
  Servo s;
  float k = 0.3;
  int off = 90;
}servo;

//structures
servo ServoDir;
Servo ServoPwr;
commande rasp;
commande recv;

void setup() {
  Serial.begin(230400); // opens serial port, sets data rate to 57600 baud
  pinMode(4, INPUT);
  pinMode(5, INPUT);

  // Initialisation commandes
  ServoDir.s.attach(2);// attacher le varaible de l'angle du servo-moteur à un pin
  ServoPwr.attach(3);
  delay(1000);
  ServoDir.s.write(0);
  ServoPwr.writeMicroseconds(1500);
  delay(1000);
  ServoPwr.writeMicroseconds(1500);
  ServoDir.s.write(0);
}

String incomingByte;
char test[25];
int i;
int mycmd, moy, sum;

void loop() {

  // Commande receiver
  recv.pwr = 0;
  recv.dir = 0;
  for (i = 1; i < 4; i++) {
    recv.pwr += pulseIn(4, HIGH);
    recv.dir += pulseIn(5, HIGH);
  }
  recv.pwr = recv.pwr/(i-1);
  recv.dir = map(recv.dir/(i-1), 1000, 2000, -90, 90);



  if (Serial.available()) { 
    incomingByte = "";
    while (Serial.available()) {
      delay(2);
      char c = (char)Serial.read();
      incomingByte += c;
    }

    // Decoupage du message en deux (direction_power)
    int passed=0,ci=0,pi=0;
    char sdir[5]; char spwr[5];
    for (int i = 0; i<incomingByte.length(); i++) {
      if (incomingByte[i] == '_') {
        passed = 1;
      }
      else {
        if (passed == 0) {
          sdir[ci] = incomingByte[i];
          ci++;
        }
        else {
          spwr[pi] = incomingByte[i];
          pi++;
        }
      }
    }
    sdir[ci] = '\0';
    spwr[pi] = '\0';

    // // Validation reception
    // sprintf(test, "%s_%s", sdir, spwr);
    // Serial.write(test);

    // Commande recue par la raspberry
    rasp.dir = max(-90,  min(-atoi(sdir), 90));
    rasp.pwr = max(1500, min( atoi(spwr), 1700));
//    
//    for (i = 0; i < 1; i++) {
//      Dir[i] = Dir[i+1];
//    }
//    Dir[i] = rasp.dir;
//    for (i = 0; i < 2; i++) {
//      rasp.dir += Dir[i];
//    }
//    rasp.dir = rasp.dir/(i-1);

     rasp.dir = lastdir + max(-30, min((rasp.dir - lastdir), 30));
     
     lastdir = rasp.dir;
    
  }






  // Application direction si pas d'outrepassement receiver
  if (abs(recv.dir) < 45) {
    ServoDir.s.write(ServoDir.k*rasp.dir+ServoDir.off);
  }
  else {
    ServoDir.s.write(ServoDir.k*recv.dir+ServoDir.off);
  }

  // Application de la commande si pas d'outrepassement receiver
  if (recv.pwr < 1499) {
    ServoPwr.writeMicroseconds(rasp.pwr);
  }
  else {
    // ServoPwr.writeMicroseconds(1500); // Emergency stop
    ServoPwr.writeMicroseconds(recv.pwr);
  }

}

// Commande raspberry
void serialEvent() {



}
