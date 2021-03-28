#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include "rs232.h"
#include "liaison.h"

#define DEBUG 0
#define BUF_SIZE 128

int cport_nr=24; /* /dev/ttyACM0 */
char str_send[BUF_SIZE];

int Liaison_Initialize() {
  int bdrate=230400;
  char mode[]={'8','N','1',0};
  if (RS232_OpenComport(cport_nr, bdrate, mode, 0)) {
    printf("Can not open comport\n");
    return 1;
  }
  return 0;
}

int Liaison_SendData(int cmd, int pwr) {
  sprintf(str_send, "%d_%d", cmd, pwr);
  RS232_cputs(cport_nr, str_send);
  if (DEBUG > 1) {
    printf("Sent: '%s'\n", str_send);
  }
  return 0;
}
