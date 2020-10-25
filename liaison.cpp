#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include "rs232.h"
#include "liaison.h"

#define BUF_SIZE 128
// int cport_nr=25; /* /dev/ttyACM1 */
int cport_nr=24; /* /dev/ttyACM0 */

int main()
{

  Initialize();

  int cmd = 0 , pwr = 0;
  for(int j=0;j<100;j++)
  {
    usleep(100000);
    cmd = -90 +rand()%180;
    pwr = 1300+rand()%600;
    SendData(cmd,pwr);
  }
  return 0;
}


int Initialize() {
  int bdrate=230400; /* 9600 baud */
  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
  if(RS232_OpenComport(cport_nr, bdrate, mode, 0))
  {
    printf("Can not open comport\n");
    return 1;
  }
  return 0;
}

int SendData(int cmd, int pwr) {
  int n;
  char str_send[BUF_SIZE]; // send data buffer
  unsigned char str_recv[BUF_SIZE];
  sprintf(str_send, "%d_%d", cmd, pwr);
  RS232_cputs(cport_nr, str_send); // sends string on serial
  printf("Sent: '%s'\n", str_send);
  do{
    n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
    str_recv[n] = 0;   /* always put a "null" at the end of a string! */
    if (n > 0)
    printf("Recv: '%s'\n\n", (char *)str_recv);
    usleep(100);  /* waits for reply 100ms */
  }while(n == 0);
  return 0;
}
