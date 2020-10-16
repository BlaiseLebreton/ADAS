#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include "rs232.h"
#include "liaison.h"

#define BUF_SIZE 128
int cport_nr=24; /* /dev/ttyUSB0 */

// int main()
// {
//
//   Initialize();
//
//   int cmd = 0 , pwr = 0;
//   for(int j=0;j<20;j++)
//   {
//     cmd = rand()%100;
//     pwr = rand()%10;
//     SendData(cmd,pwr);
//   }
//   return 0;
// }


int Initialize() {
  int bdrate=57600; /* 9600 baud */
  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
  if(RS232_OpenComport(cport_nr, bdrate, mode, 0))
  {
    printf("Can not open comport\n");
    return 1;
  }
  return 0;
}

int SendData(int cmd, int pwr) {
  int i,n;
  char str_cmd[20], str_pwr[20];
  sprintf(str_cmd, "%d", cmd);
  sprintf(str_pwr, "%d", pwr);
  char str_send[2][BUF_SIZE]; // send data buffer
  unsigned char str_recv[BUF_SIZE]; // recv data buffer
  strcpy(str_send[0], str_cmd);
  strcpy(str_send[1], str_pwr);
  for(i=0;i<2;i++)
  {
    RS232_cputs(cport_nr, str_send[i]); // sends string on serial
    printf("Sent to Arduino: '%s'\n", str_send[i]);
    do{
      n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
      str_recv[n] = 0;   /* always put a "null" at the end of a string! */
      if (n > 0)
      printf("Received %i bytes: '%s'\n\n", n, (char *)str_recv);
      usleep(100);  /* waits for reply 100ms */
    }while(n == 0);
  }
  return 0;
}
