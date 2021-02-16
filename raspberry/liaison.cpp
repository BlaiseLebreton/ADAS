#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include "rs232.h"
#include "liaison.h"

#define DEBUG 1
#define BUF_SIZE 128

// int cport_nr=25; /* /dev/ttyACM1 */
int cport_nr=24; /* /dev/ttyACM0 */

// int main()
// {
//   Liaison_Initialize();
//   int cmd = 0 , pwr = 0;
//   for(int j=0;j<100;j++)
//   {
//     usleep(100000);
//     cmd = -90 +rand()%180;
//     pwr = 1300+rand()%600;
//     Liaison_SendData(cmd,pwr);
//   }
//   return 0;
// }


int Liaison_Initialize() {
  int bdrate=230400; /* 9600 baud */
  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
  if(RS232_OpenComport(cport_nr, bdrate, mode, 0))
  {
    printf("Can not open comport\n");
    return 1;
  }
  return 0;
}

char str_send[BUF_SIZE];
unsigned char str_recv[BUF_SIZE];
int Liaison_SendData(int cmd, int pwr) {
  sprintf(str_send, "%d_%d", cmd, pwr);

  if (strcmp(str_send,(char*)str_recv) != 0) { // Si commande différente ou dernière commande mal transmise (str_recv erronee)
    int n;
    RS232_cputs(cport_nr, str_send);
    if (DEBUG > 1)
      printf("Sent: '%s'\n", str_send);
    do{
      n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
      str_recv[n] = 0;
      if (n > 0 && DEBUG > 1)
        printf("Recv: '%s'\n\n", str_recv);
      usleep(100);  /* waits for reply 100ms */
    }while(n == 0);
  }
  return 0;
}
