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

int _cmd,_pwr;
int Liaison_SendData(int cmd, int pwr) {
  // if (strcmp(str_send,str_recv) != 0) { // Si commande différente ou dernière commande mal transmise
  if (cmd != _cmd || pwr != _pwr) { // Si commande différente
    int n;
    char str_send[BUF_SIZE];
    unsigned char str_recv[BUF_SIZE];
    sprintf(str_send, "%d_%d", cmd, pwr);
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
    _cmd = cmd;
    _pwr = pwr;
  }
  return 0;
}
// int SendData(int cmd, int pwr) {
//   int i,n;
//   char str_cmd[20], str_pwr[20];
//   sprintf(str_cmd, "%d", cmd);
//   sprintf(str_pwr, "%d", pwr);
//   char str_send[2][BUF_SIZE]; // send data buffer
//   unsigned char str_recv[BUF_SIZE]; // recv data buffer
//   strcpy(str_send[0], str_cmd);
//   strcpy(str_send[1], str_pwr);
//   for(i=0;i<2;i++)
//   {
//     RS232_cputs(cport_nr, str_send[i]); // sends string on serial
//     printf("Sent to Arduino: '%s'\n", str_send[i]);
//     do{
//       n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
//       str_recv[n] = 0;   /* always put a "null" at the end of a string! */
//       if (n > 0)
//       printf("Received %i bytes: '%s'\n\n", n, (char *)str_recv);
//       usleep(1000);  /* waits for reply 100ms */
//     }while(n == 0);
//   }
//   return 0;
// }
