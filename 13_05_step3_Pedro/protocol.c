/*Non-Canonical Input Processing*/
// baseado no writenoncanonical
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "linklayer.h"


#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1


/* 5 bytes   , Transition conditions */
#define SET_LENGHT  5 // tamanho do SET
#define FLAG  0x5c  // ( (0x5c)) 0101 1100 flag de inicio e fim
#define A_EM  0x03  // (0x03) 0000 0011Campo de Endereço (A) de commandos do Emissor, resposta do Receptor
#define A_RE  0x01  // (0x01)0000 0001 Campo de Endereço (A) de commandos do Receptor, resposta do Emissor

#define C_SET 0x08 // 0000 1000(0x08) Campo de Controlo - SET (set up)
#define C_UA  0x06 // 0000 0110(0x06) Campo de Controlo - UA (Unnumbered Acknowledgement)

#define BCC(a,c) (a ^ c) // fazer xor a c




/*  State Machine */

#define Start      0
#define Flag_Rcv   1
#define A_Rcv      2
#define C_Rcv      3
#define Bcc_Ok     4
#define Stop_Final 5




int main(int argc, char** argv)
{
    

     int fd,c, res,Times_Written ,read_resp=0,write_resp=0;
    struct termios oldtio,newtio;
    char buf[255],sent[255];
    int i, sum = 0, speed = 0;

    size_t buff_lenght=sizeof(buf)/sizeof(buf[0]);
    size_t sent_lenght=sizeof(sent)/sizeof(sent[0]);


// 255 maior numero de 8 bits , 2letras hex=8bits

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */




    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

  


// faz o llopen , guardando parametros de conectian serial
     int open_result,write_result;

    linkLayer struct1;


    struct1.role=1;
    struct1.baudRate=BAUDRATE ;
    struct1.numTries=0;
    struct11.timeout=0;


    open_result=llopen(open_result); // 1 sucesso , outro falha
    write_result=llwrite();
    llread();
    llclose();


}
