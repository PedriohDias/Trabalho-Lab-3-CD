#ifndef LINKLAYER
#define LINKLAYER

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

typedef struct linkLayer{
    char serialPort[50];
    int role; //defines the role of the program: 0==Transmitter, 1=Receiver
    int baudRate;
    int numTries;
    int timeOut;
} linkLayer;

//ROLE
#define NOT_DEFINED -1
#define TRANSMITTER 0
#define RECEIVER 1


//SIZE of maximum acceptable payload; maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 1000

//CONNECTION deafault values
#define BAUDRATE_DEFAULT B38400
#define MAX_RETRANSMISSIONS_DEFAULT 3
#define TIMEOUT_DEFAULT 4
#define _POSIX_SOURCE 1 /* POSIX compliant source */

//MISC
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





struct Parameters{
off_t filesize; // tamanho fo file ,bytes
char filename[255]; // nome ficehiro
int type; // TRANSMITTER | RECEIVER
int gate; // /dev/ttySx | gate is x
};


static struct Paramenters parameter; 

// Opens a conection using the "port" parameters defined in struct linkLayer, returns "-1" on error and "1" on sucess
int llopen(linkLayer connectionParameters);
// Sends data in buf with size bufSize
int llwrite(char* buf, int bufSize);
// Receive data in packet
int llread(char* packet);
// Closes previously opened connection; if showStatistics==TRUE, link layer should print statistics in the console on close
int llclose(int showStatistics);

#endif

void Argumentos(int argc, char** argv)
{

      if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }

    parameter.filename=argv[1];

}

int llopen(linkLayer connectionParameters)
{   int fd;

      fd = open(parameter.filename, O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

  

    struct termios oldtio,newtio;


      bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 5;   /* blocking read until 5 chars received */

  

    return 1;
   /* printf("New termios structure set\n");*/

}

int llwrite(char* buf, int bufSize)
{
    int fd,i,res;
    fd = open(parameter.filename, O_RDWR | O_NOCTTY );

    buf[0]= FLAG; // F
    buf[1]= A_EM; // A
    buf[2]=  C_SET; // C
    buf[3]= BCC(A_EM, C_SET);//0x04;//BCC(A_EM, C_SET); // BCC
    buf[4]= FLAG; // F

int Times_Written=0;

// enviar os 5 bytes

for(i=0 ; i<5;i++)
 {
    
    res= write(fd,&sent[i],1); // retorna o n bytes escrito com sucesso , -1 erro
    printf("Bytes written %d in %d  , writted %x\n",res,i,sent[i]);
    Times_Written+=res;
 }

   // res= write(fd,sent,sent_lenght); // retorna o n bytes escrito com sucesso , -1 erro

if (Times_Written!=5)  //caso nao envie algo byte
    return -1;

 printf("%d bytes Escritos ou enviados iniciais\n\n\n", Times_Written);
    



}
