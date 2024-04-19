/*Non-Canonical Input Processing*/
// WORK WITH THE write_string
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1


/*  State Machine */

#define Start      0
#define Flag_Rcv   1
#define A_Rcv      2
#define C_Rcv      3
#define Bcc_Ok     4
#define Stop_Final 5

/* 5 bytes   , Transition conditions */
#define SET_LENGHT  5 // tamanho do SET
#define FLAG  0x5c  // ( (0x5c)) 0101 1100 flag de inicio e fim
#define A_EM  0x03  // (0x03) 0000 0011Campo de Endereço (A) de commandos do Emissor, resposta do Receptor
#define A_RE  0x01  // (0x01)0000 0001 Campo de Endereço (A) de commandos do Receptor, resposta do Emissor

#define C_SET 0x08 // 0000 1000(0x08) Campo de Controlo - SET (set up)
#define C_UA  0x06 // 0000 0110(0x06) Campo de Controlo - UA (Unnumbered Acknowledgement)

#define BCC(a,c) (a ^ c) // fazer xor a c


volatile int STOP=FALSE;

int main(int argc, char** argv)
{
    int fd,c, res, i=0, state=Start;
    struct termios oldtio,newtio;
    unsigned char buf[255], printer[255], aux[2], x, aux1, set[5], z;

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

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 1 chars received */

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }
    
    for(int j=0;j<255;j++){
    printer[j]='\0';
    }

    printf("New termios structure set\n");

  
int Times_Written=0;
unsigned char receive[2];
char sent[255];
  // receber os 5 bytes

for(i=0 ; i<5;i++)
 {
    
    res= read(fd,receive,1); // retorna o n bytes escrito com sucesso , -1 erro
   // aux1=aux[0];
     receive[res]='\0';
     // aux[res]='\0';
    printf("Bytes receive %x in %d\n",receive[0],i);
    
    Times_Written+=res;
 }

   // res= write(fd,sent,sent_lenght); // retorna o n bytes escrito com sucesso , -1 erro

if (Times_Written!=5)  //caso nao envie algo byte
    return -1;

 printf("%d bytes Recebidos\n", Times_Written);
   // return 0;



sleep(5);






    sent[0]= FLAG; // F
    sent[1]= A_RE ;// A   
    sent[2]= C_UA; // C
    sent[3]= BCC(A_EM, C_SET); // BCC
    sent[4]= FLAG; // F


// enviar os 5 bytes

for(i=0 ; i<5;i++)
 {
    
    res= write(fd,&sent[i],1); // retorna o n bytes escrito com sucesso , -1 erro
    printf("Bytes written %x in %d\n",sent[i],i);
    Times_Written+=res;
   
 }

   // res= write(fd,sent,sent_lenght); // retorna o n bytes escrito com sucesso , -1 erro

if (Times_Written!=5)  //caso nao envie algo byte
    return -1;

 printf("%d bytes Escritos ou enviados\n", Times_Written);
    



    /*
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */

    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
