/*Non-Canonical Input Processing*/
//sudo socat -d -d PTY,link=/dev/ttyS10,mode=777 PTY,link=/dev/ttyS11,mode=777
//sudo socat -d -d PTY,link=/dev/ttyS0,mode=777 PTY,link=/dev/ttyS1,mode=777

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1




/* 5 bytes   , Transition conditions */
#define SET_LENGHT  5 // tamanho do SET
#define FLAG  0x5c  // ( (0x5c)) 0101 1100 flag de inicio e fim
#define A_EM  0x3//0x03  // (0x03) 0000 0011Campo de Endereço (A) de commandos do Emissor, resposta do Receptor
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




volatile int STOP=FALSE;

int main(int argc, char** argv)
{
    int fd,c, res;
    struct termios oldtio,newtio;
    char buf[255];

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
    newtio.c_cc[VMIN]     = 5;   /* blocking read until 5 chars received */

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");


// Machine State
unsigned char buffer_hex[2],Store_hex;
int read_resp=0;
int  Times_Written=0;

int state=0;

printf("\t%d\n\n",state);
while (STOP==FALSE)  // to last state machine
{ 
    read_resp=read(fd,buffer_hex,1); // read =1 sucess , <0 failure
    printf(" Receive %x  try %d\n\n",buffer_hex[0],read_resp);  //HEXADECIMAL

    
    if(read_resp<0)  
        printf("Could not receive , please try again \n");

    if(read_resp==1)
        Times_Written++;

 

    if(buffer_hex[0]=='\0')
        STOP=1;
    

buffer_hex[1]='\0'; // ultimo espaco como \0
Store_hex=buffer_hex[0];




/* Agora State machine */

printf("\n\t\t   STATE =  %i\n",state);

//printf("%d : %x  :%x \n",state,Store_hex,buffer_hex[0]);
switch (state)
{
case Start:
    if(Store_hex==FLAG)
        {
        state=Flag_Rcv;

        }
    break;

case Flag_Rcv:
     if(Store_hex== A_EM){
                    state = A_Rcv;
                }
     else if(Store_hex == Flag_Rcv){
                    continue;                    
                }
                else{
                    state = Start;
                }
                break;


case A_Rcv:
            if (Store_hex== FLAG)
            {
                state = Flag_Rcv;

            }
            if (Store_hex == C_SET)
            {
                state = C_Rcv;

            }
            else
            {
                state = Start;

            }
            break;


    case C_Rcv:

            if (Store_hex== FLAG)
            {
                state = Flag_Rcv;

            }
            if (Store_hex==BCC(A_EM,C_SET))
           {
//printf("erro?\n");
                state = Bcc_Ok;
      
            }
            else
            {
//printf("ok\n");
//printf("%d",state);
                state = Start;

            }
            break;
        

    case Bcc_Ok:
        if(Store_hex==FLAG)
            {
                state=Stop_Final;
                STOP=TRUE;

            }
        else
        {
            state=Start;
        }
    break;


}

//   printf("\t\t%d\n",Times_Written);
    if(Times_Written==5 && state !=Stop_Final)  
    {
        printf("Could not reach final state machine , stop\n\n");
        break;
    }

}
printf("\n\t\t FINAL STATE =  %i\n",state);

sleep(4);
//printf("ola\n");

// segunda parte inicio
int i=0; 
Times_Written=0;
char sent[255];


// Now we sent 5 bytes ,C and BCC different 

sent[0] = FLAG;
sent[1] = A_EM;
sent[2] = 0XA4;//C_UA;
sent[3] = BCC(A_EM , C_UA);
sent[4] = FLAG;



for (i=0; i<5; i++)
{
res=write(fd,&sent[i],1);
 printf("Bytes written %x in %d\n",sent[i],i);
    Times_Written+=res;


}

if (Times_Written!=5)  //caso nao envie algo byte
    return -1;


 printf("%d bytes Escritos ou enviados\n", Times_Written);


// 



// segunda parte fim



    /*
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */

    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
