/*Non-Canonical Input Processing*/

// Pedro Lin
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <strings.h>


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



volatile int STOP=FALSE;


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




    sent[0]= FLAG; // F
    sent[1]= A_EM; // A
    sent[2]= C_SET; // C
    sent[3]= BCC(A_EM, C_SET);//0x04;//BCC(A_EM, C_SET); // BCC
    sent[4]= FLAG; // F


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

 printf("%d bytes Escritos ou enviados\n\n\n", Times_Written);
    

 sleep(3); //envia mas precisa de esperar um bocado para receber ou instantaneo ?


unsigned char buffer_hex[2],Store_hex;

/*  State Machine of Reception Set Message*/
Times_Written=0;
int state=0;

while (STOP==FALSE)  // to last state machine
{ 
    read_resp=read(fd,buffer_hex,1); // read =1 sucess , <0 failure
    printf(" Receive state %i  , received_byte %x , \n\n",read_resp,buffer_hex[0]);  //HEXADECIMAL

    if(read_resp<0)  
        printf("Could not receive , please try again \n");

  if(read_resp==1)
        Times_Written++;

 

buffer_hex[1]='\0'; // ultimo espaco como \0
Store_hex=buffer_hex[0];


/* Agora State machine */
printf("\n\t\t STATE =  %i\n",state);

switch (state)
{
case Start:
    if(Store_hex==FLAG)
        {
        state=Flag_Rcv;

        }
    break;

case Flag_Rcv:
    if(Store_hex==A_EM)  // basicamente se nao for  A_RE ou flag volta ao inicio
        {
        state=A_Rcv;
        break;
        }
    
    if(Store_hex==FLAG)
        {
        break;
        }

    state=Start;



case A_Rcv:
            if (Store_hex== FLAG)
            {
                state = Flag_Rcv;
            }
            if (Store_hex ==C_UA)
            {
                state = C_Rcv;
            }
            else
            {
                state = Start;
            }
            break;


        case C_Rcv:

            if (Store_hex= FLAG)
            {
                state = Flag_Rcv;
            }
            if (Store_hex=BCC(A_RE,C_UA))
            {
                state = Bcc_Ok;
            }
            else
            {
                state = Start;
            }
            break;
        

    case Bcc_Ok:
        if(Store_hex==FLAG)
            {
                state=Stop_Final;
                STOP=TRUE;
            }



}




   printf("\t\t%d\n",Times_Written);
    if(Times_Written==5 && state !=Stop_Final)  
    {
        printf("Could not reach final state machine , stop\n\n");
        break;
    }




}

  
printf("\n\t\t FINAL STATE =  %i\n",state);



/*
    for (i = 0; i < 255; i++) {
        buf[i] = 'a';
    }

    /*testing*/
  /*  buf[25] = '\n';

    res = write(fd,buf,255);
    printf("%d bytes written\n", res);
*/

    /*
    O ciclo FOR e as instruções seguintes devem ser alterados de modo a respeitar
    o indicado no guião
    */


    if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }


    close(fd);
    return 0;
}
