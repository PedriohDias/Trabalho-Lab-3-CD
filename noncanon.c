/*Non-Canonical Input Processing*/

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
    int fd,c, res;
    int i = 0;
    int state = Start;
    struct termios oldtio,newtio;
    unsigned char buf[255];
    unsigned char print[255];

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
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */ /* mudei porque acho que faz sentido ele ler um a um em vez de ler os 5 de uma vez e haver sobreposição de informação */

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

for(int j=0;j<255;j++){
    print[j]='\0';
    } /* certificar que cada posição do array esta livre*/ 



// Machine State
unsigned char buffer_hex[2],Store_hex;
int read_resp=0;

/*int state=0; não é preciso, pois já defini no inicio da int main() state = start */

printf("\t%d\n\n",state);
while (STOP==FALSE)  // to last state machine
{ 
read_resp=read(fd,buffer_hex,1); // read =1 sucess , <0 failure
buffer_hex[read_resp]='\0';
printf(":%x:%d\n", buffer_hex[0], read_resp);
        print[i] = buffer_hex[0];
    /*    if (buffer_hex[0]== '\0' ) {
STOP=1;
}*/
        i++;
 buffer_hex[1]='\0'; // ultimo espaco como \0 --- Não sei se isto é preiso, eu acho qe nao, mas vou deixar estar porque não percebi o que tetaste fazer então pode estar bem 

        Store_hex=buffer_hex[0];
        

   /* printf(" Receive %x  try %d\n\n",buffer_hex[0],read_resp);  //HEXADECIMAL   -- Isto não e preciso acho eu portanto apaguei no meu documento */

    
    if(read_resp<0)  {
        printf("Could not receive , please try again \n"); 
}


/* Agora State machine */


//printf("%d : %x  :%x \n",state,Store_hex,buffer_hex[0]);

unsigned char x;

switch (state)
{
case Start:
                if(Store_hex == FLAG){
                     state = Flag_Rcv;
                }
                break;
            
            case Flag_Rcv:
                if(Store_hex  == A_EM) {
                    state = A_Rcv;
                }
                else if(Store_hex == FLAG){
                    //continue;
break;                 
                }
                else{
                    state = Start;
                }
                break;

            case A_Rcv:
                if(Store_hex == FLAG){
                    state = Flag_Rcv;
                }
                else if(Store_hex = C_SET){
                    state = C_Rcv;
                }
                else{
                    state = Start;
                }
                break;
            case C_Rcv:

                x = A_EM^C_SET;

                if(Store_hex == FLAG){
                    state = Flag_Rcv;
                }
                else if(Store_hex == x){
                    state = Bcc_Ok;
                }
                else{
                    state = Start;
                }
                break;
            case Bcc_Ok:
                if(Store_hex == FLAG){
                    state = Stop_Final; 
                    STOP=TRUE;
                }
                else{
                    state = Start;
                }
                break;
                }
printf("\n\t\t STATE =  %i\n",state);
}
for (i=0;i<5;i++){
            printf("\n%x\n",print[i]);
        }

sleep(2);

unsigned char w;

    set[0]=FLAG;
    set[1]=0x09;
    set[2]=0x06;             
    set[3]=set[1]^set[2];
    set[4]=set[0];
        
    for(int n=0;n<5;n++){
        w=set[n];
        res = write(fd, &w, 1);
                    
    }

    /*
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */

    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;

}

