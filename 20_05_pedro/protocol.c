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




//Frame Specification & Header Delimitation 

#define Frame_Head 0x05c
#define Frame_Adress 0x01  // sent transmiter ,sender 0x03


int fd;
int llopen(linkLayer connectionParameters)
    { //role 0 transmitter 
    
      struct termios newtio;
      int  Times_Written,i;
       char sent[255];
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 5;   /* blocking read until 5 chars received */


    if(connectionParameters.role==TRANSMITTER)
{ 
    fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY);
    if (fd < 0) { printf("\nErro ao open\n "); }

  // comeca a enviar 5 primeirosbytes

  
    sent[0]= FLAG; // F
    sent[1]= A_EM; // A
    sent[2]=  C_SET; // C
    sent[3]= BCC(A_EM, C_SET);//0x04;//BCC(A_EM, C_SET); // BCC
    sent[4]= FLAG; // F


// enviar os 5 bytes
int res;
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



// emissor state machine





unsigned char buffer_hex[2],Store_hex;

/*  State Machine of Reception Set Message*/
Times_Written=0;
int state=0,read_resp;

while (STOP==FALSE)  // to last state machine
{ 
    read_resp=read(fd,buffer_hex,1); // read =1 sucess , <0 failure
    printf(" Receive state %i  , received_byte %x , \n\n",read_resp,buffer_hex[0]);  //HEXADECIMAL

    if(read_resp<0)  
        printf("Could not receive , please try again \n");

 // if(read_resp==1)
  //      Times_Written++;

 

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
        
        }
    
    else if(Store_hex==FLAG)
        {
        continue;
        }

    else
     state=Start;

    break;

case A_Rcv:
            if (Store_hex== FLAG)
            {
                state = Flag_Rcv;
            }
            else if (Store_hex ==C_UA)
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
            else if (Store_hex==BCC(A_EM,C_UA))
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
        else
            state=Start;

        break;

}




}

  
printf("\n\t\t FINAL STATE =  %i\n",state);








}



// now return the sucess;
return 1;



   
   
   
    }



int llwrite(char* buf, int bufSize)

{
    unsigned char controlfield;
    unsigned char frame[4 + bufSize + 4]; // inicio and final
    int i=0;

    frame[0] = Frame_Head;
    frame[1] = Frame_Adress;
    frame[2] = controlfield;
    frame[3] = BCC(frame[1],frame[2]);


  for ( i = 0; i < bufSize; i++) {
        frame[4 + i] = buf[i];
                                  }

  unsigned char BCC2;
  // ultima parte do frame
  frame[4+bufSize]=BCC2;
  frame[4+bufSize+1]=Frame_Head;


      // Enviar o frame
    int Frame_Sent;
    Frame_Sent=write(fd,frame,sizeof(frame));

    if(Frame_Sent==-1)
      {
        printf("\nErro na frame\n");
        return -1;

      }

    // verificar a frame enviarda
    for(i=0 ; i <sizeof(bufSize);i++)
    {
        printf("%x",frame[i]);

    }
    
    printf("\n Final\n");


    // segunda parte , de receber o RR e do REJ do receiver

unsigned char Control_Response[5];
int Bytes_Recebidos;
int Bytes_Esperados;
int Response_Control;
Bytes_Recebidos=0;
Bytes_Esperados=sizeof(Control_Response);

//garantir receber os 5 bytes
while(Bytes_Recebidos<Bytes_Esperados)
{
    Response_Control=read(fd,Control_Response+Bytes_Recebidos,Bytes_Esperados-Bytes_Recebidos);

    if(Response_Control == -1)
        {   
            printf("\nErro no control\n");
            return -1;
        }
     if(Response_Control == 0)
     {
        printf("\nFinal control,fim de file\n");


     }   

    //proximo ciclo
    Bytes_Recebidos=Response_Control;

}


int llclose(linkLayer connectionParameters, int showStatistics)
{
    unsigned char Disc_Frame[10];
    int Bytes_Write;
    if(connectionParameters.role == TRANSMITTER)
    {
        Bytes_Write=write(fd,Disc_Frame,sizeof(Disc_Frame));






    }

}


int llread(char* packet)
{


}









}
