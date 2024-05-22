#include "linklayer.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS4"
#define _POSIX_SOURCE 1 /* POSIX compliant source */

#define FALSE 0
#define TRUE 1

//variaveis globais 
volatile int STOP = FALSE;
int tries = 0, fd, state = Start, res;
struct termios oldtio, newtio;
unsigned char ua[5], aux, x, a, c;
unsigned char buffer_hex[2],Store_hex;


//Estados
#define Start 0
#define Flag_Rcv 1
#define A_Rcv 2
#define C_Rcv 3
#define Bcc_Ok 4
#define Stop_1 5
#define INFO 6
#define Stop_Finall 7

//Flags
#define FLAG 0x5c
#define A_EM  0x03  
#define A_RE  0x01  
#define C_SET 0x08 
#define C_UA  0x06 
#define SET_LENGHT  5 // tamanho do SET
#define BCC(a,c) (a ^ c) // fazer xor a c

#define C_DISC 0x10
#define C_I0 0x80
#define C_I1 0xC0
#define C_R0 0x01
#define C_R1 0x11
#define C_RJ0 0x05
#define C_RJ1 0x15



// Flags controlo
#define SET 0
#define UA 1
#define DISC 2
#define RR 3

int write_func(unsigned char *vet, int tamanho)
{
    int i=0, total=0;
    int res;

     unsigned char aux;

    for (i = 0;i< tamanho;i++)
    {
        aux = vet[i];
        res = write(fd, &aux, 1);
        total += res;
    }

    if (total !=tamanho)
        return -1;

    printf("%d bytes written\n", total);
    return 1;
}

void state_machine_control(int control)
{
    //int a, c;
    
    //int res;
    int i = 0;
    //int state = Start;
    //unsigned char buffer_hex[2],Store_hex;
int read_resp=0;

//unsigned char x;

unsigned char buf[255];
    unsigned char print[255];
    
    if (Store_hex == C_I0)
        c = C_I0;
    else if (Store_hex == C_I1)
        c = C_I1;
    else if (Store_hex == C_I1)
        c = C_I1;


    if (control == SET)
    {
        a = A_EM;
        c = C_SET;
    }

    else if (control == UA)
    {
        a = A_RE;
        c = C_UA;
    }

    else if (control == DISC)
    {
        a = A_EM;
        c = C_DISC;
    }

    else if (control == RR)
    {
        a = A_EM;
        //c = C_I1;
    }

    switch (state)
    {
    case Start:
               if(Store_hex == FLAG){
                     state = Flag_Rcv;
                }
                break;
            
            
            case Flag_Rcv:
                if(Store_hex  == a) {
                    state = A_Rcv;
                }
                else if(Store_hex == FLAG){
            state=Flag_Rcv; 
                                   
                }
                else{
                    state = Start;
                }
                break;

            case A_Rcv:
              if(Store_hex == FLAG){
                    state = Flag_Rcv;
                }
                else if(Store_hex = c){
                    state = C_Rcv;
                }
                else{
                    state = Start;
                }
                break;

            case C_Rcv:

               x = a^c;

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
                    state = Stop_1; 
                    STOP=TRUE;
                }
                else{
                    state = Start;
                }
                break; 
            }
}

void state_machine_data()
{

    if (Store_hex == C_I0)
        c = C_I0;
    else if (Store_hex == C_I1)
        c = C_I1;

    switch (state)
    {

    case Start:
        if (Store_hex == FLAG)
        {
            state = Flag_Rcv;
        }
        break;

    case Flag_Rcv:
        if (Store_hex == A_EM)
        {
            state = A_Rcv;
        }
        else if (Store_hex == FLAG)
        {
        	state=Flag_Rcv;
        }
        else
        {
            state = Start;
        }
        break;

    case A_Rcv:
        if (Store_hex == FLAG)
        {
            state = Flag_Rcv;
        }
        if (Store_hex == c)
        {
            state = C_Rcv;
        }
        else
        {
            state = Start;
        }
        break;

    case C_Rcv:

        x = A_EM ^ c;
        if (Store_hex == FLAG)
        {
            state = Flag_Rcv;
        }
        if (Store_hex == x)
        {
            state = Bcc_Ok;
        }
        else
        {
            state = Start;
        }
        break;

    case Bcc_Ok:

         state = INFO;
         break;
    
    case INFO:

        if (Store_hex == FLAG)
        {
            state = Stop_Finall;
            STOP = TRUE;
        }
        else {
            state = Start;
            }
            
        break;
    }
}

int llopen(linkLayer connectionParameters)
{
    unsigned char set[5];
    int i = 0;

    ua[0] = FLAG;
    ua[1] = A_RE;
    ua[2] = C_UA;
    ua[3] = A_RE ^ C_UA;
    ua[4] = FLAG;

    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = connectionParameters.timeOut * 10;
    newtio.c_cc[VMIN] = 0;

    if (connectionParameters.role == 1)
    {
        newtio.c_cc[VTIME] = connectionParameters.timeOut * 30;
        newtio.c_cc[VMIN] = 0;
    }

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

   

    if (connectionParameters.role == 0)
    {
        STOP = FALSE;
        tries = 0;
        state = Start;

        set[0] = FLAG;
        set[1] = A_EM;
        set[2] = C_SET;
        set[3] = A_EM ^ C_SET;
        set[4] = FLAG;

        if (write_func(set, 5) == -1)
            return -1;


        while (STOP == FALSE)
        {
            res = read(fd, &Store_hex, 1);

            if ((res <= 0) && (state == 0)) 
            {
                if (tries < connectionParameters.numTries-1)
                {
                    if (write_func(set, 5) == -1)
                        return -1;
                }

                else
                {
                    printf("Nao conecta ao recetor\n");
                    return -1;
                }

                tries++;
            }

            printf("0x%02x\n", (unsigned int)(Store_hex & 0xFF));

            state_machine_control(UA); /* Verifies that the UA message was received correctly*/
            printf("State: %i\n", state);
        }

        printf("UA com successo\n");

        return 1;
    }

    if (connectionParameters.role == 1)
    {
        while (STOP == FALSE)
        {
            res = read(fd, &Store_hex, 1); 

            if (res <= 0)
            {
                printf("Timeout on RCV\n\n");
                return -1;
            }

            state_machine_control(SET);
        }
        
        printf("SET com sucesso\n");

        if (write_func(ua, 5) == -1)
        {
            printf("Erro na escritura UA\n");
            return -1;
        }
    

        return 1;
    }

    return -1;
}


int llwrite(char *buf, int bufSize)
{
    unsigned char escrita[(MAX_PAYLOAD_SIZE)*2 + 1], bcc2, buf1[bufSize + 1], append[4];
    int i = 0;
    size_t escrita_len;
    tries = 0;
    STOP = FALSE;
    state = Start;

    append[0] = FLAG;
    append[1] = A_EM;
    append[2] = C_I0;
    if (ctrl_val % 2)
        append[2] = C_I1;
    append[3] = A_EM ^ C_I0;

    printf("valor do controlo: 0x%02x\n", (unsigned int)(append[2] & 0xFF));

    bcc2 = buf[0] ^ buf[1];

    for (i = 2; i < bufSize; i++)
    {
        bcc2 = bcc2 ^ buf[i];
    }

    buf1[bufSize] = bcc2;

    for (i = 0; i < bufSize; i++)
    {
        buf1[i] = buf[i];
    }

    escrita_len = strlen(escrita);

    if (write_func(append, 4) == -1) {
        return -1;
    }

    if (write_func(escrita, escrita_len) == -1) {
        return -1;

    }
    

    if (write_func(append, 1) == -1) {

        return -1;
    }
    

    while (STOP == FALSE)
    {
        res = read(fd, &Store_hex, 1); 

        if ((res <= 0) && (state == 0))
        {
            if (tries < 3)
            {
                if (write_func(append, 4) == -1)
                    return -1;

                if (write_func(escrita, escrita_len) == -1)
                    return -1;

                if (write_func(append, 1) == -1)
                    return -1;

            }

            else
            {
                printf("Nao comunica com o recetor\n");
                return -1;
            }

            tries++;
        }

        state_machine_control(RR);
    }

    ctrl_val++;

    printf("\n Confirmado ----\n\n");

    return bufSize;
}


int llread(char *packet)
{
    int i = 0;
    unsigned char escrita[MAX_PAYLOAD_SIZE * 2 + 6], rr[5], leitura[MAX_PAYLOAD_SIZE + 1];
    STOP = 0;
    state = Start;

    rr[0] = FLAG;
    rr[1] = A_EM;
    rr[2] = C_I0;
    if (ctrl_val % 2)
        rr[2] = C_I1;
    rr[3] = A_EM ^ C_I0;
    rr[4] = FLAG;

    while (STOP == FALSE)
    {                          
        res = read(fd, &Store_hex, 1);

        if (res <= 0)
        {
            printf("Timeout no RCV (llread)\n");
            return -1;
        }

        state_machine_data();
        if (state == INFO)
        {
            escrita[i] = Store_hex;
            i++;
        }
    }

    size_t len = strlen(leitura);
    if (len == -1)
        return -1;
    len--;
    for (i = 0; i < len; i++)
    {
        packet[i] = leitura[i];
    }
    printf("Bytes recebidos: %i\n", len);

    ctrl_val++;
    printf("Control flag val: %i\n", ctrl_val);



    if (write_func(rr, 5) == -1)
        return -1;


    return len;
}


int llclose(linkLayer connectionParameters, int showStatistics)
{
    unsigned char disc[5];

    disc[0] = FLAG;
    disc[1] = A_SET;
    disc[2] = C_DISC;
    disc[3] = A_SET ^ C_DISC;
    disc[4] = FLAG;

    if (connectionParameters.role == 0)
    {
        STOP = FALSE;
        tries = 0;
        state = 0;

        if (write_func(disc, 5) == -1)
            return -1;

        printf("DISC enviado\n");

        while (STOP == FALSE)
        {
            res = read(fd, &Store_hex, 1);

            if ((res <= 0) && (state == 0))
            {
                if (tries < connectionParameters.numTries-1)
                {
                    if (write_func(disc, 5) == -1)
                        return -1;
                }

                else
                {
                    printf("Não conectou com o recetor\n");
                    return -1;
                }

                tries++;
            }

            state_machine_control(DISC);
            printf("Estado: %i\n", state);
        }


        printf("DISC com successo\n");

        if (write_func(ua, 5) == -1)
            return -1;

        printf("UA enviado---- Ficheiro enviado com sucesso ----\n");
       

        return 1;
    }

    if (connectionParameters.role == 1)
    {
        STOP = 0;
        state = Start;

        while (STOP == FALSE)
        {                            
            res = read(fd, &Store_hex, 1);

            if (res <= 0)
            {
                printf("Timeout no RCV");
                return -1;
            }

            state_machine_control(DISC);
            printf("State: %i\n", state);
        }

        printf("DISC ---- successo\n");

        STOP = FALSE;
        state = 0;

        if (write_func(disc, 5) == -1)
            return -1;

        printf("DISC enviadas\n");

        while (STOP == FALSE)
        {                            /* loop for input */
            res = read(fd, &Store_hex, 1); /* returns after 1 chars have been input */

            if (res <= 0)
            {
                printf("Timeout on RCV");
                return -1;
            }

            state_machine_control(UA);
            printf("Estado: %i\n", state);
        }
        
        printf("UA com successo e finalizado\n");

        return 1;
    }

    return -1;
}



























