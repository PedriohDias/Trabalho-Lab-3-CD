#include "linklayer.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS4"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define TIMEOUT 30
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
#define DATA 6
#define Stop_Finall 7

//Flags
#define FLAG 0x5c
#define A_EM  0x03  
#define A_RE  0x01  
#define C_SET 0x08 
#define C_UA  0x06 
#define SET_LENGHT  5 // tamanho do SET
#define BCC(a,c) (a ^ c) // fazer xor a c

#define C_DISC 0x0b
#define C_I0 0x00
#define C_I1 0x02
#define C_R0 0x01
#define C_R1 0x21
#define C_RJ0 0x05
#define C_RJ1 0x25
#define ESC 0x5d



// Flags controlo
#define SET 0
#define UA 1
#define DISC 2
#define RR 3

int write_func(unsigned char *vet, int tamanho)
{
    int i=0, total=0;
    //int res;

     //unsigned char aux;

    for (i = 0;i< tamanho;i++)
    {
        aux = vet[i];
        res = write(fd, &aux, 1);
        total += res;
    }

    if (total !=tamanho)
        return -1;

    printf("%d bytes written\n", total);
    return 0;
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

    else (control == RR)
    {
        a = A_EM;
        c = C_I1;
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

    case START:
        if (Store_hex == FLAG)
        {
            state = FLAG_RCV;
        }
        break;

    case FLAG_RCV:
        if (Store_hex == A_SET)
        {
            state = A_RCV;
        }
        else if (Store_hex == FLAG_RCV)
        {
        }
        else
        {
            state = START;
        }
        break;

    case A_RCV:
        if (Store_hex == FLAG)
        {
            state = FLAG_RCV;
        }
        if (Store_hex == c)
        {
            printf("\nControl flag value: 0x%02x\n\n", (unsigned int)(c & 0xFF));
            state = C_RCV;
        }
        else
        {
            state = START;
        }
        break;

    case C_RCV:

        x = A_SET ^ c;
        if (Store_hex == FLAG)
        {
            state = FLAG_RCV;
        }
        if (Store_hex == x)
        {
            state = BCC_OK;
        }
        else
        {
            state = START;
        }
        break;

    case BCC_OK:

        state = DATA;
        break;

    case DATA:
        if (Store_hex == FLAG)
        {
            state = Stop_Finall;
            STOP = TRUE;
        }

        break;
    }
}

