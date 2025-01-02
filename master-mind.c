/*
 * MasterMind implementation: template; see comments below on which parts need to be completed
 * CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
 * This repo: https://gitlab-student.macs.hw.ac.uk/f28hs-2021-22/f28hs-2021-22-staff/f28hs-2021-22-cwk2-sys

 * Compile:
 gcc -c -o lcdBinary.o lcdBinary.c
 gcc -c -o master-mind.o master-mind.c
 gcc -o master-mind master-mind.o lcdBinary.o
 * Run:
 sudo ./master-mind

 OR use the Makefile to build
 > make all
 and run
 > make run
 and test
 > make test

 ***********************************************************************
 * The Low-level interface to LED, button, and LCD is based on:
 * wiringPi libraries by
 * Copyright (c) 2012-2013 Gordon Henderson.
 ***********************************************************************
 * See:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
*/

/* ======================================================= */
/* SECTION: includes                                       */
/* ------------------------------------------------------- */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include <unistd.h>
#include <string.h>
#include <time.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

/* --------------------------------------------------------------------------- */
/* Config settings */
/* you can use CPP flags to e.g. print extra debugging messages */
/* or switch between different versions of the code e.g. digitalWrite() in Assembler */

#define DEBUG
#undef ASM_CODE

// =======================================================
// Tunables
// PINs (based on BCM numbering)
// For wiring see CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
// GPIO pin for green LED
#define LED 13
// GPIO pin for red LED
#define LED2 5
// GPIO pin for button
#define BUTTON 19
// =======================================================
// delay for loop iterations (mainly), in ms
// in mili-seconds: 0.2s
#define DELAY 200
// in micro-seconds: 3s
#define TIMEOUT 3000000
// =======================================================
// APP constants   ---------------------------------
// number of colours and length of the sequence
#define COLS 3
#define SEQL 3
// =======================================================

// generic constants

#ifndef TRUE
#define TRUE (1 == 1)
#define FALSE (1 == 2)
#endif

#define PAGE_SIZE (4 * 1024)
#define BLOCK_SIZE (4 * 1024)

#define INPUT 0
#define OUTPUT 1

#define LOW 0
#define HIGH 1

// =======================================================
// Wiring (see inlined initialisation routine)

#define STRB_PIN 24
#define RS_PIN 25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

/* ======================================================= */
/* SECTION: constants and prototypes                       */
/* ------------------------------------------------------- */

// =======================================================
// char data for the CGRAM, i.e. defining new characters for the display

static unsigned char newChar[8] =
    {
        0b11111,
        0b10001,
        0b10001,
        0b10101,
        0b11111,
        0b10001,
        0b10001,
        0b11111,
};

/* Constants */

static const int colors = COLS;
static const int seqlen = SEQL;

static char *color_names[] = {"red", "green", "blue"};

static int *theSeq = NULL;

static int *seq1, *seq2, *cpy1, *cpy2;

/* --------------------------------------------------------------------------- */

// data structure holding data on the representation of the LCD
struct lcdDataStruct
{
  int bits, rows, cols;
  int rsPin, strbPin;
  int dataPins[8];
  int cx, cy;
};

static int lcdControl;

/* ***************************************************************************** */
/* INLINED fcts from wiringPi/devLib/lcd.c: */
// HD44780U Commands (see Fig 11, p28 of the Hitachi HD44780U datasheet)

#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY 0x04
#define LCD_CTRL 0x08
#define LCD_CDSHIFT 0x10
#define LCD_FUNC 0x20
#define LCD_CGRAM 0x40
#define LCD_DGRAM 0x80

// Bits in the entry register

#define LCD_ENTRY_SH 0x01
#define LCD_ENTRY_ID 0x02

// Bits in the control register

#define LCD_BLINK_CTRL 0x01
#define LCD_CURSOR_CTRL 0x02
#define LCD_DISPLAY_CTRL 0x04

// Bits in the function register

#define LCD_FUNC_F 0x04
#define LCD_FUNC_N 0x08
#define LCD_FUNC_DL 0x10

#define LCD_CDSHIFT_RL 0x04

// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define PI_GPIO_MASK (0xFFFFFFC0)

static unsigned int gpiobase;
static uint32_t *gpio;

// static int timed_out = 0;
volatile sig_atomic_t timed_out = 0;
/* ------------------------------------------------------- */
// misc prototypes

int failure(int fatal, const char *message, ...);
void waitForEnter(void);
void waitForButton(uint32_t *gpio, int button);

/* ======================================================= */
/* SECTION: hardware interface (LED, button, LCD display)  */
/* ------------------------------------------------------- */
/* low-level interface to the hardware */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Either put them in a separate file, lcdBinary.c, and use   */
/* inline Assembler there, or use a standalone Assembler file */
/* You can also directly implement them here (inline Asm).    */
/* ********************************************************** */

/* These are just prototypes; you need to complete the code for each function */

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
void digitalWrite(uint32_t *gpio, int pin, int value)
{
  int res;
  int res2;
  if (value == 1)
  {
    asm volatile(
        "\tMOV R0, %[inp1]\n"
        "\tLDR R1, [R0, #0x1C]\n" // adding offset of GPSET0 to gpio base address
        "\tMOV R2, #1\n"
        "\tLSL R2, R2, %[inp2]\n" // Shift the value in R2 left by inp2 bits (pin number)
        "\tSTR  R2, [R0, #0x1C]\n"// Store the modified value back at gpio address + offset (GPSET0)
        : [result] "=r"(res)
        : [inp1] "r"(gpio),
          [inp2] "r"(pin)
        : "r0", "r1", "r2");
  }
  else
  {
    asm volatile(
        "\tMOV R0, %[inp1]\n"
        "\tLDR R1, [R0, #0x28]\n" // Load the value at gpio address + offset (GPCLR0)
        "\tMOV R2, #1\n"
        "\tLSL R2, R2, %[inp2]\n"
        "\tSTR R2, [R0, #0x28]\n"
        : [result] "=r"(res2)
        : [inp1] "r"(gpio),
          [inp2] "r"(pin)
        : "r0", "r1", "r2");
  }
}

/* set the @mode@ of a GPIO @pin@ to INPUT or OUTPUT; @gpio@ is the mmaped GPIO base address */
void pinMode(uint32_t *gpio, int pin, int mode)
{
  int fsel;
  int shift;
  int res;
  // calculate fsel and shift values
  if (pin < 10)
  {
    fsel = 0;
    shift = pin * 3;
  }
  else
  {
    fsel = pin / 10;
    shift = (pin % 10) * 3;
  }
  //*C version-->(gpio + fSel) = (*(gpio + fSel) & ˜(7 << shift)) | (1 <<shift) ;
  if (mode == 1)
  {
    asm volatile(
        "\tMOV R0, %[inp1]\n"            // moves memory address of gpio pointer
        "\tLDR R1, [%[inp1], %[inp2]]\n" // loads value from memory at address gpio+offset(fsel*4)
        "\tMOV R3, #7\n"
        "\tLSL R3, R3, %[inp3]\n"        // creating bitmask
        "\tBIC R4, R1, R3\n"             // bitwise AND operation on value in r1 and value of not operation of r3
        "\tMOV R3, #1\n"                 // to set mode
        "\tLSL R3, R3, %[inp3]\n"        // LSL shift by the value of 'shift'
        "\tORR R4, R4, R3\n"             // bitwise OR to set approprirate bit in the GPIO register to 1 setting it to OUTPUT mode
        "\tSTR R4, [%[inp1], %[inp2]]\n" // stores it back into memory
        : [result] "=r"(res)
        : [inp1] "r"(gpio),
          [inp2] "r"(fsel * 4),
          [inp3] "r"(shift)
        : "r0", "r1", "r3", "r4");
  }
  else
  { // If mode is not 1, set pin to input
    asm volatile(
        "\tMOV R0, %[inp1]\n"
        "\tLDR R1, [%[inp1], %[inp2]]\n"
        "\tMOV R3, #7\n"
        "\tLSL R3, R3, %[inp3]\n"
        "\tBIC R4, R1, R3\n"
        "\tSTR R4, [%[inp1], %[inp2]]\n"
        : [result] "=r"(res)
        : [inp1] "r"(gpio),
          [inp2] "r"(fsel * 4),
          [inp3] "r"(shift)
        : "r0", "r1", "r3", "r4");
  }
}

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
/* can use digitalWrite(), depending on your implementation */
void writeLED(uint32_t *gpio, int led, int value)
{
  digitalWrite(gpio, led, value);
}

/* read a @value@ (LOW or HIGH) from pin number @pin@ (a button device); @gpio@ is the mmaped GPIO base address */
int readButton(uint32_t *gpio, int button)
{
  int res;
  asm volatile(
      "\tMOV R0, %[inp1]\n"
      "\tLDR R1, [R0, #0x34]\n" // offset for GPLEV0 register to read button value
      "\tMOV R2, #1\n"
      "\tLSL R2, R2, %[inp2]\n" // shifts 1 by button value
      "\tTST R2,R1\n"           // sets status bits based on result r2 AND r1
      "\tMOVEQ %[result],#0\n"  // move 0 to output parameter result if zero flag is set
      "\tMOVNE %[result],#1\n"  // move 1 if zero flag is not set
      : [result] "=r"(res)
      : [inp1] "r"(gpio),
        [inp2] "r"(button)
      : "r0", "r1", "r2");

  return res; // return value
}

/* wait for a button input on pin number @button@; @gpio@ is the mmaped GPIO base address */
/* can use readButton(), depending on your implementation */
void waitForButton(uint32_t *gpio, int button)
{
  int currentState = 0;
  while (currentState == 0)
  {
     // Read the current state of the button using the readButton function
    currentState = readButton(gpio, button);
  }
}

/* ======================================================= */
/* SECTION: game logic                                     */
/* ------------------------------------------------------- */
/* AUX fcts of the game logic */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* initialise the secret sequence; by default it should be a random sequence */
void initSeq()
{
  /* ***  COMPLETE the code here  ***  */
  srand(time(NULL)); // Seed the random number generator with current time
  for (int i = 0; i < seqlen; i++)
  {
    theSeq[i] = rand() % colors + 1;
  }; // Generate random value between 1 and 3 and store in sequence
}

/* display the sequence on the terminal window, using the format from the sample run in the spec */
void showSeq(int *seq)
{
  /* ***  COMPLETE the code here  ***  */
  printf("The Sequence is: \n");
  for (int i = 0; i < seqlen; i++) // Iterate through the sequence
  {
    printf(" %s ", color_names[seq[i]-1]); // Print the color name
  }
  printf("\n");
}

#define NAN1 8
#define NAN2 9

//C version of CountMatches
// char* getHint(char* secret, char* guess) {
//     int bulls = 0;
//     int cows = 0;
//     int s[3] = {0}; // Array to store occurrences of digits in secret and guess
//     int g[3] = {0}; 

//     for (int i = 0; i < strlen(secret); i++) {
//         if (secret[i] == guess[i]) {
//             bulls++;
//         } else {
//             s[secret[i]]++; 
//             g[guess[i]]++;  
//         }
//     }
    
//     // Count cows by taking minimum occurrences of each digit
//     for (int i = 0; i < 3; i++) {
//        if (s[i] < g[i]) {
//             cows += s[i];
//         } else {
//             cows += g[i];
//         }
//     }

//     // Allocate memory for the pair of values
//     int *matches = (int *)malloc(2 * sizeof(int));
//     // Set the values
//     matches[0] = bulls;
//     matches[1] = cows;
    
//     return matches;
// }


int * /* or int* */ countMatches(int *seq1, int *seq2)
{

  int *matches = (int *)malloc(2 * sizeof(int));// Allocate memory for the matches array
  int res1;
  int res2;
  int seq1Arr[3]={seq1[0], seq1[1], seq1[2]};// Storing seq1 and seq2 as arrays.
  int seq2Arr[3]={seq2[0], seq2[1], seq2[2]};
  int s[4]={0,0,0,0};//Implemented secret and guess arrays to store the frequency of characters that occur in seq1 and seq2.
  int g[4]={0,0,0,0};
// pointers for the arrays.
  int *s1 = &seq1Arr[0];
  int *s2 = &seq2Arr[0];
  int *ap = &s[0];
  int *ap1 = &g[0];

  asm volatile(
      "\tLDR R1, %[inp0]\n"// Load the addresses of arrays into register
      "\tLDR R2, %[inp1]\n"
      "\tLDR R3, %[inp2]\n"
      "\tLDR R4, %[inp3]\n"
       "\tMOV R5, #0\n"
       "\tLDR R6, [R1]\n"// Load the first element of the seq1 array into R6
       "\tLDR R7, [R2]\n"// Load the first element of the seq2 array into R7
       "\tCMP R7, R6\n" // Compare the first elements of both arrays
       "\tBNE approx1\n" 
       "\tADD R5,#1\n" // Used to increment bulls value
       "\tb step2\n"
       "\tapprox1:\n"
       "\tLDR R3, %[inp2]\n"
       "\tLDR R4, %[inp3]\n"  
       "\tMOV R8, #0\n" // intializing counter
       "\tloop1:\n"
       "\tADD R8,#1\n"
       "\tADD R3,#4\n"//Starts incrementing pointer from index 1 .
       "\tCMP R6, R8\n"
       "\tBNE loop1\n"
       "\tLDR R8, [R3]\n"// Load the value at the memory location pointed by R3 into R8
       "\tADD R8, #1\n"  // Increment the value by 1
       "\tSTR R8, [R3]\n" // Store the updated value back to the memory location pointed by R3
       "\tMOV R8, #0\n" // Reset loop counter register R8 to 0
       "\tloop1a:\n" // Same operations done for the guess pointer r4
       "\tADD R8,#1\n"
       "\tADD R4,#4\n"
       "\tCMP R7, R8\n"
       "\tBNE loop1a\n"
       "\tLDR R8, [R4]\n"
       "\tADD R8, #1\n"
       "\tSTR R8, [R4]\n"
       "\tLDR R3, %[inp2]\n"
       "\tLDR R4, %[inp3]\n" 
       "\tstep2:\n"
       "\tADD R1, #4\n"
       "\tADD R2, #4\n"
       "\tLDR R6, [R1]\n"
       "\tLDR R7, [R2]\n"
       "\tCMP R7, R6\n"
       "\tBNE approx2\n"
       "\tADD R5, #1\n"
       "\tb step3\n"
       "\tapprox2:\n"
       "\tLDR R3, %[inp2]\n"
       "\tLDR R4, %[inp3]\n"
       "\tMOV R8, #0\n"
       "\tloop2:\n"
       "\tADD R8,#1\n"
       "\tADD R3,#4\n"
       "\tcmp R6, R8\n"
       "\tbne loop2\n"
       "\tLDR R8, [R3]\n"
       "\tADD R8, #1\n"
       "\tSTR R8, [R3]\n"
       "\tMOV R8, #0\n"
       "\tloop2a:\n"
       "\tADD R8,#1\n"
       "\tADD R4,#4\n"
       "\tcmp R7, R8\n"
       "\tbne loop2a\n"
       "\tLDR R8, [R4]\n" 
       "\tADD R8, #1\n"
       "\tSTR R8, [R4]\n"
       "\tLDR R3, %[inp2]\n"
       "\tLDR R4, %[inp3]\n"
       "\tstep3:\n"
       "\tADD R1, #4\n"
       "\tADD R2, #4\n"
       "\tLDR R6, [R1]\n"
       "\tLDR R7, [R2]\n"
       "\tCMP R7, R6\n"
       "\tbne approx3\n"
       "\tadd R5, #1\n"
       "\tb end\n"
       "\tapprox3:\n"
       "\tLDR R3, %[inp2]\n"
       "\tLDR R4, %[inp3]\n"   
       "\tMOV R8, #0\n"
       "\tloop3:\n"
       "\tADD R8,#1\n"
       "\tADD R3,#4\n"
       "\tcmp R6, R8\n"
       "\tbne loop3\n"
       "\tLDR R8, [R3]\n"
       "\tADD R8, #1\n"
       "\tSTR R8, [R3]\n"
       "\tMOV R8, #0\n"
       "\tloop3a:\n"
       "\tADD R8,#1\n"
       "\tADD R4,#4\n"
       "\tcmp R7, R8\n"
       "\tbne loop3a\n"
       "\tLDR R8, [R4]\n"
       "\tADD R8, #1\n"
       "\tSTR r8, [R4]\n"
       "\tend:\n"
       "\tMOV R9, #0\n"
       "\tcmp r5, #3\n"
       "\tbeq suss\n"
       "\tLDR R3, %[inp2]\n"
       "\tLDR R4, %[inp3]\n"

       "\tMOV R10, #0\n"
       "\tMOV R9, #0\n"
       "\tloop4:\n"
       "\tADD R3,#4\n"
       "\tADD R4,#4\n"
       "\tLDR R8, [R3]\n"
       "\tLDR R6, [R4]\n"
       "\tcmp R6, R8\n"
       "\tBLE low\n"
       "\tADD R9, R8\n" // Find minimum from secret and guess arrays and adding them to R9
       "\tADD R10, #1\n"
       "\tcmp R10, #3\n"
       "\tbne loop4\n"
       "\tb suss\n"
       "\tlow:\n"
       "\tADD R9, R6\n" 
       "\tADD R10, #1\n" 
       "\tcmp r10, #3\n"  
       "\tbne loop4\n"
       "\tsuss:\n" 
       "\tMOV %[result1], r5\n" // Stores bull result in R5
       "\tMOV %[result2], r9\n" //Stores cow result in R9
      : [result1] "=r"(res1),
        [result2] "=r"(res2)
      : [inp0] "g"(s1),
        [inp1] "g"(s2),
        [inp2] "g"(ap),
        [inp3] "g"(ap1)
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r8", "r9", "r10");

  matches[0] = res1;//storing result into matches array
  matches[1] = res2;
  
  return matches;
}
/* show the results from calling countMatches on seq1 and seq1 */
void showMatches(int * /* or int* */ code, /* only for debugging */ int *seq1, int *seq2, /* optional, to control layout */ int lcd_format)
{
  /* ***  COMPLETE the code here  ***  */
  int exactMatches, approxMatches;                                 // Declare two variables for storing the number of exact and approximate matches.
  if (code == NULL || seq1 == NULL || seq2 == NULL || seqlen <= 0) // Check for invalid input parameters
  {
    printf("Invalid input parameters\n"); // Print an error message if any input parameter is invalid.
    return;
  }
  if (code[0] == seqlen) // Check if the first element in the code array is equal to the sequence length.
  {                      // If true, assign the values of the two elements in the code array to exactMatches and approxMatches respectively.
    exactMatches = code[0];
    approxMatches = code[1];
    printf("%d exact\n", exactMatches);
    printf("%d approximate\n", approxMatches);
    // Display the number of exact and approximate matches.
  }
  else
  {
    exactMatches = code[0];
    approxMatches = code[1];
    printf("%d exact\n", exactMatches);// Print the number of exact matches approx matches.
    printf("%d approximate\n", approxMatches);
  }
}

/* parse an integer value as a list of digits, and put them into @seq@ */
/* needed for processing command-line with options -s or -u            */
void readSeq(int *seq, int val)
{
  /* ***  COMPLETE the code here  ***  */
  // define maximum length of sequence
  int i = 0;
  while (val > 0 && i < seqlen)
  {
    seq[i++] = val % 10;
    val /= 10;
  }
  // reverse the order of the digits using swap method
  int j = i - 1;
  i = 0;
  while (i < j)
  {
    int temp = seq[i];// Swap the digits at positions 'i' and 'j'
    seq[i++] = seq[j];
    seq[j--] = temp;
  }
}

/* read a guess sequence fron stdin and store the values in arr */
/* only needed for testing the game logic, without button input */
int readNum(int max)
{
  int guess = 0;
  scanf("%d", &guess);
  int *arr = (int *)malloc(seqlen * sizeof(int));

  readSeq(arr, guess);
  int len = 0;
  while (arr[len] != 0 && len < max)
  {
    len++;
  }
  free(arr);
  return len;
}

/* ======================================================= */
/* SECTION: TIMER code                                     */
/* ------------------------------------------------------- */
/* TIMER code */

/* timestamps needed to implement a time-out mechanism */
// static uint64_t startT, stopT;
volatile uint64_t startT, stopT;
/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* you may need this function in timer_handler() below  */
/* use the libc fct gettimeofday() to implement it      */
uint64_t timeInMicroseconds()
{
  /* ***  COMPLETE the code here  ***  */
  struct timeval tv, tNow, tLong, tEnd;
  uint64_t now;
  gettimeofday(&tv, NULL);
  now = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec; // in us
  // now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ; // in ms

  return (uint64_t)now; 
}

/* this should be the callback, triggered via an interval timer, */
/* that is set-up through a call to sigaction() in the main fct. */
void timer_handler(int signum)
{
  /* ***  COMPLETE the code here  ***  */
  static int count = 0;
  stopT = timeInMicroseconds();
  count++;

  timed_out = 1;
}

/* initialise time-stamps, setup an interval timer, and install the timer_handler callback */
void initITimer(uint64_t timeout)
{
  /* ***  COMPLETE the code here  ***  */
  struct itimerval timer;
  struct sigaction sa;
  fprintf(stderr, " Setting timer with a delay of %d micro-seconds ...\n", DELAY);
  /* Install timer_handler as the signal handler for SIGALRM. */
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = &timer_handler;
  // ORIG: sigaction (SIGVTALRM, &sa, NULL);
  /* From the man page:
       The  sigaction()  system  call  is used to change the action taken by a
       process on receipt of a specific signal.  (See signal(7) for  an  over‐
       view of signals.)
       signum  specifies the signal and can be any valid signal except SIGKILL
       and SIGSTOP.
       If act is non-NULL, the new action for signal signum is installed  from
       act.  If oldact is non-NULL, the previous action is saved in oldact.
  */
  sigaction(SIGALRM, &sa, NULL);
  /* set the timer to expire after 250 msec... */
  timer.it_value.tv_sec = timeout;
  timer.it_value.tv_usec = 0;
  /* ... and every 250 msec after that. */
  timer.it_interval.tv_sec = 0;
  timer.it_interval.tv_usec = 0;
  /* Start a virtual timer. It counts down whenever this process is executing. */
  // ORIG: setitimer (ITIMER_VIRTUAL, &timer, NULL);
  setitimer(ITIMER_REAL, &timer, NULL);
  // setitimer(ITIMER_VIRTUAL, &timer, NULL);
  startT = timeInMicroseconds();
}

/* ======================================================= */
/* SECTION: Aux function                                   */
/* ------------------------------------------------------- */
/* misc aux functions */

int failure(int fatal, const char *message, ...)
{
  va_list argp;
  char buffer[1024];

  if (!fatal) //  && wiringPiReturnCodes)
    return -1;

  va_start(argp, message);
  vsnprintf(buffer, 1023, message, argp);
  va_end(argp);

  fprintf(stderr, "%s", buffer);
  exit(EXIT_FAILURE);

  return 0;
}

/*
 * waitForEnter:
 *********************************************************************************
 */

void waitForEnter(void)
{
  printf("Press ENTER to continue: ");
  (void)fgetc(stdin);
}

/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void delay(unsigned int howLong)
{
  struct timespec sleeper, dummy;

  sleeper.tv_sec = (time_t)(howLong / 1000);
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000;

  nanosleep(&sleeper, &dummy);
}

/* From wiringPi code; comment by Gordon Henderson
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicroseconds(unsigned int howLong)
{
  struct timespec sleeper;
  unsigned int uSecs = howLong % 1000000;
  unsigned int wSecs = howLong / 1000000;

  /**/ if (howLong == 0)
    return;
#if 0
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
#endif
  else
  {
    sleeper.tv_sec = wSecs;
    sleeper.tv_nsec = (long)(uSecs * 1000L);
    nanosleep(&sleeper, NULL);
  }
}

/* ======================================================= */
/* SECTION: LCD functions                                  */
/* ------------------------------------------------------- */
/* medium-level interface functions (all in C) */

/* from wiringPi:
 * strobe:
 *	Toggle the strobe (Really the "E") pin to the device.
 *	According to the docs, data is latched on the falling edge.
 *********************************************************************************
 */

void strobe(const struct lcdDataStruct *lcd)
{

  // Note timing changes for new version of delayMicroseconds ()
  digitalWrite(gpio, lcd->strbPin, 1);
  delayMicroseconds(50);
  digitalWrite(gpio, lcd->strbPin, 0);
  delayMicroseconds(50);
}

/*
 * sentDataCmd:
 *	Send an data or command byte to the display.
 *********************************************************************************
 */

void sendDataCmd(const struct lcdDataStruct *lcd, unsigned char data)
{
  register unsigned char myData = data;
  unsigned char i, d4;

  if (lcd->bits == 4)
  {
    d4 = (myData >> 4) & 0x0F;
    for (i = 0; i < 4; ++i)
    {
      digitalWrite(gpio, lcd->dataPins[i], (d4 & 1));
      d4 >>= 1;
    }
    strobe(lcd);

    d4 = myData & 0x0F;
    for (i = 0; i < 4; ++i)
    {
      digitalWrite(gpio, lcd->dataPins[i], (d4 & 1));
      d4 >>= 1;
    }
  }
  else
  {
    for (i = 0; i < 8; ++i)
    {
      digitalWrite(gpio, lcd->dataPins[i], (myData & 1));
      myData >>= 1;
    }
  }
  strobe(lcd);
}

/*
 * lcdPutCommand:
 *	Send a command byte to the display
 *********************************************************************************
 */

void lcdPutCommand(const struct lcdDataStruct *lcd, unsigned char command)
{
#ifdef DEBUG
  // fprintf(stderr, "lcdPutCommand: digitalWrite(%d,%d) and sendDataCmd(%d,%d)\n", lcd->rsPin, 0, lcd, command);
#endif
  digitalWrite(gpio, lcd->rsPin, 0);
  sendDataCmd(lcd, command);
  delay(2);
}

void lcdPut4Command(const struct lcdDataStruct *lcd, unsigned char command)
{
  register unsigned char myCommand = command;
  register unsigned char i;

  digitalWrite(gpio, lcd->rsPin, 0);

  for (i = 0; i < 4; ++i)
  {
    digitalWrite(gpio, lcd->dataPins[i], (myCommand & 1));
    myCommand >>= 1;
  }
  strobe(lcd);
}

/*
 * lcdHome: lcdClear:
 *	Home the cursor or clear the screen.
 *********************************************************************************
 */

void lcdHome(struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdHome: lcdPutCommand(%d,%d)\n", lcd, LCD_HOME);
#endif
  lcdPutCommand(lcd, LCD_HOME);
  lcd->cx = lcd->cy = 0;
  delay(5);
}

void lcdClear(struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  // fprintf(stderr, "lcdClear: lcdPutCommand(%d,%d) and lcdPutCommand(%d,%d)\n", lcd, LCD_CLEAR, lcd, LCD_HOME);
#endif
  lcdPutCommand(lcd, LCD_CLEAR);
  lcdPutCommand(lcd, LCD_HOME);
  lcd->cx = lcd->cy = 0;
  delay(5);
}

/*
 * lcdPosition:
 *	Update the position of the cursor on the display.
 *	Ignore invalid locations.
 *********************************************************************************
 */

void lcdPosition(struct lcdDataStruct *lcd, int x, int y)
{

  if ((x > lcd->cols) || (x < 0))
    return;
  if ((y > lcd->rows) || (y < 0))
    return;

  lcdPutCommand(lcd, x + (LCD_DGRAM | (y > 0 ? 0x40 : 0x00) /* rowOff [y] */));

  lcd->cx = x;
  lcd->cy = y;
}

/*
 * lcdDisplay: lcdCursor: lcdCursorBlink:
 *	Turn the display, cursor, cursor blinking on/off
 *********************************************************************************
 */

void lcdDisplay(struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |= LCD_DISPLAY_CTRL;
  else
    lcdControl &= ~LCD_DISPLAY_CTRL;

  lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

void lcdCursor(struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |= LCD_CURSOR_CTRL;
  else
    lcdControl &= ~LCD_CURSOR_CTRL;

  lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

void lcdCursorBlink(struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |= LCD_BLINK_CTRL;
  else
    lcdControl &= ~LCD_BLINK_CTRL;

  lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

/*
 * lcdPutchar:
 *	Send a data byte to be displayed on the display. We implement a very
 *	simple terminal here - with line wrapping, but no scrolling. Yet.
 *********************************************************************************
 */

void lcdPutchar(struct lcdDataStruct *lcd, unsigned char data)
{
  digitalWrite(gpio, lcd->rsPin, 1);
  sendDataCmd(lcd, data);

  if (++lcd->cx == lcd->cols)
  {
    lcd->cx = 0;
    if (++lcd->cy == lcd->rows)
      lcd->cy = 0;

    // TODO: inline computation of address and eliminate rowOff
    lcdPutCommand(lcd, lcd->cx + (LCD_DGRAM | (lcd->cy > 0 ? 0x40 : 0x00) /* rowOff [lcd->cy] */));
  }
}

/*
 * lcdPuts:
 *	Send a string to be displayed on the display
 *********************************************************************************
 */

void lcdPuts(struct lcdDataStruct *lcd, const char *string)
{
  while (*string)
    lcdPutchar(lcd, *string++);
}

/* ======================================================= */
/* SECTION: aux functions for game logic                   */
/* ------------------------------------------------------- */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* --------------------------------------------------------------------------- */
/* interface on top of the low-level pin I/O code */

/* blink the led on pin @led@, @c@ times */
void blinkN(uint32_t *gpio, int led, int c)
{
  /* ***  COMPLETE the code here  ***  */
  for (int i = 0; i < c; i++)
  {
    // Writes to LED to turn it on
    digitalWrite(gpio, led, HIGH);
    delay(700);
    // Writes to LED to turn it off
    digitalWrite(gpio, led, LOW);
    delay(700);
  }
}

/* ======================================================= */
/* SECTION: main fct                                       */
/* ------------------------------------------------------- */

int main(int argc, char *argv[])
{ // this is just a suggestion of some variable that you may want to use
  struct lcdDataStruct *lcd;
  int bits, rows, cols;
  unsigned char func;

  int found = 0, attempts = 0, i, j, code;
  int c, d, buttonPressed, rel, foo;
  int *attSeq;

  int pinLED = LED, pin2LED2 = LED2, pinButton = BUTTON;

  int fSel, shift, pin, clrOff, setOff, off, res;
  int fd;

  int exact, contained;
  char str1[32];
  char str2[32];

  struct timeval t1, t2;
  int t;

  char buf[32];

  // variables for command-line processing
  char str_in[20], str[20] = "some text";
  int verbose = 0, debug = 0, help = 0, opt_m = 0, opt_n = 0, opt_s = 0, unit_test = 0;
  int *res_matches;
  theSeq = (int *)malloc(seqlen * sizeof(int));
  const int LIMIT = 7;
  // -------------------------------------------------------
  // process command-line arguments

  // see: man 3 getopt for docu and an example of command line parsing
  { // see the CW spec for the intended meaning of these options
    int opt;
    while ((opt = getopt(argc, argv, "hvdus:")) != -1)
    {
      switch (opt)
      {
      case 'v':
        verbose = 1;
        break;
      case 'h':
        help = 1;
        break;
      case 'd':
        debug = 1;
        break;
      case 'u':
        unit_test = 1;
        break;
      case 's':
        opt_s = atoi(optarg);
        break;
      default: /* '?' */
        fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
        exit(EXIT_FAILURE);
      }
    }
  }

  if (help)
  {
    fprintf(stderr, "MasterMind program, running on a Raspberry Pi, with connected LED, button and LCD display\n");
    fprintf(stderr, "Use the button for input of numbers. The LCD display will show the matches with the secret sequence.\n");
    fprintf(stderr, "For full specification of the program see: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf\n");
    fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
    exit(EXIT_SUCCESS);
  }

  if (unit_test && optind >= argc - 1)
  {
    fprintf(stderr, "Expected 2 arguments after option -u\n");
    exit(EXIT_FAILURE);
  }

  if (verbose && unit_test)
  {
    printf("1st argument = %s\n", argv[optind]);
    printf("2nd argument = %s\n", argv[optind + 1]);
  }

  if (verbose)
  {
    fprintf(stdout, "Settings for running the program\n");
    fprintf(stdout, "Verbose is %s\n", (verbose ? "ON" : "OFF"));
    fprintf(stdout, "Debug is %s\n", (debug ? "ON" : "OFF"));
    fprintf(stdout, "Unittest is %s\n", (unit_test ? "ON" : "OFF"));
    if (opt_s)
      fprintf(stdout, "Secret sequence set to %d\n", opt_s);
  }

  seq1 = (int *)malloc(seqlen * sizeof(int));
  seq2 = (int *)malloc(seqlen * sizeof(int));
  cpy1 = (int *)malloc(seqlen * sizeof(int));
  cpy2 = (int *)malloc(seqlen * sizeof(int));

  // check for -u option, and if so run a unit test on the matching function
  if (unit_test && argc > optind + 1)
  { // more arguments to process; only needed with -u
    strcpy(str_in, argv[optind]);
    opt_m = atoi(str_in);
    strcpy(str_in, argv[optind + 1]);
    opt_n = atoi(str_in);
    // CALL a test-matches function; see testm.c for an example implementation
    readSeq(seq1, opt_m); // turn the integer number into a sequence of numbers
    readSeq(seq2, opt_n); // turn the integer number into a sequence of numbers
    if (verbose)
      fprintf(stdout, "Testing matches function with sequences %d and %d\n", opt_m, opt_n);
    res_matches = countMatches(seq1, seq2);
    showMatches(res_matches, seq1, seq2, 1);
    exit(EXIT_SUCCESS);
  }
  else
  {
    /* nothing to do here; just continue with the rest of the main fct */
  }

  if (opt_s)
  { // if -s option is given, use the sequence as secret sequence
    if (theSeq == NULL)
      theSeq = (int *)malloc(seqlen * sizeof(int));
    readSeq(theSeq, opt_s);
    // showSeq(theSeq);
    if (verbose)
      fprintf(stderr, "Running program with secret sequence:\n");
    showSeq(theSeq);
  }

  // -------------------------------------------------------
  // LCD constants, hard-coded: 16x2 display, using a 4-bit connection
  bits = 4;
  cols = 16;
  rows = 2;
  // -------------------------------------------------------

  printf("Raspberry Pi LCD driver, for a %dx%d display (%d-bit wiring) \n", cols, rows, bits);

  if (geteuid() != 0)
    fprintf(stderr, "setup: Must be root. (Did you forget sudo?)\n");

  // init of guess sequence, and copies (for use in countMatches)
  attSeq = (int *)malloc(seqlen * sizeof(int));
  cpy1 = (int *)malloc(seqlen * sizeof(int));
  cpy2 = (int *)malloc(seqlen * sizeof(int));

  // -----------------------------------------------------------------------------
  // constants for RPi2
  gpiobase = 0x3F200000;

  // -----------------------------------------------------------------------------
  // memory mapping
  // Open the master /dev/memory device

  if ((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0)
    return failure(FALSE, "setup: Unable to open /dev/mem: %s\n", strerror(errno));

  // GPIO:
  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, gpiobase);
  if ((int32_t)gpio == -1)
    return failure(FALSE, "setup: mmap (GPIO) failed: %s\n", strerror(errno));

  // -------------------------------------------------------
  // Configuration of LED and BUTTON

  /* ***  COMPLETE the code here  ***  */

  pinMode(gpio, 5, OUTPUT);
  pinMode(gpio, 13, OUTPUT);
  pinMode(gpio, 19, INPUT);

  // -------------------------------------------------------
  // INLINED version of lcdInit (can only deal with one LCD attached to the RPi):
  // you can use this code as-is, but you need to implement digitalWrite() and
  // pinMode() which are called from this code
  // Create a new LCD:
  lcd = (struct lcdDataStruct *)malloc(sizeof(struct lcdDataStruct));
  if (lcd == NULL)
    return -1;

  // hard-wired GPIO pins
  lcd->rsPin = RS_PIN;
  lcd->strbPin = STRB_PIN;
  lcd->bits = 4;
  lcd->rows = rows; // # of rows on the display
  lcd->cols = cols; // # of cols on the display
  lcd->cx = 0;      // x-pos of cursor
  lcd->cy = 0;      // y-pos of curosr

  lcd->dataPins[0] = DATA0_PIN;
  lcd->dataPins[1] = DATA1_PIN;
  lcd->dataPins[2] = DATA2_PIN;
  lcd->dataPins[3] = DATA3_PIN;

  // lcds [lcdFd] = lcd ;

  digitalWrite(gpio, lcd->rsPin, 0);
  pinMode(gpio, lcd->rsPin, OUTPUT);
  digitalWrite(gpio, lcd->strbPin, 0);
  pinMode(gpio, lcd->strbPin, OUTPUT);

  for (i = 0; i < bits; ++i)
  {
    digitalWrite(gpio, lcd->dataPins[i], 0);
    pinMode(gpio, lcd->dataPins[i], OUTPUT);
  }
  delay(35); // mS

  // Gordon Henderson's explanation of this part of the init code (from wiringPi):
  // 4-bit mode?
  //	OK. This is a PIG and it's not at all obvious from the documentation I had,
  //	so I guess some others have worked through either with better documentation
  //	or more trial and error... Anyway here goes:
  //
  //	It seems that the controller needs to see the FUNC command at least 3 times
  //	consecutively - in 8-bit mode. If you're only using 8-bit mode, then it appears
  //	that you can get away with one func-set, however I'd not rely on it...
  //
  //	So to set 4-bit mode, you need to send the commands one nibble at a time,
  //	the same three times, but send the command to set it into 8-bit mode those
  //	three times, then send a final 4th command to set it into 4-bit mode, and only
  //	then can you flip the switch for the rest of the library to work in 4-bit
  //	mode which sends the commands as 2 x 4-bit values.

  if (bits == 4)
  {
    func = LCD_FUNC | LCD_FUNC_DL; // Set 8-bit mode 3 times
    lcdPut4Command(lcd, func >> 4);
    delay(35);
    lcdPut4Command(lcd, func >> 4);
    delay(35);
    lcdPut4Command(lcd, func >> 4);
    delay(35);
    func = LCD_FUNC; // 4th set: 4-bit mode
    lcdPut4Command(lcd, func >> 4);
    delay(35);
    lcd->bits = 4;
  }
  else
  {
    failure(TRUE, "setup: only 4-bit connection supported\n");
    func = LCD_FUNC | LCD_FUNC_DL;
    lcdPutCommand(lcd, func);
    delay(35);
    lcdPutCommand(lcd, func);
    delay(35);
    lcdPutCommand(lcd, func);
    delay(35);
  }

  if (lcd->rows > 1)
  {
    func |= LCD_FUNC_N;
    lcdPutCommand(lcd, func);
    delay(35);
  }

  // Rest of the initialisation sequence
  lcdDisplay(lcd, TRUE);
  lcdCursor(lcd, FALSE);
  lcdCursorBlink(lcd, FALSE);
  lcdClear(lcd);

  lcdPutCommand(lcd, LCD_ENTRY | LCD_ENTRY_ID);     // set entry mode to increment address counter after write
  lcdPutCommand(lcd, LCD_CDSHIFT | LCD_CDSHIFT_RL); // set display shift to right-to-left

  // END lcdInit ------
  // -----------------------------------------------------------------------------
  // Start of game
  fprintf(stderr, "Printing welcome message on the LCD display ...\n");
  /* ***  COMPLETE the code here  ***  */

  /* initialise the secret sequence */
  if (!opt_s)
    initSeq();
  if (debug)
    showSeq(theSeq);

  fprintf(stderr, "Welcome to Master-mind!!\n"); // Print welcome message to standard error
  // Set LCD cursor position to top-left corner
  lcdPosition(lcd, 0, 0);
  lcdPuts(lcd, "WELCOME TO");
  // Set LCD cursor position to second row
  lcdPosition(lcd, 0, 1);
  lcdPuts(lcd, "MASTERMIND");

  delay(2000);   // Wait for 2 seconds
  lcdClear(lcd); // clear display

  delay(150);
  // Print message to standard error and set LCD cursor position to top-left corner to display "PRESS BUTTON"
  fprintf(stderr, "Press button to start playing\n");
  // Set LCD cursor position to top-left row
  lcdPosition(lcd, 0, 0);
  lcdPuts(lcd, "PRESS BUTTON");
  // Set LCD cursor position to second row
  lcdPosition(lcd, 0, 1);
  lcdPuts(lcd, "TO START PLAYING");

  waitForButton(gpio, BUTTON);
 
 

  int counter = 0;

  // -----------------------------------------------------------------------------
  // +++++ main loop
  while (!found && attempts < 4) // while loop to keep playing until the sequence is found or the player has used up all their attempts
  {
    attempts++;
    counter = 0;
    fprintf(stderr, "Enter your guess: \n"); // Print a prompt for the user to enter their guess on the console
    lcdClear(lcd);                           // clear the LCD screen
    lcdPosition(lcd, 0, 0);                  // set the cursor position to the first row and first column of the LCD
    lcdPuts(lcd, "ENTER YOUR");              // print "ENTER YOUR" on the first row of the LCD
    lcdPosition(lcd, 0, 1);                  // set the cursor position to the second row and first column of the LCD
    lcdPuts(lcd, "GUESS");                   // print "GUESS" on the second row of the LCD

    timed_out = 0; // set timed_out flag to 0
    int inpNo = 0; // initialize a variable inpNo to keep track of the number of inputs

    int *result = (int *)malloc(seqlen * sizeof(int)); // Dynamically allocate memory for the result array based on the length of the sequence

    while (inpNo < 3) // while loop to get 3 inputs from the user
    {
      delay(300);
      initITimer(3);// Initialize a timer with a 3-second duration
      counter = 0;
      while (timed_out == 0 && counter < 3) // while loop to wait for user input or timeout or max input of 3
      {
       
        if (readButton(gpio, BUTTON)) // if the button is pressed, increment the counter
        {
          counter++;
           delay(300);  
       }  
      }
      printf("Number of button presses: %d\n", counter);
      if (counter !=0)
      {
      delay(200);            // remove
      blinkN(gpio, LED2, 1); // blink the second LED once
      delay(100);
      blinkN(gpio, LED, counter); // blink the first LED the number of times the button was pressed

      attSeq[inpNo] = counter; // store the button press in the attSeq array
   
      counter = 0;
      timed_out = 0;
      inpNo++; // increment number of inputs
      } else 
      {
         printf("Wrong input,enter again: %d\n", counter);// if no button pressed, system asks user to enter again.
         counter = 0;
         timed_out = 0;
      }
    }
    blinkN(gpio, LED2, 2); // blink the second LED twice to indicate input completion
    delay(200);
    fprintf(stderr, "Your guess: ");
    for (int i = 0; i < seqlen; i++) // loop through the attSeq array to print the user's guess to console
    {
      printf(" %s ", color_names[attSeq[i] - 1]);
    }

    result = countMatches(theSeq, attSeq); // get the number of exact and approximate matches

    int exact = result[0];
    int approx = result[1];

    if (exact == 3)
    {
      found = 1;
      break;
    }

    printf("\nexact matches : %d and approx matches: %d\n", exact, approx);

    delay(200);
    lcdClear(lcd);
    char var[15];
    sprintf(var, "EXACT: %d", exact);
    lcdPosition(lcd, 0, 0);
    lcdPuts(lcd, var);

    sprintf(var, "APPROX: %d", approx);
    lcdPosition(lcd, 0, 1);
    lcdPuts(lcd, var);

    delay(300);
    blinkN(gpio, LED, exact);
    blinkN(gpio, LED2, 1);
    delay(150);
    blinkN(gpio, LED, approx);

    // end of round
    delay(400);
    blinkN(gpio, LED2, 3);

    lcdClear(lcd);
  
    
    /* ******************************************************* */
    /* ***  COMPLETE the code here  ***                        */
    /* this needs to implement the main loop of the game:      */
    /* check for button presses and count them                 */
    /* store the input numbers in the sequence @attSeq@        */
    /* compute the match with the secret sequence, and         */
    /* show the result                                         */
    /* see CW spec for details                                 */
    /* ******************************************************* */
  }
  if (found)// If a sequence is found
  {
    delay(800);
    fprintf(stderr, "SUCCESS");// Print success message to stderr
    lcdClear(lcd);
    lcdPosition(lcd, 0, 0);
    lcdPuts(lcd, "SUCCESS!");
    char var[25];
    sprintf(var, "Attempts:%d ", attempts);
    lcdPosition(lcd, 0, 1);
    lcdPuts(lcd, var);// Display attempts information on the LCD

    digitalWrite(gpio, LED2, HIGH);
    delay(200);
    blinkN(gpio, LED, 3);

    waitForEnter();

    digitalWrite(gpio, LED2, LOW);
    lcdClear(lcd);
    /* ***  COMPLETE the code here  ***  */
  }
  else// If no sequence is found
  {
    lcdClear(lcd);
    fprintf(stderr, "No sequence found\n");// Print message to stderr
    lcdPosition(lcd, 0, 0);
    lcdPuts(lcd, " SORRY :( ");
    lcdPosition(lcd, 0, 1);
    lcdPuts(lcd, "YOU LOST");
  }
 return 0;
 
}
