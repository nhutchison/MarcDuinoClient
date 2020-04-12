/*********************************
 * Marc Verdiell
 * Aug 18, 2013
 * A print library that resembles Arduino
 * No polymorphism in C, so naming of the print function changes
 ***************************************/

#ifndef Print_h
#define Print_h

#include <inttypes.h>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0

// private, don't call
void printNumber(unsigned long, uint8_t);
void print(uint8_t);

// call these
void printchar(char);
void printstr(char[]);
void printstr_p(const char *progmem_s ); //string from program memory
/******this is how to use it
#include <avr/pgmspace.h> 						// optional, already included in this header
const char string[] PROGMEM = "Hello World"; 	// like normal, but const before and PROGMEM after
printstr_p(string);								// print like normal, but use the _p version
**********************/
void printchar(char c);
void printstr(char c[]);
void printstr_p(const char *progmem_s );
void printuint(uint8_t n);
void printint(int);
void printlong(long);
void printulong(unsigned long);
void printbase(long n, int base);

void println(void);
void printlnchar(char);
void printlnstr(char[]);
void printlnstr_p(const char[]);	// string from program memory
void printlnuint(uint8_t);
void printlnint(int);
void printlnlong(long);
void printlnulong(unsigned long);
void printlnbase(long, int);

#endif
