/*********************************
 * Marc Verdiell
 * Aug 18, 2013
 * A print library that resembles Arduino
 * No polymorphism in C, so naming of the function changes
 ***************************************/

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include "serial.h"
#include "Print.h"

// Public Methods //////////////////////////////////////////////////////////////

void print(uint8_t b)
{
  // print on uart0 by default. Must be initialized firts
	serial_putc(b);
}

void printchar(char c)
{
  print((uint8_t) c);
}

void printstr(char c[])
{
  while (*c)
    print(*c++);
}

// to print from program memory
void printstr_p(const char *progmem_s )
{
    register char c;
    while ( (c = pgm_read_byte(progmem_s++)) )
      print(c);
}

void printuint(uint8_t n)
{
  printNumber(n, 10);
}

void printint(int n)
{
  printlong((long) n);
}

void printlong(long n)
{
  if (n < 0) {
    print('-');
    n = -n;
  }
  printNumber(n, 10);
}

void printulong(unsigned long n)
{
  printNumber(n, 10);
}

void printbase(long n, int base)
{
  if (base == 0)
    print((char) n);
  else
    printNumber(n, base);
}

void println(void)
{
  print('\r');
  print('\n');  
}

void printlnchar(char c)
{
  printchar(c);
  println();  
}

void printlnstr(char c[])
{
  printstr(c);
  println();
}

// same for string in program memory
void printlnstr_p(const char c[])
{
  printstr_p(c);
  println();
}


void printlnuint(uint8_t n)
{
	printNumber(n, 10);
	println();
}

void printlnint(int n)
{
  printint(n);
  println();
}

void printlnlong(long n)
{
  printlong(n);
  println();  
}

void printlnulong(unsigned long n)
{
  printulong(n);
  println();  
}

void printlnbase(long n, int base)
{
  printbase(n, base);
  println();
}

// Private Methods /////////////////////////////////////////////////////////////

void printNumber(unsigned long n, uint8_t base)
{
  unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
  unsigned long i = 0;

  if (n == 0) {
    print('0');
    return;
  } 

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    print((char) (buf[i - 1] < 10 ?
      '0' + buf[i - 1] :
      'A' + buf[i - 1] - 10));
}
