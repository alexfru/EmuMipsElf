/*
Copyright (c) 2013, Alexey Frunze
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met: 

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

/*****************************************************************************/
/*                                                                           */
/*                                 SPIM Test                                 */
/*                                                                           */
/*                  A short test application for EmuMipsElf.                 */
/*                                                                           */
/*****************************************************************************/

#ifndef __SMALLER_C__

#define CONST const

typedef unsigned size_t;
typedef int ssize_t;

#define TO_SIZE_T (size_t)

#else // #ifndef __SMALLER_C__

#define CONST

#define size_t unsigned int
#define ssize_t signed int

#define TO_SIZE_T

#endif

// flags in SPIM's file open():
//   Read = 0x0, Write = 0x1, Read/Write = 0x2
//   OR Create = 0x100, Truncate = 0x200, Append = 0x8
//   OR Text = 0x4000, Binary = 0x8000
#define O_RDONLY 0x0000
#define O_WRONLY 0x0001
#define O_RDWR   0x0002
#define O_APPEND 0x0008
#define O_CREAT  0x0100
#define O_TRUNC  0x0200
#define O_TEXT   0x4000
#define O_BINARY 0x8000

extern void __spim_print_int(int);
extern void __spim_print_string(CONST char*);
extern int __spim_read_int(void);
extern void __spim_read_string(char*, size_t);
extern void* __spim_sbrk(ssize_t);
extern void __spim_exit(int);
extern int __spim_print_char(int);
extern int __spim_read_char(void);
extern int __spim_open(CONST char*, int);
extern ssize_t __spim_read(int, void*, size_t);
extern ssize_t __spim_write(int, CONST void*, size_t);
extern int __spim_close(int);

void print_hex(unsigned n)
{
  if (n >= 16)
    print_hex(n / 16);
  __spim_print_char("0123456789ABCDEF"[n % 16]);
}

int main(int argc, char** argv)
{
  int i, fd;
  char ch;
  char s[64];

  __spim_print_string("Hello World from spimtst.c!\n\n");

  __spim_print_string("argc: ");
  __spim_print_int(argc);
  __spim_print_char('\n');

  for (i = 0; i < argc; i++)
  {
    __spim_print_string("argv[");
    __spim_print_int(i);
    __spim_print_string("]: \"");
    __spim_print_string(argv[i]);
    __spim_print_string("\"\n");
  }

  __spim_print_char('\n');

  __spim_print_string("__spim_sbrk(0x7FFFFFFF): "); print_hex(TO_SIZE_T __spim_sbrk(0x7FFFFFFF)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(0): "); print_hex(TO_SIZE_T __spim_sbrk(0)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(0): "); print_hex(TO_SIZE_T __spim_sbrk(0)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(1): "); print_hex(TO_SIZE_T __spim_sbrk(1)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(2): "); print_hex(TO_SIZE_T __spim_sbrk(2)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(4): "); print_hex(TO_SIZE_T __spim_sbrk(4)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(-8): "); print_hex(TO_SIZE_T __spim_sbrk(-8)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(-7): "); print_hex(TO_SIZE_T __spim_sbrk(-7)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(0): "); print_hex(TO_SIZE_T __spim_sbrk(0)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(0x7FFFF): "); print_hex(TO_SIZE_T __spim_sbrk(0x7FFFF)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(-0x7FFFF): "); print_hex(TO_SIZE_T __spim_sbrk(-0x7FFFF)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(0x80000): "); print_hex(TO_SIZE_T __spim_sbrk(0x80000)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(-0x80000): "); print_hex(TO_SIZE_T __spim_sbrk(-0x80000)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(0): "); print_hex(TO_SIZE_T __spim_sbrk(0)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(-0x80000): "); print_hex(TO_SIZE_T __spim_sbrk(-0x80000)); __spim_print_char('\n');
  __spim_print_string("__spim_sbrk(0): "); print_hex(TO_SIZE_T __spim_sbrk(0)); __spim_print_char('\n');

  __spim_print_char('\n');

  {
    char* p = __spim_sbrk(10);
    __spim_print_string("__spim_sbrk(10): "); print_hex(TO_SIZE_T p); __spim_print_char('\n');
    if (p)
    {
      for (i = 0; i < 10; i++)
        p[i] = '0' + i;
      for (i = 0; i < 10; i++)
        __spim_print_char(p[i]);
      __spim_print_char('\n');
      __spim_print_string("__spim_sbrk(-10): "); print_hex(TO_SIZE_T __spim_sbrk(-10)); __spim_print_char('\n');
      __spim_print_string("__spim_sbrk(0): "); print_hex(TO_SIZE_T __spim_sbrk(0)); __spim_print_char('\n');
    }
    __spim_print_char('\n');
  }

  __spim_print_string("Enter an integer number:\n");
  i = __spim_read_int();

  __spim_print_string("The number is: ");
  __spim_print_int(i);
  __spim_print_string("\n\n");

  __spim_print_string("Enter a string:\n");
  __spim_read_string(s, sizeof s);

  __spim_print_string("The string is: ");
  __spim_print_string(s);
  __spim_print_char('\n');

  __spim_print_string("Enter an ASCII character (will use __spim_read_char()):\n");
  i = __spim_read_char();
  __spim_read_char();

  __spim_print_string("The character is: ");
  __spim_print_char(i);
  __spim_print_char('\n');
  __spim_print_string("The character code is: ");
  __spim_print_int(i);
  __spim_print_string("\n\n");

  __spim_print_string("Enter another ASCII character (will use __spim_read() on stdin):\n");
  __spim_read(0, &ch, 1);
  __spim_read_char();

  __spim_print_string("The character is: ");
  __spim_print_char(ch);
  __spim_print_char('\n');
  __spim_print_string("The character code is: ");
  __spim_print_int(ch);
  __spim_print_string("\n\n");

  __spim_write(1, "This is printed using __spim_write() on stdout\n", sizeof "This is printed using __spim_write() on stdout\n" - 1);
  __spim_write(2, "This is printed using __spim_write() on stderr\n", sizeof "This is printed using __spim_write() on stderr\n" - 1);
  __spim_print_char('\n');

  __spim_print_string("Writing \"spim test\" to \"spimtst.txt\"\n");
  fd = __spim_open("spimtst.txt", O_BINARY | O_TRUNC | O_CREAT | O_WRONLY);
  __spim_write(fd, "spim test", sizeof "spim test" - 1);
  __spim_close(fd);

  __spim_print_string("Reading \"spim test\" back from \"spimtst.txt\"\n");
  fd = __spim_open("spimtst.txt", O_BINARY | O_RDONLY);
  i = __spim_read(fd, s, sizeof s - 1);
  __spim_close(fd);
  if (i < 0)
    i = 0;
  s[i] = '\0';

  __spim_print_string("The read back string is: \"");
  __spim_print_string(s);
  __spim_print_string("\"\n");

  __spim_exit(-42);
  return 123;
}
