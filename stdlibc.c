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
/*                                 stdlibc                                   */
/*                                                                           */
/*   Parts of the standard C library needed to run SmallerC on MipsEmuElf.   */
/*                                                                           */
/*****************************************************************************/

#ifndef __SMALLER_C__

#define CONST const

typedef unsigned size_t;
typedef int ssize_t;
typedef void FILE;

#define TO_CHAR (char)
#define TO_INT (int)
#define TO_CHAR_PTR (char*)
#define TO_VOID_PTR_PTR (void**)
#define TO_FILE_PTR (FILE*)

#else // #ifndef __SMALLER_C__

#define CONST
#define size_t unsigned int
#define ssize_t signed int
#define FILE void

#define TO_CHAR
#define TO_INT
#define TO_CHAR_PTR
#define TO_VOID_PTR_PTR
#define TO_FILE_PTR

#endif

#define NULL 0
#define EOF (-1)

int isspace(int c)
{
  return c == ' ' || c == '\f' || c == '\n' || c == '\r' || c == '\t' || c == '\v';
}

int isdigit(int c)
{
  return c >= '0' && c <= '9';
}

int isalpha(int c)
{
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

int isalnum(int c)
{
  return isalpha(c) || isdigit(c);
}

int atoi(CONST char* s)
{
  int r = 0;
  while (isdigit(*s))
    r = r * 10 + *s++ - '0';
  return r;
}

size_t strlen(CONST char* str)
{
  CONST char* s;

  if (str == NULL)
    return 0;

  for (s = str; *s; s++);

  return s - str;
}

char* strcpy(char* dst, CONST char* src)
{
  char* p = dst;

  while ((*p++ = *src++) != 0);

  return dst;
}

char* strchr(CONST char* s, int c)
{
  char ch = TO_CHAR c;

  while (*s)
  {
    if (*s == ch)
      return TO_CHAR_PTR s;
    s++;
  }

  if (!ch)
    return TO_CHAR_PTR s;

  return NULL;
}

int strcmp(CONST char* s1, CONST char* s2)
{
  while (*s1 == *s2)
  {
    if (!*s1)
      return 0;
    s1++;
    s2++;
  }

  return (*s1 & 0xFF) - (*s2 & 0xFF);
}

int strncmp(CONST char* s1, CONST char* s2, size_t n)
{
  if (!n)
    return 0;

  do
  {
    if (*s1 != *s2++)
      return (*s1 & 0xFF) - (*--s2 & 0xFF);
    if (!*s1++)
      break;
  } while (--n);

  return 0;
}

void* memmove(void* dst, CONST void* src, size_t n)
{
  char* d = dst;
  char* s = TO_CHAR_PTR src;

  if (s < d)
  {
    s += n;
    d += n;
    while (n--)
      *--d = *--s;
  }
  else
  {
    while (n--)
      *d++ = *s++;
  }

  return dst;
}

void* memcpy(void* dst, CONST void* src, size_t n)
{
  char* p1 = dst;
  CONST char* p2 = src;

  while (n--)
    *p1++ = *p2++;

  return dst;
}

void* memset(void* s, int c, size_t n)
{
  char* p = s;

  while (n--)
    *p++ = c;

  return s;
}

extern int putchar(int c);
int fputc(int c, FILE* stream);

int puts(CONST char *s)
{
  int c;

  while ((c = *s++))
    putchar(c);
  return putchar('\n');
}

int __putchar__(char** buf, FILE* stream, int c)
{
  if (buf)
  {
    *(*buf)++ = c;
  }
  else if (stream)
  {
    fputc(c, stream);
  }
  else
  {
    putchar(c);
  }
  return 1;
}

int __vsprintf__(char** buf, FILE* stream, CONST char* fmt, void* vl)
{
  int* pp = vl;
  int cnt = 0;
  CONST char* p;
  CONST char* phex;
  char s[1/*sign*/+10/*magnitude*/+1/*\0*/]; // up to 11 octal digits in 32-bit numbers
  char* pc;
  int n, sign, msign;
  int minlen, len;
  int leadchar;

  for (p = fmt; *p != '\0'; p++)
  {
    if (*p != '%' || p[1] == '%')
    {
      __putchar__(buf, stream, *p);
      p = p + (*p == '%');
      cnt++;
      continue;
    }
    p++;
    minlen = 0;
    msign = 0;
    if (*p == '+') { msign = 1; p++; }
    else if (*p == '-') { msign = -1; p++; }
    if (isdigit(*p))
    {
      if (*p == '0')
        leadchar = '0';
      else
        leadchar = ' ';
      while (isdigit(*p))
        minlen = minlen * 10 + *p++ - '0';
      if (msign < 0)
        minlen = -minlen;
      msign = 0;
    }
    if (!msign)
    {
      if (*p == '+') { msign = 1; p++; }
      else if (*p == '-') { msign = -1; p++; }
    }
    phex = "0123456789abcdef";
    switch (*p)
    {
    case 'c':
      while (minlen > 1) { __putchar__(buf, stream, ' '); cnt++; minlen--; }
      __putchar__(buf, stream, *pp++);
      while (-minlen > 1) { __putchar__(buf, stream, ' '); cnt++; minlen++; }
      cnt++;
      break;
    case 's':
      pc = TO_CHAR_PTR *pp++;
      len = 0;
      if (pc)
        len = strlen(pc);
      while (minlen > len) { __putchar__(buf, stream, ' '); cnt++; minlen--; }
      if (len)
        while (*pc != '\0')
        {
          __putchar__(buf, stream, *pc++);
          cnt++;
        }
      while (-minlen > len) { __putchar__(buf, stream, ' '); cnt++; minlen++; }
      break;
    case 'i':
    case 'd':
      pc = &s[sizeof s - 1];
      *pc = '\0';
      len = 0;
      n = *pp++;
      sign = 1 - 2 * (n < 0);
      do
      {
        *--pc = '0' + (n - n / 10 * 10) * sign;
        n = n / 10;
        len++;
      } while (n);
      if (sign < 0)
      {
        *--pc = '-';
        len++;
      }
      else if (msign > 0)
      {
        *--pc = '+';
        len++;
        msign = 0;
      }
      while (minlen > len) { __putchar__(buf, stream, leadchar); cnt++; minlen--; }
      while (*pc != '\0')
      {
        __putchar__(buf, stream, *pc++);
        cnt++;
      }
      while (-minlen > len) { __putchar__(buf, stream, ' '); cnt++; minlen++; }
      break;
    case 'u':
      pc = &s[sizeof s - 1];
      *pc = '\0';
      len = 0;
      n = *pp++;
      do
      {
        unsigned nn = n;
        *--pc = '0' + nn % 10;
        n = nn / 10;
        len++;
      } while (n);
      if (msign > 0)
      {
        *--pc = '+';
        len++;
        msign = 0;
      }
      while (minlen > len) { __putchar__(buf, stream, leadchar); cnt++; minlen--; }
      while (*pc != '\0')
      {
        __putchar__(buf, stream, *pc++);
        cnt++;
      }
      while (-minlen > len) { __putchar__(buf, stream, ' '); cnt++; minlen++; }
      break;
    case 'X':
      phex = "0123456789ABCDEF";
      // fallthrough
    case 'p':
    case 'x':
      pc = &s[sizeof s - 1];
      *pc = '\0';
      len = 0;
      n = *pp++;
      do
      {
        *--pc = phex[n & 0xF];
        n = (n >> 4) & ((1 << (8 * sizeof n - 4)) - 1); // drop sign-extended bits
        len++;
      } while (n);
      while (minlen > len) { __putchar__(buf, stream, leadchar); cnt++; minlen--; }
      while (*pc != '\0')
      {
        __putchar__(buf, stream, *pc++);
        cnt++;
      }
      while (-minlen > len) { __putchar__(buf, stream, ' '); cnt++; minlen++; }
      break;
    case 'o':
      pc = &s[sizeof s - 1];
      *pc = '\0';
      len = 0;
      n = *pp++;
      do
      {
        *--pc = '0' + (n & 7);
        n = (n >> 3) & ((1 << (8 * sizeof n - 3)) - 1); // drop sign-extended bits
        len++;
      } while (n);
      while (minlen > len) { __putchar__(buf, stream, leadchar); cnt++; minlen--; }
      while (*pc != '\0')
      {
        __putchar__(buf, stream, *pc++);
        cnt++;
      }
      while (-minlen > len) { __putchar__(buf, stream, ' '); cnt++; minlen++; }
      break;
    default:
      return -1;
    }
  }

  return cnt;
}

int vprintf(CONST char* fmt, void* vl)
{
  return __vsprintf__(NULL, NULL, fmt, vl);
}

int printf(CONST char* fmt, ...)
{
  void** pp = TO_VOID_PTR_PTR &fmt;
  return vprintf(fmt, pp + 1);
}

int vsprintf(char* buf, CONST char* fmt, void* vl)
{
  return __vsprintf__(&buf, NULL, fmt, vl);
}

int sprintf(char* buf, CONST char* fmt, ...)
{
  void** pp = TO_VOID_PTR_PTR &fmt;
  return vsprintf(buf, fmt, pp + 1);
}

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

extern int open(CONST char*, int);
extern ssize_t read(int, void*, size_t);
extern ssize_t write(int, CONST void*, size_t);
extern int close(int);

FILE* fopen(CONST char* name, CONST char* mode)
{
  FILE* f = NULL;
  int m, fd;

  // This mapping is probably wrong:
  if (!strcmp(mode, "r") || !strcmp(mode, "rt"))
    m = O_TEXT | O_RDONLY;
  else if (!strcmp(mode, "w") || !strcmp(mode, "wt"))
    m = O_TEXT | O_TRUNC | O_CREAT | O_WRONLY;
  else if (!strcmp(mode, "a") || !strcmp(mode, "at"))
    m = O_TEXT | O_APPEND | O_WRONLY;
  else if (!strcmp(mode, "rb"))
    m = O_BINARY | O_RDONLY;
  else if (!strcmp(mode, "wb"))
    m = O_BINARY | O_TRUNC | O_CREAT | O_WRONLY;
  else if (!strcmp(mode, "ab"))
    m = O_BINARY | O_APPEND | O_WRONLY;
  else if (!strcmp(mode, "r+") || !strcmp(mode, "r+t") || !strcmp(mode, "rt+"))
    m = O_TEXT | O_RDWR;
  else if (!strcmp(mode, "w+") || !strcmp(mode, "w+t") || !strcmp(mode, "wt+"))
    m = O_TEXT | O_TRUNC | O_CREAT | O_RDWR;
  else if (!strcmp(mode, "a+") || !strcmp(mode, "a+t") || !strcmp(mode, "at+"))
    m = O_TEXT | O_APPEND | O_RDWR;
  else if (!strcmp(mode, "r+b") || !strcmp(mode, "rb+"))
    m = O_BINARY | O_RDWR;
  else if (!strcmp(mode, "w+b") || !strcmp(mode, "wb+"))
    m = O_BINARY | O_TRUNC | O_CREAT | O_RDWR;
  else if (!strcmp(mode, "a+b") || !strcmp(mode, "ab+"))
    m = O_BINARY | O_APPEND | O_RDWR;
  else
    return NULL;

  fd = open(name, m);

  if (fd >= 0)
    f = TO_FILE_PTR fd;

  return f;
}

size_t fread(void* ptr, size_t size, size_t count, FILE* stream)
{
  int fd = TO_INT stream;
  ssize_t sz;

  // check for zero or too large size * count
  if (!size ||
      !count ||
      size * count / count != size)
    return 0;

  count *= size;

  // check if the total size is greater than the maximum signed value of type ssize_t
  if (count > ((count - count - 1) >> 1))
    return 0;

  sz = read(fd, ptr, count);

  if (sz < 0)
    return 0;

  return sz / size;
}

size_t fwrite(CONST void* ptr, size_t size, size_t count, FILE* stream)
{
  int fd = TO_INT stream;
  ssize_t sz;

  // check for zero or too large size * count
  if (!size ||
      !count ||
      size * count / count != size)
    return 0;

  count *= size;

  // check if the total size is greater than the maximum signed value of type ssize_t
  if (count > ((count - count - 1) >> 1))
    return 0;

  sz = write(fd, ptr, count);

  if (sz < 0)
    return 0;

  return sz / size;
}

int fgetc(FILE* stream)
{
  char ch;

  if (fread(&ch, 1, 1, stream) == 1)
    return ch & 0xFF;

  return EOF;
}

int fputc(int c, FILE* stream)
{
  char ch = c;

  if (fwrite(&ch, 1, 1, stream) == 1)
    return ch & 0xFF;

  return EOF;
}

int fputs(CONST char* s, FILE* stream)
{
  size_t count = strlen(s);
  int fd = TO_INT stream;
  ssize_t sz = write(fd, s, count);

  if (sz < 0)
    return EOF;
  return 0;
}

int vfprintf(FILE* stream, CONST char* fmt, void* vl)
{
  return __vsprintf__(NULL, stream, fmt, vl);
}

int fprintf(FILE* stream, CONST char* fmt, ...)
{
  void** pp = TO_VOID_PTR_PTR &fmt;
  return vfprintf(stream, fmt, pp + 1);
}

int fclose(FILE* stream)
{
  return close(TO_INT stream);
}
