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
/*                                EmuMipsElf                                 */
/*                                                                           */
/*       A simple 32-bit MIPS emulator with SPIM system calls support.       */
/*                                                                           */
/*  Supports 32-bit ELF executables and non-privileged integer instructions. */
/*                                                                           */
/*  MIPS16e, MIPS64, floating point, privileged/system instructions are not  */
/*                                supported.                                 */
/*                                                                           */
/*****************************************************************************/

#include <limits.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

typedef unsigned char uchar, uint8;
typedef signed char schar, int8;
typedef unsigned short ushort, uint16;
typedef short int16;
#if UINT_MAX >= 0xFFFFFFFF
typedef unsigned uint32;
typedef int int32;
#else
typedef unsigned long uint32;
typedef long int32;
#endif
typedef unsigned uint;
typedef unsigned long ulong;
typedef long long longlong;
typedef unsigned long long ulonglong;
#if ULONG_MAX >= 0xFFFFFFFFFFFFFFFFULL
typedef unsigned long uint64;
typedef long int64;
#else
typedef unsigned long long uint64;
typedef long long int64;
#endif

#define C_ASSERT(expr) extern char CAssertExtern[(expr)?1:-1]

C_ASSERT(CHAR_BIT == 8);
C_ASSERT(sizeof(uint16) == 2);
C_ASSERT(sizeof(uint32) == 4);
C_ASSERT(sizeof(uint64) == 8);
C_ASSERT(sizeof(size_t) >= 4);

#define MAX_FILES 16
FILE* files[MAX_FILES];

FILE* ElfFile = NULL;
char* SectNames = NULL;
uint32 EntryPointAddr = 0;
ulonglong EmulateCnt = 0;

void mycloseall(void);

void error(char* format, ...)
{
  va_list vl;
  va_start(vl, format);

  if (ElfFile)
    fclose(ElfFile);
  mycloseall();

  puts("");
  vprintf(format, vl);

  printf(" # %llu instruction(s) emulated\n\n", EmulateCnt);

  va_end(vl);
  exit(-1);
}

#pragma pack(push,1)

typedef struct
{
  uint8  e_ident[16];
  uint16 e_type;
  uint16 e_machine;
  uint32 e_version;
  uint32 e_entry;
  uint32 e_phoff;
  uint32 e_shoff;
  uint32 e_flags;
  uint16 e_ehsize;
  uint16 e_phentsize;
  uint16 e_phnum;
  uint16 e_shentsize;
  uint16 e_shnum;
  uint16 e_shstrndx;
} Elf32Hdr;

typedef struct
{
  uint32 sh_name;
  uint32 sh_type;
  uint32 sh_flags;
  uint32 sh_addr;
  uint32 sh_offset;
  uint32 sh_size;
  uint32 sh_link;
  uint32 sh_info;
  uint32 sh_addralign;
  uint32 sh_entsize;
} Elf32SectHdr;

typedef struct
{
  uint32 a_magic;   /* magic number */
#define OMAGIC 0407 /* old impure format */

  uint32 a_text;    /* size of text segment */
  uint32 a_data;    /* size of initialized data */
  uint32 a_bss;     /* size of uninitialized data */
  uint32 a_reltext; /* size of text relocation info */
  uint32 a_reldata; /* size of data relocation info */
  uint32 a_syms;    /* size of symbol table */
  uint32 a_entry;   /* entry point */
} AoutHdr;

#pragma pack(pop)

C_ASSERT(sizeof(Elf32Hdr) == 52);
C_ASSERT(sizeof(Elf32SectHdr) == 40);
C_ASSERT(sizeof(AoutHdr) == 32);

typedef struct
{
  void* Data;
  uint32 Addr;
  uint32 Size;
  uint32 Flags;
} tSection;

tSection* Sections = NULL;
uint SectionCnt = 0;

#define STACK_SIZE 0x10000
#define HEAP_SIZE 0x80000
C_ASSERT(STACK_SIZE >= 256 && STACK_SIZE < 0xFFFFFFFF && STACK_SIZE % 4 == 0); // less than 4GB, multiple of 4
C_ASSERT(HEAP_SIZE >= 256 && HEAP_SIZE < 0xFFFFFFFF && HEAP_SIZE % 4 == 0); // less than 4GB, multiple of 4
C_ASSERT(HEAP_SIZE < 0xFFFFFFFF - STACK_SIZE); // HEAP_SIZE + STACK_SIZE < 4GB

uint32 HeapStartAddr;
uint32 HeapSbrkAddr;

#define REG_SP 29
#define REG_RA 31
#define REG_LO 32
#define REG_HI 33
#define REG_PC 34
uint32 Regs[32 + 3];

uint32 minAddr = 0xFFFFFFFF;
uint32 maxAddr = 0;

int isElf(void)
{
  Elf32Hdr elfHdr;
  int is = fread(&elfHdr, 1, sizeof elfHdr, ElfFile) == sizeof elfHdr &&
           memcmp(elfHdr.e_ident, "\x7F""ELF", 4) == 0;
  if (fseek(ElfFile, 0, SEEK_SET))
    error("Can't read file\n");
  return is;
}

void loadElf(void)
{
  Elf32Hdr elfHdr;
  Elf32SectHdr sectHdr;
  uint idx;
  int unsupported = 0;

  if (fread(&elfHdr, 1, sizeof elfHdr, ElfFile) != sizeof elfHdr)
    error("Can't read file\n");

  if (memcmp(elfHdr.e_ident, "\x7F""ELF", 4))
    error("Not an ELF file\n");
  if (elfHdr.e_ident[6] != 1)
    error("Not a v1 ELF file\n");
  if (elfHdr.e_ehsize != sizeof elfHdr)
    error("Unexpected ELF header size\n");
  if (elfHdr.e_shentsize != sizeof sectHdr)
    error("Unexpected ELF section size\n");

  if (elfHdr.e_ident[4] != 1)
    error("Not a 32-bit file\n");
  if (elfHdr.e_ident[5] != 1)
    error("Not a little-endian file\n");
  if (elfHdr.e_type != 2)
    error("Not an executable file\n");
  if (elfHdr.e_machine != 8)
    error("Not a MIPS executable\n");

  if (fseek(ElfFile, elfHdr.e_shoff + elfHdr.e_shstrndx * sizeof sectHdr, SEEK_SET))
    error("Can't read file\n");
  if (fread(&sectHdr, 1, sizeof sectHdr, ElfFile) != sizeof sectHdr)
    error("Can't read file\n");

  if ((SectNames = malloc(sectHdr.sh_size)) == NULL)
    error("Out of memory\n");

  if (fseek(ElfFile, sectHdr.sh_offset, SEEK_SET))
    error("Can't read file\n");
  if (fread(SectNames, 1, sectHdr.sh_size, ElfFile) != sectHdr.sh_size)
    error("Can't read file\n");

  if ((Sections = malloc((elfHdr.e_shnum + 1) * sizeof(tSection))) == NULL)
    error("Out of memory\n");

  printf(" #  # Type     XAW   VirtAddr   FileOffs       Size Name\n");
  for (idx = 0; idx < elfHdr.e_shnum; idx++)
  {
    const char* typeName = "????????";
    const char* const typeNames[] =
    {
      "NULL",
      "PROGBITS",
      "SYMTAB",
      "STRTAB",
      "RELA",
      "HASH",
      "DYNAMIC",
      "NOTE",
      "NOBITS",
      "REL",
      "SHLIB",
      "DYNSYM",
    };
    const char* name = "";

    if (fseek(ElfFile, elfHdr.e_shoff + idx * sizeof sectHdr, SEEK_SET))
      error("Can't read file\n");
    if (fread(&sectHdr, 1, sizeof sectHdr, ElfFile) != sizeof sectHdr)
      error("Can't read file\n");
    if (sectHdr.sh_type == 0)
      memset(&sectHdr, 0, sizeof sectHdr);

    if (sectHdr.sh_name)
      name = SectNames + sectHdr.sh_name;

    unsupported |=
      (!strcmp(name, ".dynsym") ||
       !strcmp(name, ".dynstr") ||
       !strcmp(name, ".dynamic") ||
       !strcmp(name, ".hash") ||
       !strcmp(name, ".got") ||
       !strcmp(name, ".plt") ||
       sectHdr.sh_type == 5 || // SHT_HASH
       sectHdr.sh_type == 6 || // SHT_DYNAMIC
       sectHdr.sh_type == 11); // SHT_DYNSYM

    if (sectHdr.sh_type < sizeof typeNames / sizeof typeNames[0])
      typeName = typeNames[sectHdr.sh_type];

    printf(" # %2u %-8s %c%c%c 0x%08lX 0x%08lX %10lu %s\n",
           idx,
           typeName,
           "-X"[(sectHdr.sh_flags / 4) & 1],
           "-A"[(sectHdr.sh_flags / 2) & 1],
           "-W"[(sectHdr.sh_flags / 1) & 1],
           (ulong)sectHdr.sh_addr,
           (ulong)sectHdr.sh_offset,
           (ulong)sectHdr.sh_size,
           name);

    if ((sectHdr.sh_flags & 2) && sectHdr.sh_size) // SHF_ALLOC and size > 0
    {
      if ((Sections[SectionCnt].Data = malloc(sectHdr.sh_size)) == NULL)
        error("Out of memory\n");
      Sections[SectionCnt].Addr = sectHdr.sh_addr;
      Sections[SectionCnt].Size = sectHdr.sh_size;
      Sections[SectionCnt].Flags = (sectHdr.sh_flags & 1) | ((sectHdr.sh_flags & 4) >> 1); // bit0=Writable,bit1=eXecutable

      if (sectHdr.sh_type == 1) // SHT_PROGBITS
      {
        if (fseek(ElfFile, sectHdr.sh_offset, SEEK_SET))
          error("Can't read file\n");
        if (fread(Sections[SectionCnt].Data, 1, Sections[SectionCnt].Size, ElfFile) != Sections[SectionCnt].Size)
          error("Can't read file\n");
      }
      else // SHT_NOBITS, .bss, etc
      {
        memset(Sections[SectionCnt].Data, 0, Sections[SectionCnt].Size);
      }

      SectionCnt++;

      if (minAddr > sectHdr.sh_addr)
        minAddr = sectHdr.sh_addr;
      if (maxAddr < sectHdr.sh_addr + sectHdr.sh_size - 1)
        maxAddr = sectHdr.sh_addr + sectHdr.sh_size - 1;
    }
  }

  EntryPointAddr = elfHdr.e_entry;

  printf(" # Entry Point:    0x%08lX\n", (ulong)EntryPointAddr);
  puts("");

  if (unsupported)
    error("Dynamically linked or unsupported type of executable\n");
}

void loadAout(void)
{
  AoutHdr aoutHdr;

  if (fread(&aoutHdr, 1, sizeof aoutHdr, ElfFile) != sizeof aoutHdr)
    error("Can't read file\n");

  if (aoutHdr.a_magic != OMAGIC)
    error("Not an a.out file\n");

  if ((Sections = malloc((1 + 1) * sizeof(tSection))) == NULL)
    error("Out of memory\n");

#define USER_DATA_START 0x7F008000
#define MAXMEM (96*1024)
#define USER_DATA_END (USER_DATA_START + MAXMEM)

  Sections[SectionCnt].Addr = USER_DATA_START;
  Sections[SectionCnt].Size = aoutHdr.a_text + aoutHdr.a_data + aoutHdr.a_bss;
  Sections[SectionCnt].Flags = 3; // bit0=Writable,bit1=eXecutable

  printf(" #  # XAW   VirtAddr   FileOffs       Size Name\n");
  printf(" #  0 XAW 0x%08lX 0x%08lX %10lu .text + .data + .bss\n",
         (ulong)Sections[SectionCnt].Addr,
         (ulong)sizeof(AoutHdr),
         (ulong)Sections[SectionCnt].Size);

  if (Sections[SectionCnt].Size > MAXMEM)
    error("Section(s) bigger than %lu bytes\n", (ulong)MAXMEM);

  // Allow the entire address space in case library's (s)brk()
  // uses the space between _end and MAXMEM.

  Sections[SectionCnt].Size = MAXMEM;
  if ((Sections[SectionCnt].Data = calloc(1, Sections[SectionCnt].Size)) == NULL)
    error("Out of memory\n");

  if (fread(Sections[SectionCnt].Data, 1, aoutHdr.a_text + aoutHdr.a_data, ElfFile) != aoutHdr.a_text + aoutHdr.a_data)
    error("Can't read file\n");

  if (minAddr > Sections[SectionCnt].Addr)
    minAddr = Sections[SectionCnt].Addr;
  if (maxAddr < Sections[SectionCnt].Addr + Sections[SectionCnt].Size - 1)
    maxAddr = Sections[SectionCnt].Addr + Sections[SectionCnt].Size - 1;

  SectionCnt++;

  EntryPointAddr = aoutHdr.a_entry;

  printf(" # Entry: 0x%08lX\n", (ulong)EntryPointAddr);
  puts("");
}

void Prepare(int argc, char** argv)
{
  uint idx;
  uint32 heapAndStackSize = HEAP_SIZE + STACK_SIZE;
  uint32 cmdLineLen = 0;
  // Pseudo-section for the heap, stack and command line parameters

  // compute the size of argv[] and its strings
  for (idx = 0; idx < (uint)argc; idx++)
    cmdLineLen += strlen(argv[idx]) + 1;
  heapAndStackSize += (cmdLineLen + 3) / 4 * 4; // argc ASCIIZ strings from argv[] (round up to whole 32-bit words)
  heapAndStackSize += argc * 4; // argc pointers
  // TBD!!! Ideally, there should be argc+1 pointers (the last pointer being NULL)

  // allocate stack buffer
  if ((Sections[SectionCnt].Data = calloc(1, heapAndStackSize)) == NULL)
    error("Out of memory\n");

  // find a place for the heap and the stack in the address space
  if (maxAddr <= 0xFFFFFFFF - heapAndStackSize &&
      maxAddr + heapAndStackSize <= 0xFFFFFFF0)
    Sections[SectionCnt].Addr = -heapAndStackSize - 16; // top of address space
  else if (heapAndStackSize <= 0xFFFFFFFF - 65536 &&
           minAddr >= 65536 + heapAndStackSize)
    Sections[SectionCnt].Addr = 65536; // bottom of address space (64K reserved to catch NULL pointer dereferences)
  else
    error("Can't allocate heap and stack\n");
  Sections[SectionCnt].Size = heapAndStackSize;
  Sections[SectionCnt].Flags = 1; // Writable

  // set up argv[]
  cmdLineLen = 0;
  for (idx = 0; idx < (uint)argc; idx++)
  {
    uint32* ArgV = (uint32*)((char*)Sections[SectionCnt].Data + STACK_SIZE) + idx;
    char* param = (char*)Sections[SectionCnt].Data + STACK_SIZE + 4 * argc + cmdLineLen;
    strcpy(param, argv[idx]);
    cmdLineLen += strlen(argv[idx]) + 1;
    *ArgV = (param - (char*)Sections[SectionCnt].Data) + Sections[SectionCnt].Addr;
  }

  HeapSbrkAddr = HeapStartAddr =
    Sections[SectionCnt].Addr + STACK_SIZE + 4 * argc + (cmdLineLen + 3) / 4 * 4;

  Regs[REG_PC] = EntryPointAddr;
  Regs[REG_SP] = Sections[SectionCnt++].Addr + STACK_SIZE;
  Regs[4] = argc; // argc
  Regs[5] = Regs[REG_SP]; // argv
  Regs[6] = 0; // env
}

void Emulate(void);

int main(int argc, char** argv)
{
  uint32 ui32 = 0x44434241;
  uint16 ui16 = 0x3231;
  if (memcmp(&ui32, "ABCD", sizeof ui32) || memcmp(&ui16, "12", sizeof ui16))
    error("EmuMipsElf runs on little-endian platforms only\n");
  if (argc < 2 || !(ElfFile = fopen(argv[1], "rb")))
    error("Usage:\n  EmuMipsElf <mips32 elf executable or RetroBSD mips32 a.out executable> [params]\n");
  if (isElf())
    loadElf();
  else
    loadAout();
  Prepare(argc - 1, argv + 1);
  Emulate();
  fclose(ElfFile);
  return 0;
}

const char* AccessToStr(uint32 access, int allowed)
{
  switch (access + 4 * !!allowed)
  {
  case 0: return "R";
  case 1: return "W";
  case 2: return "X";

  case 4: return "R";
  case 5: return "RW";
  case 6: return "RX";
  case 7: return "RWX";
  }
  return "?";
}

void* TranslateAddr(uint32 Addr, uint32 Size, uint32 Access)
{
  uint i;
  void* p = NULL;
  for (i = 0; i < SectionCnt; i++)
    if (Sections[i].Addr <= Addr &&
        Addr <= (uint32)0 - Size &&
        Addr + Size - 1 <= Sections[i].Addr + Sections[i].Size - 1)
    {
      p = (char*)Sections[i].Data + Addr - Sections[i].Addr;
      break;
    }
  if (!p)
    error("Access violation at PC = 0x%08lX: Unmapped address (addr=0x%08lX,size=%lu,access=%s)\n",
          (ulong)Regs[REG_PC], (ulong)Addr, (ulong)Size, AccessToStr(Access, 0));
  if (Size > 1 && (Addr & (Size - 1)))
    error("Access violation at PC = 0x%08lX: Misaligned address (addr=0x%08lX,size=%lu,access=%s,allowed access=%s)\n",
          (ulong)Regs[REG_PC], (ulong)Addr, (ulong)Size, AccessToStr(Access, 0), AccessToStr(Sections[i].Flags, 1));
  if (Access && !(Sections[i].Flags & Access)) // if write or execute but read-only or non-executable
    error("Access violation at PC = 0x%08lX: Disallowed access (addr=0x%08lX,size=%lu,access=%s,allowed access=%s)\n",
          (ulong)Regs[REG_PC], (ulong)Addr, (ulong)Size, AccessToStr(Access, 0), AccessToStr(Sections[i].Flags, 1));
  return p;
}

uint8 ReadByte(uint32 Addr)
{
  return *(uint8*)TranslateAddr(Addr, 1, 0);
}
void WriteByte(uint32 Addr, uint8 Val)
{
  *(uint8*)TranslateAddr(Addr, 1, 1) = Val;
}

uint16 ReadHalfWord(uint32 Addr)
{
  return *(uint16*)TranslateAddr(Addr, 2, 0);
}
void WriteHalfWord(uint32 Addr, uint16 Val)
{
  *(uint16*)TranslateAddr(Addr, 2, 1) = Val;
}

uint32 ReadWord(uint32 Addr)
{
  return *(uint32*)TranslateAddr(Addr, 4, 0);
}
void WriteWord(uint32 Addr, uint32 Val)
{
  *(uint32*)TranslateAddr(Addr, 4, 1) = Val;
}

uint32 FetchProgramWord(uint32 Addr)
{
  return *(uint32*)TranslateAddr(Addr, 4, 2);
}

/*
  Supported instructions:

    add, addi, addiu, addu, and, andi,
    bal, beq, beql, bgez, bgezal, bgezall,
    bgezl, bgtz, bgtzl, blez, blezl, bltz,
    bltzal, bltzall, bltzl, bne, bnel, break,
    clo, clz,
    div, divu,
    ext,
    ins,
    j, jal, jalr, jr,
    lb, lbu, lh, lhu, lui, lw, lwl, lwr,
    madd, maddu, mfhi, mflo, movn, movz, msub,
    msubu, mthi, mtlo, mul, mult, multu,
    nop, nor,
    or, ori,
    rotr, rotrv,
    sb, seb, seh, sh, sll, sllv, slt, slti, sltiu,
    sltu, sra, srav, srl, srlv, sub, subu, sw,
    swl, swlr, synci, syscall,
    teq, teqi, tge, tgei, tgeiu, tgeu, tlt, tlti,
    tltiu, tltu, tne, tnei,
    wsbh,
    xor, xori

  Unsupported instructions:

    bc2f, bc2fl, bc2t, bc2tl,
    cache, cfc2, cop0, cop2, ctc2,
    deret, di,
    ehb, ei, eret,
    jalr.hb, jr.hb,
    ll, lwc2,
    mfc0, mfc2, mtc0, mtc2, mthc2,
    pref,
    rdhwr, rdpgpr,
    sc, sdbbp, ssnop, swc2, sync,
    wait, wrpgpr
*/

uint32 DoSysCall(uint32 instr);
void DoBreak(uint32 instr);
void DoTrap(uint32 instr);
void DoOverflow(void);
void DoInvalidInstruction(uint32 instr);

uint CountLeadingZeroes(uint32 n)
{
  uint c = 0;
  if (n == 0)
    return 32;
  while (n < 0x80000000)
    n <<= 1, c++;
  return c;
}

uint CountLeadingOnes(uint32 n)
{
  uint c = 0;
  while (n >= 0x80000000)
    n <<= 1, c++;
  return c;
}

uint32 ShiftRightArithm(uint32 n, uint32 c)
{
  uint32 s = -(n >> 31);
  n >>= c;
  n |= s << (31 - c) << 1;
  return n;
}

uint32 RotateRight(uint32 n, uint32 c)
{
  return (n >> c) | (n << (31 - c) << 1);
}

// Define this macro to do a more rigorous check
// for invalid/unsupported instructions
// (at the expense of performance, of course).
#define CHECK_INVALID_INSTR
void Emulate(void)
{
  int delaySlot = 0;
  uint32 postDelaySlotPc = 0;
  uint32 instr = 0;

  for (;;)
  {
    const uint32 pc = Regs[REG_PC];
    uint32 nextPc = pc + 4;
    /*const uint32*/ instr = FetchProgramWord(pc);
#if 0
    const uint32 op = instr >> 26;
    const uint32 r1 = (instr >> 21) & 0x1F;
    const uint32 r2 = (instr >> 16) & 0x1F;
    const uint32 r3 = (instr >> 11) & 0x1F;
    const uint32 shft = (instr >> 6) & 0x1F;
    const uint32 fxn = instr & 0x3F;
    const uint32 imm16 = instr & 0xFFFF;
    const uint32 simm16 = (int16)imm16;
    const uint32 jtgt = instr & 0x3FFFFFF;
#else
#define op     (instr >> 26)
#define r1     ((instr >> 21) & 0x1F)
#define r2     ((instr >> 16) & 0x1F)
#define r3     ((instr >> 11) & 0x1F)
#define shft   ((instr >> 6) & 0x1F)
#define fxn    (instr & 0x3F)
#define imm16  (instr & 0xFFFF)
#define simm16 ((int16)imm16)
#define jtgt   (instr & 0x3FFFFFF)
#endif

    switch (op)
    {
    case 0:
      switch (fxn)
      {
      case 0:
#ifdef CHECK_INVALID_INSTR
        if (r1)
          goto lInvalidInstruction;
#endif
        Regs[r3] = Regs[r2] << shft;
        break; // sll d,w,shft
      case 2:
        switch (r1)
        {
        case 0: Regs[r3] = Regs[r2] >> shft; break; // srl d,w,shft
        case 1: Regs[r3] = RotateRight(Regs[r2], shft); break; // rotr d,w,shft
        default: goto lInvalidInstruction;
        }
        break;
      case 3:
#ifdef CHECK_INVALID_INSTR
        if (r1)
          goto lInvalidInstruction;
#endif
        Regs[r3] = ShiftRightArithm(Regs[r2], shft);
        break; // sra d,w,shft
      case 4:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = Regs[r2] << (Regs[r1] & 31);
        break; // sllv d,w,s
      case 6:
        switch (shft)
        {
        case 0: Regs[r3] = Regs[r2] >> (Regs[r1] & 31); break; // srlv d,w,s
        case 1: Regs[r3] = RotateRight(Regs[r2], Regs[r1] & 31); break; // rotrv d,w,s
        default: goto lInvalidInstruction;
        }
        break;
      case 7:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = ShiftRightArithm(Regs[r2], Regs[r1] & 31);
        break; // srav d,w,s
      case 8:
#ifdef CHECK_INVALID_INSTR
        if (r2 | r3 | shft)
          goto lInvalidInstruction;
#endif
        nextPc = Regs[r1]; delaySlot = 1;
        break; // jr s
      case 9:
#ifdef CHECK_INVALID_INSTR
        if (r2 | shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = nextPc + 4; nextPc = Regs[r1]; delaySlot = 1;
        break; // jalr [d,] s
      case 10:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        if (Regs[r2] == 0) Regs[r3] = Regs[r1];
        break; // movz d,s,t
      case 11:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        if (Regs[r2]) Regs[r3] = Regs[r1];
        break; // movn d,s,t
      case 12: 
        {
          // RetroBSD may advance PC on returning from a syscall handler,
          // skipping 2 instructions that follow the syscall instruction.
          // Those 2 instructions typically set C's errno variable and
          // are either executed on error or skipped on success.
          // Account for this peculiarity.
          uint32 skip = DoSysCall(instr);
          nextPc += skip * 4;
        }
        break; // syscall code
      case 13: goto lBreak; break; // break code
      case 16:
#ifdef CHECK_INVALID_INSTR
        if (r1 | r2 | shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = Regs[REG_HI];
        break; // mfhi d
      case 17:
#ifdef CHECK_INVALID_INSTR
        if (r2 | r3 | shft)
          goto lInvalidInstruction;
#endif
        Regs[REG_HI] = Regs[r1];
        break; // mthi s
      case 18:
#ifdef CHECK_INVALID_INSTR
        if (r1 | r2 | shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = Regs[REG_LO];
        break; // mflo d
      case 19:
#ifdef CHECK_INVALID_INSTR
        if (r2 | r3 | shft)
          goto lInvalidInstruction;
#endif
        Regs[REG_LO] = Regs[r1];
        break; // mtlo s
      case 24:
#ifdef CHECK_INVALID_INSTR
        if (r3 | shft)
          goto lInvalidInstruction;
#endif
        {
          int64 p = (int64)(int32)Regs[r1] * (int32)Regs[r2];
          Regs[REG_LO] = (uint32)p;
          Regs[REG_HI] = (uint32)(p >> 32);
        }
        break; // mult s,t
      case 25:
#ifdef CHECK_INVALID_INSTR
        if (r3 | shft)
          goto lInvalidInstruction;
#endif
        {
          uint64 p = (uint64)Regs[r1] * Regs[r2];
          Regs[REG_LO] = (uint32)p;
          Regs[REG_HI] = (uint32)(p >> 32);
        }
        break; // multu s,t
      case 26:
#ifdef CHECK_INVALID_INSTR
        if (r3 | shft)
          goto lInvalidInstruction;
#endif
        if (!(Regs[r2] == 0 || (Regs[r1] == 0x80000000 && Regs[r2] == 0xFFFFFFFF)))
          Regs[REG_LO] = (int32)Regs[r1] / (int32)Regs[r2], Regs[REG_HI] = (int32)Regs[r1] % (int32)Regs[r2];
        break; // div s,t
      case 27:
#ifdef CHECK_INVALID_INSTR
        if (r3 | shft)
          goto lInvalidInstruction;
#endif
        if (Regs[r2])
          Regs[REG_LO] = Regs[r1] / Regs[r2], Regs[REG_HI] = Regs[r1] % Regs[r2];
        break; // divu s,t
      case 32:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        {
          uint32 sum = Regs[r1] + Regs[r2];
          if (((Regs[r1] ^ Regs[r2] ^ 0x80000000) & 0x80000000) &&
              ((sum ^ Regs[r1]) & 0x80000000))
            goto lOverflow;
          Regs[r3] = sum;
        }
        break; // add d,s,t
      case 33:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = Regs[r1] + Regs[r2];
        break; // addu d,s,t
      case 34:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        {
          uint32 diff = Regs[r1] - Regs[r2];
          if (((Regs[r1] ^ Regs[r2]) & 0x80000000) &&
              ((diff ^ Regs[r1]) & 0x80000000))
            goto lOverflow;
          Regs[r3] = diff;
        }
        break; // sub d,s,t
      case 35:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = Regs[r1] - Regs[r2];
        break; // subu d,s,t
      case 36:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = Regs[r1] & Regs[r2];
        break; // and d,s,t
      case 37:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = Regs[r1] | Regs[r2];
        break; // or d,s,t
      case 38:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = Regs[r1] ^ Regs[r2];
        break; // xor d,s,t
      case 39:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = ~(Regs[r1] | Regs[r2]);
        break; // nor d,s,t
      case 42:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = (int32)Regs[r1] < (int32)Regs[r2];
        break; // slt d,s,t
      case 43:
#ifdef CHECK_INVALID_INSTR
        if (shft)
          goto lInvalidInstruction;
#endif
        Regs[r3] = Regs[r1] < Regs[r2];
        break; // sltu d,s,t

      case 48: if ((int32)Regs[r1] >= (int32)Regs[r2]) goto lTrap; break; // tge s,t
      case 49: if (Regs[r1] >= Regs[r2]) goto lTrap; break; // tgeu s,t
      case 50: if ((int32)Regs[r1] < (int32)Regs[r2]) goto lTrap; break; // tlt s,t
      case 51: if (Regs[r1] < Regs[r2]) goto lTrap; break; // tltu s,t
      case 52: if (Regs[r1] == Regs[r2]) goto lTrap; break; // teq s,t
      case 53: if (Regs[r1] != Regs[r2]) goto lTrap; break; // tne s,t

      default: goto lInvalidInstruction;
      }
      break;

    case 1:
      switch (r2)
      {
      case 0: if ((int32)Regs[r1] < 0) nextPc += simm16 << 2, delaySlot = 1; break; // bltz s,p
      case 1: if ((int32)Regs[r1] >= 0) nextPc += simm16 << 2, delaySlot = 1; break; // bgez s,p
      case 2: if ((int32)Regs[r1] < 0) nextPc += simm16 << 2, delaySlot = 1; else nextPc += 4; break; // bltzl s,p
      case 3: if ((int32)Regs[r1] >= 0) nextPc += simm16 << 2, delaySlot = 1; else nextPc += 4; break; // bgezl s,p

      case 8: if ((int32)Regs[r1] >= (int32)simm16) goto lTrap; break; // tgei s,j
      case 9: if (Regs[r1] >= (uint32)simm16) goto lTrap; break; // tgeiu s,j
      case 10: if ((int32)Regs[r1] < (int32)simm16) goto lTrap; break; // tlti s,j
      case 11: if (Regs[r1] < (uint32)simm16) goto lTrap; break; // tltiu s,j
      case 12: if (Regs[r1] == (uint32)simm16) goto lTrap; break; // teqi s,j
      case 14: if (Regs[r1] != (uint32)simm16) goto lTrap; break; // tnei s,j

      case 16: Regs[REG_RA] = nextPc + 4; if ((int32)Regs[r1] < 0) nextPc += simm16 << 2, delaySlot = 1; break; // bltzal s,p
      case 17: Regs[REG_RA] = nextPc + 4; if ((int32)Regs[r1] >= 0) nextPc += simm16 << 2, delaySlot = 1; break; // bgezal s,p
      case 18: Regs[REG_RA] = nextPc + 4; if ((int32)Regs[r1] < 0) nextPc += simm16 << 2, delaySlot = 1; else nextPc += 4; break; // bltzall s,p
      case 19: Regs[REG_RA] = nextPc + 4; if ((int32)Regs[r1] >= 0) nextPc += simm16 << 2, delaySlot = 1; else nextPc += 4; break; // bgezall s,p

      case 31: break; // synci o(b)

      default: goto lInvalidInstruction;
      }
      break;

    case 2: nextPc = (pc & 0xF0000000) | (jtgt << 2); delaySlot = 1; break; // j target
    case 3: Regs[REG_RA] = nextPc + 4; nextPc = (pc & 0xF0000000) | (jtgt << 2); delaySlot = 1; break; // jal target

    case 4: if (Regs[r1] == Regs[r2]) nextPc += simm16 << 2, delaySlot = 1; break; // beq s,t,p
    case 5: if (Regs[r1] != Regs[r2]) nextPc += simm16 << 2, delaySlot = 1; break; // bne s,t,p
    case 6:
#ifdef CHECK_INVALID_INSTR
      if (r2)
        goto lInvalidInstruction;
#endif
      if ((int32)Regs[r1] <= 0) nextPc += simm16 << 2, delaySlot = 1; break; // blez s,p
    case 7:
#ifdef CHECK_INVALID_INSTR
      if (r2)
        goto lInvalidInstruction;
#endif
      if ((int32)Regs[r1] > 0) nextPc += simm16 << 2, delaySlot = 1; break; // bgtz s,p

    case 8:
      {
        uint32 sum = Regs[r1] + simm16;
        if (((Regs[r1] ^ simm16 ^ 0x80000000) & 0x80000000) &&
            ((sum ^ Regs[r1]) & 0x80000000))
          goto lOverflow;
        Regs[r2] = sum;
      }
      break; // addi d,s,const
    case 9: Regs[r2] = Regs[r1] + simm16; break; // addiu d,s,const
    case 10: Regs[r2] = (int32)Regs[r1] < (int32)simm16; break; // slti d,s,const
    case 11: Regs[r2] = Regs[r1] < imm16; break; // sltiu d,s,const
    case 12: Regs[r2] = Regs[r1] & imm16; break; // andi d,s,const
    case 13: Regs[r2] = Regs[r1] | imm16; break; // ori d,s,const
    case 14: Regs[r2] = Regs[r1] ^ imm16; break; // xori d,s,const
    case 15:
#ifdef CHECK_INVALID_INSTR
      if (r1)
        goto lInvalidInstruction;
#endif
      Regs[r2] = imm16 << 16; break; // lui d,const

    case 20: if (Regs[r1] == Regs[r2]) nextPc += simm16 << 2, delaySlot = 1; else nextPc += 4; break; // beql s,t,p
    case 21: if (Regs[r1] != Regs[r2]) nextPc += simm16 << 2, delaySlot = 1; else nextPc += 4; break; // bnel s,t,p
    case 22:
#ifdef CHECK_INVALID_INSTR
      if (r2)
        goto lInvalidInstruction;
#endif
      if ((int32)Regs[r1] <= 0) nextPc += simm16 << 2, delaySlot = 1; else nextPc += 4; break; // blezl s,p
    case 23:
#ifdef CHECK_INVALID_INSTR
      if (r2)
        goto lInvalidInstruction;
#endif
      if ((int32)Regs[r1] > 0) nextPc += simm16 << 2, delaySlot = 1; else nextPc += 4; break; // bgtzl s,p

    case 28:
#ifdef CHECK_INVALID_INSTR
      if (shft)
        goto lInvalidInstruction;
#endif
      switch (fxn)
      {
      case 0:
#ifdef CHECK_INVALID_INSTR
        if (r3)
          goto lInvalidInstruction;
#endif
        {
          int64 p = (int64)(int32)Regs[r1] * (int32)Regs[r2];
          if (Regs[REG_LO] > 0xFFFFFFFF - (uint32)p)
            Regs[REG_HI]++;
          Regs[REG_LO] += (uint32)p;
          Regs[REG_HI] += (uint32)(p >> 32);
        }
        break; // madd s,t
      case 1:
#ifdef CHECK_INVALID_INSTR
        if (r3)
          goto lInvalidInstruction;
#endif
        {
          uint64 p = (uint64)Regs[r1] * Regs[r2];
          if (Regs[REG_LO] > 0xFFFFFFFF - (uint32)p)
            Regs[REG_HI]++;
          Regs[REG_LO] += (uint32)p;
          Regs[REG_HI] += (uint32)(p >> 32);
        }
        break; // maddu s,t
      case 2: Regs[r3] = Regs[r1] * Regs[r2]; break; // mul d,s,t
      case 4:
#ifdef CHECK_INVALID_INSTR
        if (r3)
          goto lInvalidInstruction;
#endif
        {
          int64 p = (int64)(int32)Regs[r1] * (int32)Regs[r2];
          if (Regs[REG_LO] < (uint32)p)
            Regs[REG_HI]--;
          Regs[REG_LO] -= (uint32)p;
          Regs[REG_HI] -= (uint32)(p >> 32);
        }
        break; // msub s,t
      case 5:
#ifdef CHECK_INVALID_INSTR
        if (r3)
          goto lInvalidInstruction;
#endif
        {
          uint64 p = (uint64)Regs[r1] * Regs[r2];
          if (Regs[REG_LO] < (uint32)p)
            Regs[REG_HI]--;
          Regs[REG_LO] -= (uint32)p;
          Regs[REG_HI] -= (uint32)(p >> 32);
        }
        break; // msubu s,t

      case 32:
#ifdef CHECK_INVALID_INSTR
        if (r1 != r2)
          goto lInvalidInstruction;
#endif
        Regs[r3] = CountLeadingZeroes(Regs[r2]); break; // clz d,s
      case 33:
#ifdef CHECK_INVALID_INSTR
        if (r1 != r2)
          goto lInvalidInstruction;
#endif
        Regs[r3] = CountLeadingOnes(Regs[r2]); break; // clo d,s

      default: goto lInvalidInstruction;
      }
      break;

    case 31:
      switch (fxn)
      {
      case 0:
        if (shft + r3 <= 31)
        {
          uint size = r3 + 1;
          uint32 mask = (0xFFFFFFFF >> (32 - size)) << shft;
          Regs[r2] = (Regs[r1] & mask) >> shft;
        }
        break; // ext t,s,pos,sz
      case 4:
        if (r3 >= shft)
        {
          uint size = r3 - shft + 1;
          uint32 mask = (0xFFFFFFFF >> (32 - size)) << shft;
          Regs[r2] = (Regs[r2] & ~mask) | ((Regs[r1] << shft) & mask);
        }
        break; // ins t,s,pos,sz
      case 32:
#ifdef CHECK_INVALID_INSTR
        if (r1)
          goto lInvalidInstruction;
#endif
        switch (shft)
        {
        case 2: Regs[r3] = ((Regs[r2] & 0x00FF) << 8) |
                           ((Regs[r2] & 0xFF00) >> 8) |
                           ((Regs[r2] & 0x00FF0000) << 8) |
                           ((Regs[r2] & 0xFF000000) >> 8); break; // wsbh d,t
        case 16: Regs[r3] = (int8)Regs[r2]; break; // seb d,t
        case 24: Regs[r3] = (int16)Regs[r2]; break; // seh d,t
        default: goto lInvalidInstruction;
        }
        break;

      default: goto lInvalidInstruction;
      }
      break;

    case 32: Regs[r2] = (int8)ReadByte(Regs[r1] + simm16); break; // lb t,o(b)
    case 33: Regs[r2] = (int16)ReadHalfWord(Regs[r1] + simm16); break; // lh t,o(b)
    case 34:
      {
        uint32 v = ReadByte(Regs[r1] + simm16);
        v = (v << 8) | ReadByte(Regs[r1] + simm16 - 1);
        Regs[r2] = (Regs[r2] & 0xFFFF) | (v << 16);
      }
      break; // lwl t,o(b)
    case 35: Regs[r2] = ReadWord(Regs[r1] + simm16); break; // lw t,o(b)
    case 36: Regs[r2] = ReadByte(Regs[r1] + simm16); break; // lbu t,o(b)
    case 37: Regs[r2] = ReadHalfWord(Regs[r1] + simm16); break; // lhu t,o(b)
    case 38:
      {
        uint32 v = ReadByte(Regs[r1] + simm16);
        v |= (uint32)ReadByte(Regs[r1] + simm16 + 1) << 8;
        Regs[r2] = (Regs[r2] & 0xFFFF0000) | v;
      }
      break; // lwr t,o(b)

    case 40: WriteByte(Regs[r1] + simm16, (uint8)Regs[r2]); break; // sb t,o(b)
    case 41: WriteHalfWord(Regs[r1] + simm16, (uint16)Regs[r2]); break; // sh t,o(b)
    case 42:
      WriteByte(Regs[r1] + simm16, (uint8)(Regs[r2] >> 24));
      WriteByte(Regs[r1] + simm16 - 1, (uint8)(Regs[r2] >> 16));
      break; // swl t,o(b)
    case 43: WriteWord(Regs[r1] + simm16, Regs[r2]); break; // sw t,o(b)
    case 46:
      WriteByte(Regs[r1] + simm16, (uint8)Regs[r2]);
      WriteByte(Regs[r1] + simm16 + 1, (uint8)(Regs[r2] >> 8));
      break; // swr t,o(b)

    default:
      goto lInvalidInstruction;
    }

    Regs[0] = 0;

    Regs[REG_PC] = nextPc;

    if (delaySlot)
    {
      if (delaySlot == 1)
      {
         postDelaySlotPc = nextPc;
         Regs[REG_PC] = pc + 4;
         delaySlot = 2;
      }
      else
      {
         Regs[REG_PC] = postDelaySlotPc;
         delaySlot = 0;
      }
    }

    EmulateCnt++;
  } // for (;;)

lBreak:
  DoBreak(instr);
  return;

lTrap:
  DoTrap(instr);
  return;

lOverflow:
  DoOverflow();
  return;

lInvalidInstruction:
  DoInvalidInstruction(instr);
  return;
#if 01
#undef op
#undef r1
#undef r2
#undef r3
#undef shft
#undef fxn
#undef imm16
#undef simm16
#undef jtgt
#endif
}

int myfopen(const char* name, const char* mode)
{
  int i;
  for (i = 0; i < MAX_FILES; i++)
    if (files[i] == NULL &&
        (files[i] = fopen(name, mode)) != NULL)
      return 16 + i;
  return -1;
}

int myfclose(int fd)
{
  fd -= 16;
  if (fd >= 0 && fd < MAX_FILES && files[fd] != NULL)
  {
    fclose(files[fd]);
    files[fd] = NULL;
    return 0;
  }
  return -1;
}

FILE* myfdopen(int fd)
{
  fd -= 16;
  if (fd >= 0 && fd < MAX_FILES)
    return files[fd];
  return NULL;
}

void mycloseall(void)
{
  int i;
  for (i = 0; i < MAX_FILES; i++)
    if (files[i] != NULL)
      fclose(files[i]);
}

uint32 DoSysCall(uint32 instr)
{
  uint32 code = (instr >> 6) & 0xFFFFF;
  int retroBSD = code != 0;

  if (!retroBSD)
  {
    // Emulate selected SPIM system calls
    switch (Regs[2])
    {
    case 1: // print int
      {
        long n = (int32)Regs[4];
        printf("%ld", n);
      }
      break;

    // case 2: // print float
    // case 3: // print double

    case 4: // print ASCIIZ string
      {
        uint32 addr = Regs[4];
        int ch;
        while ((ch = ReadByte(addr++)) != '\0')
          putchar(ch);
      }
      break;

    case 5: // read int
      {
        char s[128], *p = s, sgn = '+';
        uint32 n = 0;

        if (fgets(s, sizeof s, stdin) == NULL)
          s[0] = '\0';

        while (*p != '\0' && isspace(*p))
          p++;

        if (*p == '+' || *p == '-')
          sgn = *p++;

        while (*p != '\0' && isdigit(*p))
          n = n * 10 + *p++ - '0';

        if (sgn == '-')
          n = -n;

        Regs[2] = n;
      }
      break;

    // case 6: // read float
    // case 7: // read double

    case 8: // read string/fgets(buffer, size, stdin)
      {
        uint32 addr = Regs[4];
        size_t l = Regs[5];
        char* p;

        if (l == 0 || l > INT_MAX - 1u)
          error("\nOut of memory at PC = 0x%08lX\n", (ulong)Regs[REG_PC]);

        // first, "probe" the memory to avoid crashing
        for (l = 0; l < Regs[5]; l++)
          WriteByte(addr + l, ReadByte(addr + l));

        p = TranslateAddr(addr, 1, 1);

        if (fgets(p, Regs[5], stdin) == NULL)
          p[0] = '\0';
      }
      break;

    case 9: // sbrk()
    _sbrk:
      {
        uint32 increment = Regs[4];
        uint32 oldAddr = HeapSbrkAddr;

        if (increment <= 0x7FFFFFFF) // non-negative increment
        {
          if (increment == 0 ||
              (HeapSbrkAddr <= 0xFFFFFFFF - increment &&
               HeapSbrkAddr + increment - HeapStartAddr <= HEAP_SIZE))
            HeapSbrkAddr += increment;
          else
            oldAddr = -1;
        }
        else // negative increment
        {
          if (HeapSbrkAddr - HeapStartAddr >= -increment)
            HeapSbrkAddr += increment;
          else
            oldAddr = -1;
        }

        Regs[2] = oldAddr;
      }
      break;

    case 10: // exit()
    case 17: // exit2()
    _exit:
      error("\n # Program terminated with code %ld\n", (long)(int32)Regs[4]);
      break;

    case 11: // putchar()
      putchar(Regs[4] & 0xFF);
      break;

    case 12: // getchar()
      Regs[2] = getchar();
      break;

    case 13: // open()
    _open:
      {
        uint32 flags = Regs[5];
        // uint32 umode = Regs[6]; // UNIX octal file mode (e.g. 0644 for rw-r--r--), ignored
        const char* cmode;

        // flags in SPIM's file open():
        //   Read = 0x0, Write = 0x1, Read/Write = 0x2
        //   OR Create = 0x100, Truncate = 0x200, Append = 0x8
        //   OR Text = 0x4000, Binary = 0x8000

        // This mapping is probably wrong:
        //  c/t  w  r/w  a
        //   0   0   0   0   r
        //   0   0   0   1   a (?)
        //   0   0   1   0   r+
        //   0   0   1   1   a+ (?)
        //   0   1   0   0   r+ (?)
        //   0   1   0   1   a (?)
        //   0   1   1   0   r+
        //   0   1   1   1   a+ (?)
        //   1   0   0   0   error (?)
        //   1   0   0   1   w (?)
        //   1   0   1   0   w+
        //   1   0   1   1   w+ (?)
        //   1   1   0   0   w
        //   1   1   0   1   w (?)
        //   1   1   1   0   w+
        //   1   1   1   1   w+ (?)
        // c/t = create/truncate
        static const char* const cmodes[2][16] =
        {
          { "rb","ab","r+b","a+b","r+b","ab","r+b","a+b",NULL,"wb","w+b","w+b","wb","wb","w+b","w+b" },
          { "r", "a", "r+", "a+", "r+", "a", "r+", "a+", NULL,"w", "w+", "w+", "w", "w", "w+", "w+" }
        };

        cmode = cmodes[(flags & 0x4000) != 0]
                      [((flags & 0x200) >> 6) | ((flags & 0x100) >> 5) |
                       ((flags & 0x8) >> 3) |
                       ((flags & 0x1) << 2) | (flags & 0x2)];

        Regs[2] = -1;

        if (cmode != NULL)
        {
          const char* name;
          uint32 addr = Regs[4];
          int ch;

          // first, "probe" the memory to avoid crashing
          while ((ch = ReadByte(addr++)) != '\0');

          // now, open
          name = TranslateAddr(Regs[4], 1, 0);
          Regs[2] = myfopen(name, cmode);
        }
      }
      break;

    case 14: // read()
    _read:
      {
        uint32 addr = Regs[5];
        size_t l, cnt = -1;
        void* p;
        FILE* f = NULL;

        // first, "probe" the memory to avoid crashing
        for (l = 0; l < Regs[6]; l++)
          WriteByte(addr + l, ReadByte(addr + l));

        p = TranslateAddr(addr, 1, 1);

        // descriptor 0 = stdin
        if (Regs[4] == 0)
          f = stdin;
        else if (Regs[4] > 2)
          f = myfdopen(Regs[4]);

        if (f != NULL)
          cnt = fread(p, 1, Regs[6], f);

        Regs[2] = cnt;
      }
      break;

    case 15: // write()
    _write:
      {
        uint32 addr = Regs[5];
        size_t l, cnt = -1;
        void* p;
        FILE* f = NULL;

        // first, "probe" the memory to avoid crashing
        for (l = 0; l < Regs[6]; l++)
          ReadByte(addr + l);

        p = TranslateAddr(addr, 1, 0);

        // descriptor 1 = stdout
        // descriptor 2 = stderr
        if (Regs[4] == 1)
          f = stdout;
        else if (Regs[4] == 2)
          f = stderr;
        else if (Regs[4] > 2)
          f = myfdopen(Regs[4]);

        if (f != NULL)
          cnt = fwrite(p, 1, Regs[6], f);

        Regs[2] = cnt;
      }
      break;

    case 16: // close()
    _close:
      if (Regs[4] > 2)
        myfclose(Regs[4]);
      break;

    case 18: // lseek(), not a SPIM syscall
    _lseek:
      {
        FILE* f = NULL;
        long pos = -1;
        // Most typical SEEK_xxx values:
        // SEEK_SET = 0
        // SEEK_CUR = 1
        // SEEK_END = 2
        if (Regs[4] > 2 && Regs[6] < 3)
          if ((f = myfdopen(Regs[4])) != NULL)
            if (!fseek(f, Regs[5], Regs[6]))
              pos = ftell(f);
        Regs[2] = pos;
      }
      break;

    default:
    _default:
      error("\nUnsupported syscall (code: 0x%05lX, V0 = 0x%08lX) at PC = 0x%08lX\n", (ulong)code, (ulong)Regs[2], (ulong)Regs[REG_PC]);
    } // endof switch (Regs[2])
  } // endof if (code == 0)
  else // elseof if (!retroBSD)
  {
    // Map RetroBSD's system calls onto SPIM system calls
#define SYS_exit        1
#define SYS_read        3
#define SYS_write       4
#define SYS_open        5
#define SYS_close       6
#define SYS_lseek       19
#define SYS_sbrk        69
    switch (code)
    {
    case SYS_exit:  goto _exit;
    case SYS_read:  goto _read;
    case SYS_write: goto _write;
    case SYS_open:  goto _open;
    case SYS_close: goto _close;
    case SYS_lseek: goto _lseek;
    case SYS_sbrk:  goto _sbrk;
    default:        goto _default;
    }
  }

  // For now, skip 2 instructions for RetroBSD syscalls as if always successful.
  return retroBSD ? 2 : 0;
}

void DoBreak(uint32 instr)
{
  uint32 code = (instr >> 16) & 0x3FF; // are there really another/extra 10 bits of the code?

  switch (code)
  {
  case 6:
    error("\nBreak: Signed division overflow at PC = 0x%08lX\n", (ulong)Regs[REG_PC]);
    break;
  case 7:
    error("\nBreak: Division by 0 at PC = 0x%08lX\n", (ulong)Regs[REG_PC]);
    break;
  default:
    error("\nBreak: Code: 0x%05lX at PC = 0x%08lX\n", (ulong)code, (ulong)Regs[REG_PC]);
    break;
  }
}

void DoTrap(uint32 instr)
{
  uint32 code = (instr >> 6) & 0x3FF;

  switch (code)
  {
  case 6:
    error("\nTrap: Signed division overflow at PC = 0x%08lX\n", (ulong)Regs[REG_PC]);
    break;
  case 7:
    error("\nTrap: Division by 0 at PC = 0x%08lX\n", (ulong)Regs[REG_PC]);
    break;
  default:
    error("\nTrap: Code: 0x%03lX at PC = 0x%08lX\n", (ulong)code, (ulong)Regs[REG_PC]);
    break;
  }
}

void DoOverflow(void)
{
  error("Signed integer addition/subtraction overflow at PC = 0x%08lX\n", (ulong)Regs[REG_PC]);
}

void DoInvalidInstruction(uint32 instr)
{
  error("Invalid/unsupported instruction: Opcode: 0x%08lX at PC = 0x%08lX\n", (ulong)instr, (ulong)Regs[REG_PC]);
}
