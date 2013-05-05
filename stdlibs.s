 # /*
 # Copyright (c) 2013, Alexey Frunze
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met: 
 #
 # 1. Redistributions of source code must retain the above copyright notice, this
 #    list of conditions and the following disclaimer. 
 # 2. Redistributions in binary form must reproduce the above copyright notice,
 #    this list of conditions and the following disclaimer in the documentation
 #    and/or other materials provided with the distribution. 
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 # ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 #
 # The views and conclusions contained in the software and documentation are those
 # of the authors and should not be interpreted as representing official policies, 
 # either expressed or implied, of the FreeBSD Project.
 # */
 #
 # /*****************************************************************************/
 # /*                                                                           */
 # /*                                 stdlibc                                   */
 # /*                                                                           */
 # /*   Parts of the standard C library needed to run SmallerC on MipsEmuElf.   */
 # /*                                                                           */
 # /*****************************************************************************/

        .text
 
# void __spim_print_int(int);
.global __spim_print_int
__spim_print_int:
    li      $2, 1 # print int
    syscall
    j       $31

# void __spim_print_string(char*);
.global __spim_print_string
__spim_print_string:
    li      $2, 4 # print string
    syscall
    j       $31

# int __spim_read_int(void);
.global __spim_read_int
__spim_read_int:
    li      $2, 5 # read int
    syscall
    j       $31

# void __spim_read_string(char*, size_t);
.global __spim_read_string
__spim_read_string:
    li      $2, 8 # read string
    syscall
    j       $31

# void* sbrk(ssize_t);
# void* __spim_sbrk(ssize_t);
.global sbrk
.global __spim_sbrk
sbrk:
__spim_sbrk:
    li      $2, 9 # sbrk
    syscall
    j       $31

# void exit(int);
# void __spim_exit(int);
.global exit
.global __spim_exit
exit:
__spim_exit:
    li      $2, 10 # exit
    syscall

# int putchar(int);
# int __spim_print_char(int);
.global putchar
.global __spim_print_char
putchar:
__spim_print_char:
    li      $2, 11 # putchar
    syscall
    move    $2, $4 # SPIM doesn't return the char, return it
    j       $31

# int getchar(void);
# int __spim_read_char(void);
.global getchar
.global __spim_read_char
getchar:
__spim_read_char:
    li      $2, 12 # getchar
    syscall
    j       $31

# int open(char*, int);
# int __spim_open(char*, int);
.global open
.global __spim_open
open:
__spim_open:
    li      $2, 13 # open
    syscall
    j       $31

# int read(int, void*, size_t);
# int __spim_read(int, void*, size_t);
.global read
.global __spim_read
read:
__spim_read:
    li      $2, 14 # read
    syscall
    j       $31

# int write(int, void*, size_t);
# int __spim_write(int, void*, size_t);
.global write
.global __spim_write
write:
__spim_write:
    li      $2, 15 # write
    syscall
    j       $31

# int close(int);
# int __spim_close(int);
.global close
.global __spim_close
close:
__spim_close:
    li      $2, 16 # close
    syscall
    li      $2, 0 # SPIM doesn't return any error code, return 0
    j       $31
