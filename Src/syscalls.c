/**
 *****************************************************************************
 **
 **  File        : syscalls.c
 **
 **  Abstract    : Atollic TrueSTUDIO Minimal System calls file
 **
 ** 		          For more information about which c-functions
 **                need which of these lowlevel functions
 **                please consult the Newlib libc-manual
 **
 **  Environment : Atollic TrueSTUDIO
 **
 **  Distribution: The file is distributed as is, without any warranty
 **                of any kind.
 **
 **  (c)Copyright Atollic AB.
 **  You may use this file as-is or modify it according to the needs of your
 **  project. This file may only be built (assembled or compiled and linked)
 **  using the Atollic TrueSTUDIO(R) product. The use of this file together
 **  with other tools than Atollic TrueSTUDIO(R) is not permitted.
 **
 *****************************************************************************
 */

/* Includes */
#include <stdint.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>

#include "stm32f4xx_hal.h"

#define PRINT_BUFFER_SIZE     65536

extern UART_HandleTypeDef huart2;

uint8_t print_buffer[PRINT_BUFFER_SIZE];
int in = 0;
int out = 0;
int printing = 0;

static int uart_printable(void);
static void uart_print(void);

void uart_ll_print(void)
{
  if (printing) return;
  if (uart_printable())
  {
    uart_print();
  }
}

void uart_hl_print(void)
{
  if (uart_printable())
  {
    printing = 1;

    if (uart_printable())
    {
      uart_print();
    }

    printing = 0;
  }
}

/**
 *
 */
static int uart_printable(void)
{

  if (in != out)
  {
    if (huart2.State == HAL_UART_STATE_READY
        || huart2.State == HAL_UART_STATE_BUSY_RX)
    {
      return 1;
    }
  }

  return 0;
}

static void uart_print(void)
{
  if (in < out)
  {

    HAL_UART_Transmit_DMA(&huart2, &print_buffer[out], PRINT_BUFFER_SIZE - out);
    out = 0;
  }
  else
  {

    HAL_UART_Transmit_DMA(&huart2, &print_buffer[out], in - out);
    out = in;
  }
}

int uart_write(int32_t file, uint8_t *ptr, int32_t len)
{

  int i;

  for (i = 0; i < len; i++)
  {
    print_buffer[in] = ptr[i];
    in++;
    if (in >= PRINT_BUFFER_SIZE)
    {
      in = in - PRINT_BUFFER_SIZE;
    }

    if (in == out)
    {
      return i;
    };
  }

  uart_hl_print();
  return len;
}

// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart_ll_print();
  }
}

/* Variables */
#undef errno
extern int32_t errno;

register uint8_t * stack_ptr asm("sp");

uint8_t *__env[1] =
{ 0 };
uint8_t **environ = __env;

/* Functions */
void initialise_monitor_handles()
{
}

int _getpid(void)
{
  return 1;
}

int _kill(int32_t pid, int32_t sig)
{
  errno = EINVAL;
  return -1;
}

void _exit(int32_t status)
{
  _kill(status, -1);
  while (1)
  {
  } /* Make sure we hang here */
}

int _write(int32_t file, uint8_t *ptr, int32_t len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  // return len;
  return uart_write(file, ptr, len);
}

caddr_t _sbrk(int32_t incr)
{
  extern uint8_t end asm("end");
  static uint8_t *heap_end;
  uint8_t *prev_heap_end;

  if (heap_end == 0) heap_end = &end;

  prev_heap_end = heap_end;
  if (heap_end + incr > stack_ptr)
  {
//		write(1, "Heap and stack collision\n", 25);
//		abort();
    printf("--------------------------- no memory ------------------ !!! \r\n");

    errno = ENOMEM;
    return (caddr_t) -1;
  }

  heap_end += incr;

  return (caddr_t) prev_heap_end;
}

int _close(int32_t file)
{
  return -1;
}

int _fstat(int32_t file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int32_t file)
{
  return 1;
}

int _lseek(int32_t file, int32_t ptr, int32_t dir)
{
  return 0;
}

int _read(int32_t file, uint8_t *ptr, int32_t len)
{
  return 0;
}

int _open(const uint8_t *path, int32_t flags, int32_t mode)
{
  /* Pretend like we always fail */
  return -1;
}

int _wait(int32_t *status)
{
  errno = ECHILD;
  return -1;
}

int _unlink(const uint8_t *name)
{
  errno = ENOENT;
  return -1;
}

int _times(struct tms *buf)
{
  return -1;
}

int _stat(const uint8_t *file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _link(const uint8_t *old, const uint8_t *new)
{
  errno = EMLINK;
  return -1;
}

int _fork(void)
{
  errno = EAGAIN;
  return -1;
}

int _execve(const uint8_t *name, uint8_t * const *argv, uint8_t * const *env)
{
  errno = ENOMEM;
  return -1;
}

