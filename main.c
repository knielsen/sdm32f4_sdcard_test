/*
 * This program turns on the 4 leds of the stm32f4 discovery board
 * one after another.
 * It defines shortcut definitions for the led pins and
 * stores the order of the leds in an array which is being
 * iterated in a loop.
 *
 * This program is free human culture like poetry, mathematics
 * and science. You may use it as such.
 */

#include <math.h>

#include "stm32f4xx.h"

#include <stm324x7i_eval_sdio_sd.h>

#include "my_sdio.h"


#define SRAM_SIZE ((uint32_t)(2*1024*1024))

/* This is apparently needed for libc/libm (eg. powf()). */
int __errno;


static void delay(__IO uint32_t nCount)
{
    while(nCount--)
        __asm("nop"); // do nothing
}


static void setup_serial(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* enable peripheral clock for USART2 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIOA Configuration:  USART2 TX on PA2 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect USART2 pins to AF2 */
  // TX = PA2
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);

  USART_Cmd(USART2, ENABLE); // enable USART2
}


static void
setup_led(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
}


static void
led_on(void)
{
  GPIO_SetBits(GPIOG, GPIO_Pin_15);
}


static void
led_off(void)
{
  GPIO_ResetBits(GPIOG, GPIO_Pin_15);
}


static void
serial_putchar(USART_TypeDef* USARTx, uint32_t c)
{
  while(!(USARTx->SR & USART_FLAG_TC));
  USART_SendData(USARTx, c);
}


static void
serial_puts(USART_TypeDef *usart, const char *s)
{
  while (*s)
    serial_putchar(usart, (uint8_t)*s++);
}


static void
serial_output_hexdig(USART_TypeDef* USARTx, uint32_t dig)
{
  serial_putchar(USARTx, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


__attribute__ ((unused))
static void
serial_output_hexbyte(USART_TypeDef* USARTx, uint8_t byte)
{
  serial_output_hexdig(USARTx, byte >> 4);
  serial_output_hexdig(USARTx, byte & 0xf);
}


__attribute__((unused))
static void
hexdump_block(USART_TypeDef *USARTx, uint8_t *buf, uint32_t len)
{
  uint32_t idx, j;

  idx = 0;
  for (;;)
  {
    if (idx >= len)
      break;

    serial_output_hexbyte(USARTx, (idx >> 24) & 0xff);
    serial_output_hexbyte(USARTx, (idx >> 16) & 0xff);
    serial_output_hexbyte(USARTx, (idx >> 8) & 0xff);
    serial_output_hexbyte(USARTx, idx & 0xff);
    serial_puts(USARTx, " ");
    for (j = 0; j < 16; ++j)
    {
      if (idx + j >= len)
        serial_puts(USARTx, "   ");
      else
      {
        serial_output_hexbyte(USARTx, buf[idx + j]);
        serial_putchar(USARTx, ' ');
      }
    }
    serial_puts(USARTx, "  ");
    for (j = 0; j < 16; ++j)
    {
      if (idx + j >= len)
        serial_putchar(USARTx, ' ');
      else
      {
        uint8_t b = buf[idx + j];
        if (b >= 32 && b <= 127)
          serial_putchar(USARTx, b);
        else
          serial_putchar(USARTx, '.');
      }
    }
    serial_puts(USARTx, "\r\n");
    idx += 16;
  }
}


__attribute__ ((unused))
static void
println_uint32(USART_TypeDef* usart, uint32_t val)
{
  char buf[13];
  char *p = buf;
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);

  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_puts(usart, buf);
}


__attribute__ ((unused))
static void
println_int32(USART_TypeDef* usart, int32_t val)
{
  if (val < 0)
  {
    serial_putchar(usart, '-');
    println_uint32(usart, (uint32_t)0 - (uint32_t)val);
  }
  else
    println_uint32(usart, val);
}


static void
float_to_str(char *buf, float f, uint32_t dig_before, uint32_t dig_after)
{
  float a;
  uint32_t d;
  uint8_t leading_zero;

  if (f == 0.0f)
  {
    buf[0] = '0';
    buf[1] = '\0';
    return;
  }
  if (f < 0)
  {
    *buf++ = '-';
    f = -f;
  }
  a =  powf(10.0f, (float)dig_before);
  if (f >= a)
  {
    buf[0] = '#';
    buf[1] = '\0';
    return;
  }
  leading_zero = 1;
  while (dig_before)
  {
    a /= 10.0f;
    d = (uint32_t)(f / a);
    if (leading_zero && d == 0 && a >= 10.0f)
      *buf++ = ' ';
    else
    {
      leading_zero = 0;
      *buf++ = '0' + d;
      f -= d*a;
    }
    --dig_before;
  }
  if (!dig_after)
  {
    *buf++ = '\0';
    return;
  }
  *buf++ = '.';
  do
  {
    f *= 10.0f;
    d = (uint32_t)f;
    *buf++ = '0' + d;
    f -= (float)d;
    --dig_after;
  } while (dig_after);
  *buf++ = '\0';
}


__attribute__ ((unused))
static void
println_float(USART_TypeDef* usart, float f,
              uint32_t dig_before, uint32_t dig_after)
{
  char buf[21];
  char *p = buf;

  float_to_str(p, f, dig_before, dig_after);
  while (*p)
    ++p;
  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_puts(usart, buf);
}


void SDIO_IRQHandler(void)
{
  SD_ProcessIRQSrc();
}

void SD_SDIO_DMA_IRQHANDLER(void)
{
  SD_ProcessDMAIRQ();
}


static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = SD_SDIO_DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
}


static uint8_t buf[512*4];

int main(void)
{
  SD_Error err;

  delay(2000000);
  setup_serial();
  setup_led();
  serial_puts(USART2, "Initialising...\r\n");
  NVIC_Configuration();
  delay(2000000);

  if (SD_Init() == SD_OK)
    serial_puts(USART2, "SD ok!?!\r\n");
  else
    serial_puts(USART2, "SD not ok :-/\r\n");
  delay(2000000);
buf[0]=1;buf[1]=2;buf[2]=4;buf[3]=100;buf[511]=254;

/*
  if ((err = SD_ReadBlock(buf, 0x00, 512)) == SD_OK)
    serial_puts(USART2, "SD read ok!?!\r\n");
  else
  {
    serial_puts(USART2, "SD read not ok: ");
    serial_puts(USART2, sdio_error_name(err));
    serial_puts(USART2, " :-/\r\n");
  }
  if (SD_WaitReadOperation())
    serial_puts(USART2, "SD wait ok!?!\r\n");
  else
    serial_puts(USART2, "SD wait not ok :-/\r\n");
  while (SD_GetStatus() != SD_TRANSFER_OK)
    ;
*/


  if ((err = SD_ReadMultiBlocks(buf, 0, 512, 4)) == SD_OK)
    serial_puts(USART2, "SD multiread ok!?!\r\n");
  else
  {
    serial_puts(USART2, "SD multiread not ok: ");
    serial_puts(USART2, sdio_error_name(err));
    serial_puts(USART2, " :-/\r\n");
  }
  if ((err = SD_WaitReadOperation()) == SD_OK)
    serial_puts(USART2, "SD wait multi ok!?!\r\n");
  else
    serial_puts(USART2, "SD wait multi not ok: ");
    serial_puts(USART2, sdio_error_name(err));
    serial_puts(USART2, " :-/\r\n");
  while (SD_GetStatus() != SD_TRANSFER_OK)
    ;


//for(;;){if (GPIO_ReadInputDataBit(SD_DETECT_GPIO_PORT, SD_DETECT_PIN)) led_on(); else led_off();}

  hexdump_block(USART2, buf, 16);

  serial_puts(USART2, "Hello world, ready to blink!\r\n");

  while (1)
  {
    led_on();
    delay(2000000);
    led_off();
    delay(2000000);
  }

  return 0;
}
