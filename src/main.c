/**
  ******************************************************************************
  * @file    main.c
  * @date    Feb 7, 2024
  * @brief   ECE 362 Lab 7 student template
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include <stdint.h>

/////
#include <stdio.h>
#include "fifo.h"
#include "tty.h"

void init_usart5() {
    // TODO
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;
    GPIOC->MODER &= ~GPIO_MODER_MODER12;
    GPIOC->MODER |= GPIO_MODER_MODER12_1;
    GPIOD->MODER &= ~GPIO_MODER_MODER2;
    GPIOD->MODER |= GPIO_MODER_MODER2_1;

    GPIOC->AFR[1] &= ~(0xf << (4*(12-8))); 
    GPIOC->AFR[1] |= (0x2 << (4*(12-8)));
    GPIOD->AFR[0] &= ~(0xf << (4*(2))); 
    GPIOD->AFR[0] |= (0x2 << (4*(2))); 

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;

    USART5->CR1 &= ~USART_CR1_UE;
    USART5->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);
    USART5->CR2 &= ~USART_CR2_STOP;
    USART5->CR1 &= ~USART_CR1_PCE;
    USART5->CR1 &= ~USART_CR1_OVER8;
    USART5->BRR = 0x1A1;
    USART5->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    USART5->CR1 |= USART_CR1_UE;

    while(!(USART5->ISR & USART_ISR_REACK) && !(USART5->ISR & USART_ISR_TEACK));
    
}



// TODO DMA data structures
#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

void enable_tty_interrupt(void) {
    // TODO
    USART5->CR1 |= USART_CR1_RXNEIE;
    USART5->CR3 |= USART_CR3_DMAR; 
    NVIC->ISER[0] |= (1<< USART3_8_IRQn);
    

    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
    DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;

    DMA2_Channel2->CMAR = (uint32_t)serfifo;
    DMA2_Channel2->CPAR = (uint32_t)&(USART5->RDR);
    DMA2_Channel2->CNDTR = FIFOSIZE;
    DMA2_Channel2->CCR &= ~DMA_CCR_DIR;
    DMA2_Channel2->CCR &= ~DMA_CCR_TCIE;
    DMA2_Channel2->CCR &= ~DMA_CCR_HTIE;
    DMA2_Channel2->CCR &= ~DMA_CCR_MSIZE;
    DMA2_Channel2->CCR &= ~DMA_CCR_PSIZE;
    DMA2_Channel2->CCR |= DMA_CCR_MINC;
    DMA2_Channel2->CCR &= ~DMA_CCR_PINC;
    DMA2_Channel2->CCR |= DMA_CCR_CIRC;
    DMA2_Channel2->CCR &= ~DMA_CCR_MEM2MEM;
    DMA2_Channel2->CCR |= (DMA_CCR_PL_0 | DMA_CCR_PL_1);

    DMA2_Channel2->CCR |= DMA_CCR_EN;
}

// Works like line_buffer_getchar(), but does not check or clear ORE nor wait on new characters in USART
char interrupt_getchar() {
    // TODO
    
    USART_TypeDef *u = USART5;

    while(fifo_newline(&input_fifo) == 0) {
        asm volatile ("wfi"); // wait for an interrupt
    }
    char ch = fifo_remove(&input_fifo);
    return ch;

}

int __io_putchar(int c) {
    // TODO copy from STEP2
        if (c == '\n'){
        while(!(USART5->ISR & USART_ISR_TXE));
        USART5->TDR = '\r';
    }
    
    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    // TODO Use interrupt_getchar() instead of line_buffer_getchar()
    return (interrupt_getchar());
}

// TODO Copy the content for the USART5 ISR here
// TODO Remember to look up for the proper name of the ISR function
void USART3_8_IRQHandler(void) {
    while(DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
        if (!fifo_full(&input_fifo))
            insert_echo_char(serfifo[seroffset]);
        seroffset = (seroffset + 1) % sizeof serfifo;
    }
}


////

void internal_clock();

#include "lcd.h"
#include "commands.h"
#include "header.h"
#include <stdio.h>
#include "tty.h"
int i2c_checknack(void);
void i2c_clearnack(void);
// DMA data structures
//int seroffset = 0;
void enable_ports();
void show_char(int n, char c);
void drive_column(int c);
int read_rows();
char rows_to_key(int rows);
//uint8_t col = 0;
char c;
// Make it easier to access keymap
// extern char keymap;
// char* keymap_arr = &keymap;
void nano_wait(int t);
// Font array in assembly file as I am too lazy to convert it into C array
// extern uint8_t font[];

// created for game logic
int currCol = 2; // track cursor columm
int prevCol = 2; // cursor column
int box_num; // column that tile is dropping in 
int boxTwo;
int left; // left pixel of box
int leftTwo;
int tileNum; 
int lane1 = 0;
int lane2 = 0;
int lane3 = 0;
int lane4 = 0;
int16_t currScore = 0; // current player's score
int highscore = 0; // game's highscore ( will be updated to and with eeprom)


void enable_ports() {
  // lab 6 for spi - show current score
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  GPIOB->MODER &= ~(0xCF000000);
  GPIOB->MODER |= 0x45000000;
  GPIOB->BSRR = GPIO_BSRR_BS_12;
  GPIOB->BSRR = GPIO_BSRR_BR_13;

  // eeprom
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable port B
  GPIOB->MODER &= ~(0x0000F000); //clear PB6-7
  GPIOB->MODER |= 0x0000A000; //set to alt function mode for PB6-7
  GPIOB->AFR[0] |= 0x1 << (6 * 4); //set to AF1
  GPIOB->AFR[0] |= 0x1 << (7 * 4); //set to AF1

    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Set PC4-PC7 (Columns) as outputs
    GPIOC->MODER &= ~0x0000FF00; // Clear bits for PC4-PC7
    GPIOC->MODER |= 0x00005500;  // Set bits to 01 (output mode)

    // Set PC0-PC3 (Rows) as inputs
    GPIOC->MODER &= ~0x000000FF; // Clear bits for PC0-PC3

    // Enable pull-down resistors on PC0-PC3
    GPIOC->PUPDR &= ~0x000000FF; // Clear bits for PC0-PC3
    GPIOC->PUPDR |= 0x000000AA;  // Set bits to 10 (pull-down)

}

// lab 6 stuff:
int msg_index = 0;
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];

// tim15 used for spi for 7 seg displays
void init_tim15(void) {
    // enable tim15 rcc clock 
    RCC->APB2ENR = RCC_APB2ENR_TIM15EN;
    // dma request at 1 kHz by setting ude in dieR 
    TIM15->PSC = 47;
    TIM15->ARR = 999;
    TIM15->DIER |= TIM_DIER_UDE; 
    TIM15->CR1 |= TIM_CR1_CEN;
}

// initialize spi2
void init_spi2(void) { 
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    // PB12 (CS-NSS), PB13 (SCK), PB15 (SDI-MOSI) - 10 
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1;
    GPIOB->MODER &= ~(GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER15_0);
    GPIOB->AFR[1] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7);
    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;
    SPI2->CR2 |= SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
    SPI2->CR1 |= SPI_CR1_MSTR;
    SPI2->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_TXDMAEN;
    SPI2->CR1 |= SPI_CR1_SPE;
}
// Configure the SPI2 peripheral to trigger the DMA channel when the
// transmitter is empty.  Use the code from setup_dma from lab 5.
void spi2_setup_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMAEN;
    // turn off enable bit 
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    // set CMAR to msg address
    DMA1_Channel5->CMAR = (uint32_t) &(msg);
    // set cpar to address of gpiob_odr 
    DMA1_Channel5->CPAR = (uint32_t) &(SPI2->DR); // changed
    // set cndtr to 8 
    DMA1_Channel5->CNDTR = 8;
    // set DIR, MINC, M size 16, P size 16, channel for circ 
    DMA1_Channel5->CCR |= DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC;
    SPI2->CR2 |= SPI_CR2_TXDMAEN;
}
// Enable the DMA channel.
void spi2_enable_dma(void) {
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}

// TFT EXTRA
void init_spi1_slow(void) { 
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    // PB3 SCK, PB4 MISO, PB5 MOSI 

    GPIOB->MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
    GPIOB->MODER &= ~(GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0);
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5);
    
    SPI1->CR1 &= ~SPI_CR1_SPE;
    // baud rate max, master, ssm, ssi
    SPI1->CR1 |=  SPI_CR1_BR | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    // data size 8. fifo reception 
    
    SPI1->CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; 
    SPI1->CR2 &= ~(SPI_CR2_DS_3);
    SPI1->CR2 |= SPI_CR2_FRXTH;
    SPI1->CR1 |= SPI_CR1_SPE;
}
void enable_sdcard(void) { 
    GPIOB->BSRR |= GPIO_BSRR_BR_2;
}
void disable_sdcard(void) { 
    GPIOB->BSRR |= GPIO_BSRR_BS_2;
}
void init_sdcard_io (void) {
    init_spi1_slow();
    GPIOB->MODER |= 0x10; // PB2 01
    disable_sdcard();
}
void sdcard_io_high_speed() { // 2.5
    SPI1->CR1 &= ~SPI_CR1_SPE; //disable SPI1 channel
    SPI1->CR1 &= ~(SPI_CR1_BR);
    SPI1->CR1 |= SPI_CR1_BR_0;//Baudrate cloack to 12MHz (if not working try lowering it)
    SPI1->CR1 |= SPI_CR1_SPE;// enable SPI channel
}
void init_lcd_spi() { //2.7
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;//Clock GPIOB
    GPIOB->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER14_0;//onfigure  PB8, PB11, and PB14 as outputs
    GPIOB->MODER &= ~(GPIO_MODER_MODER8_1 | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER14_1);
    init_spi1_slow();// cll init_spi_slow to configure SPI1
    sdcard_io_high_speed();// sdcard-io_high_speed() to make SPI1 fast
}

void print(const char str[])
{
    const char *p = str;
    for(int i=0; i<8; i++) {
        if (*p == '\0') {
            msg[i] = (i<<8);
        } else {
            msg[i] = (i<<8) | font[*p & 0x7f] | (*p & 0x80);
            p++;
        }
    }
} 
void setup_adc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= 0x3;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    ADC1->CHSELR = 0;
    ADC1->CHSELR |= 1 << 0;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
}

#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;
int joyVal = 0;

void TIM2_IRQHandler(void) {
    TIM2->SR &= ~TIM_SR_UIF;
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_EOC));
  
    joyVal = ADC1->DR;
}

void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 48000-1;
    TIM2->ARR = 100-1;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] = 1 << TIM2_IRQn;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_SetPriority(TIM2_IRQn, 1);
}

// eeprom i2c stuff
void init_i2c(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; //enable peripheral clock
    I2C1->CR1 &= ~I2C_CR1_PE; //peripheral disabled

    I2C1->CR1 |= I2C_CR1_NOSTRETCH; //disable clock stretching

    I2C1->TIMINGR &= ~(0xF << 28); //clear all registers
    I2C1->TIMINGR |= 0x5 << 28; //set prescalar to 5
    I2C1->TIMINGR |= 0x3 << 20; //set scldel
    I2C1->TIMINGR |= 0x3 << 16; //set sdadel
    I2C1->TIMINGR |= 0x3 << 8; //set sclh
    I2C1->TIMINGR |= 0x9; //set scll
    
    I2C1->CR2 &= ~I2C_CR2_ADD10; //7-bit addressing mode
    I2C1->CR1 |= I2C_CR1_PE; //peripheral enabled
}

//===========================================================================
// Send a START bit.
//===========================================================================
void i2c_start(uint32_t targadr, uint8_t size, uint8_t dir) {
    // 0. Take current contents of CR2 register. 
    uint32_t tmpreg = I2C1->CR2;

    // 1. Clear the following bits in the tmpreg: SADD, NBYTES, RD_WRN, START, STOP
    tmpreg &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);

    // 2. Set read/write direction in tmpreg.
    if(dir == 0) {
        tmpreg &= ~(1 << 10);
    }
    else {
        tmpreg |= dir << 10; //shifts dir into RD_WRN
    }

    // 3. Set the target's address in SADD (shift targadr left by 1 bit) and the data size.
    tmpreg |= ((targadr << 1 ) & I2C_CR2_SADD) | ((size << 16) & I2C_CR2_NBYTES);

    // 4. Set the START bit.
    tmpreg |= I2C_CR2_START;

    // 5. Start the conversion by writing the modified value back to the CR2 register.
    I2C1->CR2 = tmpreg;
}

//===========================================================================
// Send a STOP bit.
//===========================================================================
void i2c_stop(void) {
    // 0. If a STOP bit has already been sent, return from the function.
    if (I2C1->ISR & I2C_ISR_STOPF) {
        return;
    }

    // 1. Set the STOP bit in the CR2 register.
    I2C1->CR2 |= I2C_CR2_STOP;

    // 2. Wait until STOPF flag is reset by checking the same flag in ISR.
    while(!(I2C1->ISR & I2C_ISR_STOPF));

    // 3. Clear the STOPF flag by writing 0 to the corresponding bit in the ICR.
    I2C1->ICR |= I2C_ICR_STOPCF;
}

//===========================================================================
// Wait until the I2C bus is not busy. (One-liner!)
//===========================================================================
void i2c_waitidle(void) {
    while ((I2C1->ISR & I2C_ISR_BUSY));
}

//===========================================================================
// Send each char in data[size] to the I2C bus at targadr.
//===========================================================================
int8_t i2c_senddata(uint8_t targadr, uint8_t data[], uint8_t size, uint8_t randRead) {
    i2c_start(targadr, size, 0); //request a write transfer

    for(int i = 0; i < size; i++) {
        // Wait until the TXIS flag is set in the ISR, and quit if it takes too long:
        int count = 0;
        while ((I2C1->ISR & I2C_ISR_TXIS) == 0) {
            count += 1;
            if (count > 1000000)
                return -1;
            if (i2c_checknack()) {
                i2c_clearnack();
                i2c_stop();
                return -1;
            }
        }

        I2C1->TXDR = data[i] & I2C_TXDR_TXDATA;
    }

     while ((I2C1->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)) == 0);

    if (I2C1->ISR & I2C_ISR_NACKF) {
        i2c_clearnack();
        i2c_stop();
        return -1;
    }

    i2c_stop();
    return 0;
}

//===========================================================================
// Receive size chars from the I2C bus at targadr and store in data[size].
//===========================================================================
int i2c_recvdata(uint8_t targadr, void *data, uint8_t size) {
    i2c_start(targadr, size, 1); //send a read request

    for(int i = 0; i < size; i++) {
        int count = 0;
        while ((I2C1->ISR & I2C_ISR_RXNE) == 0) {
            count += 1;
            if (count > 1000000)
                return -1;
            if (i2c_checknack()) {
                i2c_clearnack();
                i2c_stop();
                return -1;
            }
        }

        ((uint8_t *)data)[i] = (I2C1->RXDR & I2C_RXDR_RXDATA); // Mask and store
    }

    // i2c_waitidle();
    i2c_stop();
    return 0;
}

//===========================================================================
// Clear the NACK bit. (One-liner!)
//===========================================================================
void i2c_clearnack(void) {
    I2C1->ICR |= I2C_ICR_NACKCF; // Clear the NACK flag
}

//===========================================================================
// Check the NACK bit. (One-liner!)
//===========================================================================
int i2c_checknack(void) {
    return (I2C1->ISR & I2C_ISR_NACKF) == 1;
}

//===========================================================================
// EEPROM functions
// We'll give these so you don't have to figure out how to write to the EEPROM.
// These can differ by device.

#define EEPROM_ADDR 0x57

void eeprom_write(uint16_t loc, const char* data, uint8_t len) {
    uint8_t bytes[34];
    bytes[0] = loc>>8;
    bytes[1] = loc&0xFF;
    for(int i = 0; i<len; i++){
        bytes[i+2] = data[i];
    }
    i2c_senddata(EEPROM_ADDR, bytes, len+2, 0);
}

void eeprom_read(uint16_t loc, char data[], uint8_t len) {
    // ... your code here
    uint8_t bytes[2];
    bytes[0] = loc>>8;
    bytes[1] = loc&0xFF;
    i2c_senddata(EEPROM_ADDR, bytes, 2, 1);
    // i2c_waitidle();
    i2c_recvdata(EEPROM_ADDR, data, len);
}


/////

//============================================================================
// PARAMETERS
//============================================================================
#include "ff.h"  // FatFs header for file system
#include "diskio.h"  // Disk I/O driver for FatFs

#define MODE_SD_CARD 0
#define MODE_SIMPLE_TONE 1

volatile uint8_t audio_mode = MODE_SIMPLE_TONE;  // Default mode: simple tone


#define RATE 20000

#define AUDIO_BUFFER_SIZE 8000  // 8000 bytes for a brief audio buffer
volatile int16_t audio_buffer[AUDIO_BUFFER_SIZE / 2];  // For 16-bit samples
volatile uint8_t buffer_needs_refill = 0;



// Define the WAV header structure
typedef struct {
    char riff_id[4];        // "RIFF"
    uint32_t riff_size;
    char wave_id[4];        // "WAVE"
    char fmt_id[4];         // "fmt "
    uint32_t fmt_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char data_id[4];        // "data"
    uint32_t data_size;
} WAVHeader;

#include <math.h>
#define N 1000
#define RATE 24000
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;

uint16_t volume_scale = 100;  // 150% volume (adjust as needed)

uint16_t bits_per_sample;
uint16_t num_channels;

FATFS fs;
FIL audio_file;

//============================================================================
// test audio 
//============================================================================



void init_wavetable(void) {
    for(int i=0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}


void set_freq(uint32_t f) {
    
    if (f == 0) {
        step0 = 0;
        offset0 = 0;
    } else
        step0 = (f * N / RATE) * (1<<16);

}


//============================================================================
// setup_dac()
//============================================================================


void setup_dac(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // Enable GPIOA clock
    GPIOA->MODER |= 0x00000300;         // Set PA4 to analog mode
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;  // Enable DAC clock
    DAC->CR &= ~DAC_CR_TSEL1;           // Clear TSEL1 bits
    DAC->CR |= DAC_CR_TEN1;             // Enable DAC trigger
    DAC->CR |= DAC_CR_EN1;              // Enable DAC channel 1
    DAC->CR |= DAC_CR_DMAEN1;           // Enable DAC DMA request
}



void TIM6_DAC_IRQHandler(void) {
    TIM6->SR &= ~TIM_SR_UIF;  // Clear interrupt flag

    if (audio_mode == MODE_SIMPLE_TONE) {
        offset0 += step0;
        offset1 += step1;

        if (offset0 >= (N << 16)) {
            offset0 -= (N << 16);
        }
        if (offset1 >= (N << 16)) {
            offset1 -= (N << 16);
        }

        int samp = wavetable[offset0 >> 16] + wavetable[offset1 >> 16];
        samp = samp * (2048);
        samp = (samp >> 17);
        samp += 2048;
        DAC->DHR12R1 = samp;
    }
}


void init_tim6(uint32_t sample_rate) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;  // Enable Timer 6 clock

    if (audio_mode == MODE_SIMPLE_TONE) {
        TIM6->PSC = (48000000 / (RATE * 100)) - 1;
        TIM6->ARR = 100 - 1;
    } else if (audio_mode == MODE_SD_CARD) {
        TIM6->PSC = (48000000 / (sample_rate * 100)) - 1;  // Use the provided sample rate
        TIM6->ARR = 100 - 1;
    }

    TIM6->DIER |= TIM_DIER_UIE;         // Enable update interrupt
    NVIC->ISER[0] = 1 << TIM6_IRQn;     // Enable Timer 6 interrupt in NVIC

    TIM6->CR2 |= TIM_CR2_MMS_1;         // Set TRGO on update event
    TIM6->CR1 |= TIM_CR1_CEN;           // Start Timer 6
}





// Function to set up the DAC and Timer 6 for generating a simple tone
void generate_simple_tone(uint32_t frequency) {

    audio_mode = MODE_SIMPLE_TONE;

    // Generate the waveform lookup table
    init_wavetable();
   
    // Set up the DAC
    setup_dac();

    // Initialize Timer 6 with the calculated sample rate
    init_tim6(0);
    
    // Calculate the sample rate needed for the desired frequency
    set_freq(frequency);


}

void stop_sound(){
    TIM6->CR1 &= ~TIM_CR1_CEN;      // Start Timer 6 

}

void delay_ms(uint32_t ms) {
    uint32_t i;
    while (ms-- > 0) {
        // Each loop iteration is roughly 1 ms when using a 48 MHz system clock.
        for (i = 0; i < 4800; i++) {  // Adjust this loop count for your clock speed
            __asm("nop");             // No-operation instruction to burn time
        }
    }
}



//============================================================================
// init_audio_playback()
//============================================================================

void setup_dma(void) {
    // Ensure DMA1 Channel 3 is disabled before configuring
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;

    // Enable the DMA1 clock
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Set the memory and peripheral addresses
    DMA1_Channel3->CMAR = (uint32_t)audio_buffer;       // Source: Audio buffer in memory
    DMA1_Channel3->CPAR = (uint32_t)&DAC->DHR12L1;      // Destination: DAC data register (left-aligned for 16-bit)

    // Configure the number of data items to transfer
    DMA1_Channel3->CNDTR = AUDIO_BUFFER_SIZE / 2; // Number of 16-bit samples

    // Configure DMA channel settings
    DMA1_Channel3->CCR |= DMA_CCR_DIR        // Memory to peripheral
                        | DMA_CCR_CIRC       // Circular mode
                        | DMA_CCR_MINC       // Memory increment mode
                        | DMA_CCR_MSIZE_0    // 16-bit memory size
                        | DMA_CCR_PSIZE_0    // 16-bit peripheral size
                        | DMA_CCR_HTIE       // Half-transfer interrupt enable
                        | DMA_CCR_TCIE;      // Transfer-complete interrupt enable

    // Enable the DMA interrupt in the NVIC for Channel 2 and Channel 3
    NVIC->ISER[0] = 1 << DMA1_Channel2_3_IRQn;
}



void enable_dma(void) {
    // Enable DMA Channel 3 for DAC
    DMA1_Channel3->CCR |= DMA_CCR_EN;
}


void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_HTIF3) {  // Half-transfer interrupt flag for Channel 3
        DMA1->IFCR |= DMA_IFCR_CHTIF3;  // Clear the half-transfer flag
        buffer_needs_refill = 1;        // Signal to refill the first half of the buffer
       // printf("Half-transfer interrupt: Refilling first half of buffer.\n");
    }
    
    if (DMA1->ISR & DMA_ISR_TCIF3) {  // Transfer-complete interrupt flag for Channel 3
        DMA1->IFCR |= DMA_IFCR_CTCIF3;  // Clear the transfer-complete flag
        buffer_needs_refill = 2;        // Signal to refill the second half of the buffer
        //printf("Transfer-complete interrupt: Refilling second half of buffer.\n");
    }
}


void init_audio_playback(void) {
    // Initialize SPI and SD card I/O
    init_sdcard_io();
    sdcard_io_high_speed();

    // Mount the file system using FatFs
    if (f_mount(&fs, "", 1) != FR_OK) {
        printf("Error: Unable to mount file system.\n");
        return;
    }
    printf("SD card mounted successfully.\n");
    
}

void refill_audio_buffer(void) {
    if (buffer_needs_refill) {
        UINT bytesToRead = AUDIO_BUFFER_SIZE / 2;  // Half buffer size in bytes
        UINT bytesRead;

        // Determine which half of the buffer to refill
        int16_t* buffer_ptr = (buffer_needs_refill == 1) ? audio_buffer
                                 : &audio_buffer[AUDIO_BUFFER_SIZE / 4];

        // Read audio data from the file
        if (f_read(&audio_file, buffer_ptr, bytesToRead, &bytesRead) != FR_OK || bytesRead == 0) {
            printf("End of file or read error.\n");
            TIM6->CR1 &= ~TIM_CR1_CEN;  // Stop playback if no more data
            return;
        }

        // Number of samples read
        uint32_t num_samples = bytesRead / 2;

        // Process each sample
        for (uint32_t i = 0; i < num_samples; i++) {
            int16_t sample = buffer_ptr[i];  // Original signed 16-bit sample

            // Apply volume scaling (increase volume)
            int32_t adjusted_sample = ((int32_t)sample * volume_scale) / 100;

            // Shift to positive range
            adjusted_sample += 32768;  // Shift from [-32768, +32767] to [0, 65535]

            // Clamp to prevent overflow
            if (adjusted_sample < 0)
                adjusted_sample = 0;
            else if (adjusted_sample > 65535)
                adjusted_sample = 65535;

            // Map to DAC range 1024 to 3072
            uint16_t dac_value = (adjusted_sample * 2048) / 65535 + 1024;

            buffer_ptr[i] = dac_value;  // Store adjusted sample back into buffer
        }

        buffer_needs_refill = 0;  // Reset the refill flag
    }
}




uint32_t read_wav_header(const char* filename, uint32_t* sample_rate) {
    WAVHeader header;

    if (f_open(&audio_file, filename, FA_READ) != FR_OK) {
        printf("Error: Unable to open audio file.\n");
        return 0;
    }
    printf("Audio file opened successfully: %s\n", filename);

    UINT bytesRead;
    if (f_read(&audio_file, &header, sizeof(WAVHeader), &bytesRead) != FR_OK || bytesRead != sizeof(WAVHeader)) {
        printf("Error reading WAV header\n");
        f_close(&audio_file);
        return 0;
    }
    printf("WAV header read successfully.\n");

    if (strncmp(header.riff_id, "RIFF", 4) != 0 || strncmp(header.wave_id, "WAVE", 4) != 0) {
        printf("Invalid WAV file format.\n");
        f_close(&audio_file);
        return 0;
    }

    if (header.audio_format != 1 || header.num_channels != 1 || header.bits_per_sample != 16) {
        printf("Unsupported WAV format: channels=%d, bits per sample=%d, format=%d\n",
               header.num_channels, header.bits_per_sample, header.audio_format);
        f_close(&audio_file);
        return 0;
    }

    *sample_rate = header.sample_rate;  // Set the sample rate
    return 1;
}


void play_audio(const char* filename) {
    uint32_t sample_rate;

    // Read the WAV header and get the sample rate
    if (!read_wav_header(filename, &sample_rate)) {
        return;  // Exit if there was an error
    }

    setup_dac();
    setup_dma();
    enable_dma();

    // Pass the sample rate to init_tim6
    init_tim6(sample_rate);

    TIM6->CR1 |= TIM_CR1_CEN;  // Start Timer 6 for playback
}


//============================================================================
// Keypad Scan
//============================================================================

char disp[9] = "Hello...";
//extern uint8_t font[];
volatile uint32_t interrupt_counter = 0;
volatile uint32_t last_debounce_time = 0;
char last_key = '\0'; // Initialize to null character
int no_key_detected_count = 0; // Add this global variable
#define NO_KEY_DETECTED_THRESHOLD 4 // Adjust this threshold as needed
volatile uint32_t song_num = 0;



uint8_t col = 0;
char keymap[16] = {
    'D', 'C', 'B', 'A',
    '#', '9', '6', '3',
    '0', '8', '5', '2',
    '*', '7', '4', '1'
};
char* keymap_arr = keymap;


volatile uint32_t sys_tick_counter = 0;

void SysTick_Handler(void) {
    sys_tick_counter++;
}

void init_systick(void) {
    // Assuming a system clock of 48 MHz
    SysTick_Config(48000); // 1 ms tick
}

uint32_t get_system_time_ms(void) {
    return sys_tick_counter;
}

void drive_column(int c) {
    c = c & 3;
    GPIOC->BRR = 0xF << 4;              // Clear all columns
    GPIOC->BSRR = (1 << (c + 4));       // Set the specific column high
}

int read_rows() {
    return GPIOC->IDR & 0xF;            // Read the state of rows PC0-PC3
}

char rows_to_key(int rows) {
    int offset = -1; // Initialize to an invalid index

    if (rows & 0x1) {
        offset = col * 4;
    } else if (rows & 0x2) {
        offset = col * 4 + 1;
    } else if (rows & 0x4) {
        offset = col * 4 + 2;
    } else if (rows & 0x8) {
        offset = col * 4 + 3;
    }

    if (offset != -1) {
        return keymap_arr[offset];
    } else {
        return '\0'; // No key pressed
    }
}

void handle_key(char key) {
    printf("Key pressed: %c\n", key);
    
    if (key != song_num){
        if (key == '1'){
            play_audio("audio1.wav");
        }
        else if (key == '2'){
            play_audio("audio2.wav");

        }
        else if (key == '3'){
            play_audio("audio3.wav");
        }
    }

    song_num = key;
}

void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;

    // Drive the current column
    drive_column(col);

    // Read rows and handle key press
    int rows = read_rows();
    if (rows != 0) {
        char key = rows_to_key(rows);
        if (key != '\0') {
            if (key != last_key) {
                handle_key(key);
                last_key = key; // Update the last key
            }
            no_key_detected_count = 0; // Reset the counter since a key is detected
        }
    } else {
        // No key is pressed in this scan
        no_key_detected_count++;
        if (no_key_detected_count >= NO_KEY_DETECTED_THRESHOLD) {
            // Only reset last_key after several consecutive no-key detections
            last_key = '\0';
            no_key_detected_count = 0; // Reset the counter
        }
    }

    // Move to the next column
    col = (col + 1) % 4; // Cycle through columns 0-3
}



void setup_tim7() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 480 - 1;  
    TIM7->ARR = 100 - 1;      
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM7_IRQn, 1);
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 |= TIM_CR1_CEN;
}




#include <string.h>
#include <stdio.h>
#include "commands.h"



////




int main() {
    internal_clock();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    enable_ports();
    init_spi2();
    spi2_setup_dma();
    spi2_enable_dma();
    // init_tim15();
    lcd_initME();
    nano_wait(5000);
    clearME(0xffff); // clear screen to start
    // make 4 columns (3 lines)
    drawlineME(60,0,60,320, 0xadd8e6);
    drawlineME(120,0,120,320, 0xadd8e6);
    drawlineME(180,0,180,320, 0xadd8e6);
    drawfillrectME(121,0,179,39,0x0000); 
    srand(5);

    // joystick:
    setup_adc();
    init_tim2();

    // eeprom i2c:
    init_i2c();
    
    char string[32];

    // // read last high score from eeprom
    char lasthigh[32];
    eeprom_read(0, lasthigh, 32);
    // // use sscanf to convert char * lasthigh to int highscore
    sscanf(lasthigh, "%d", &highscore);
    snprintf(string, sizeof(string), "%4d%4d", highscore, 0);
    print(string);
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    // DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    
    //music shit
    init_usart5();
    enable_tty_interrupt();
    init_systick();
    setup_tim7();
    __enable_irq();
    audio_mode = MODE_SD_CARD;
    init_audio_playback();

    while (1) {
        //printf("Interrupt count: %lu\n", interrupt_counter);
        refill_audio_buffer();  // Refills buffer when necessary
        __WFI();                // Wait for interrupt
     }

    int game = 1;

    int time = 500000;
    int timeIncrease = 0;

    while (game) {
      time = time / 100;
      tileNum = (rand() % 2) + 1;
      if (tileNum == 2) {
        box_num = (rand() % 4)+1;
        boxTwo = (rand() % 4) + 1;
        lane1 = 0;
        lane2 = 0;
        lane3 = 0;
        lane4 = 0;
        while (boxTwo == box_num) {
          boxTwo = (rand() % 4) + 1;
        }
        left = box_num * 60 - 1; // left pixel of box at box_num * 60 - 1
        leftTwo = boxTwo * 60 - 1;
        int box1On = 1; //box 1 still exists
        int box2On = 1; //box 2 still exists
        int goLeft=0;
        int goRight=0;
        // loop to shift tiles down
        for (int l = 320; l >= 0; l--) { 
          if (joyVal < 2900) goLeft = 1;
          if (joyVal > 3700) goRight = 1;
          // cursor check
          prevCol = currCol;
          if (!(l%10) && goLeft) { // if left
            currCol--;
            if (currCol <= 0) { currCol = 1; }
            goLeft=0;
          }
          else if (!(l%10) && goRight) { // else if right
            currCol++;
            if (currCol >= 4) { currCol = 4; }
            goRight=0;
          }
          if (prevCol != currCol) {
            if (prevCol == 4) drawfillrectME(0, 0, 59, 39, 0xffff);
            else drawfillrectME(60*(4-prevCol) + 1, 0, 60*(4-prevCol) - 1 + 60, 39, 0xffff);
            if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
            else drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
          }

          // move tiles down 1 pixel
          if (box1On) drawfillrectME(left-58,l-50,left,l,0x0000);
          if (box2On) drawfillrectME(leftTwo-58,l-50,leftTwo,l,0x0000);
          if (!box1On ^ !box2On) nano_wait(time * 3);
          else nano_wait(time);
          if (box1On) drawlineME(left-58,l,left,l,0xffff);
          if (box2On) drawlineME(leftTwo-58,l,leftTwo,l,0xffff);          
          
          if (l <= 0) { // game ends 
            if ((lane1 == 1) || (lane2 == 2) || (lane3 == 3) || (lane4 == 4)){
              game = 0;
              // highscore check
              if (currScore > highscore) highscore = currScore;
              break;
            }
          }

          // check what lane the box is in
          if (l < 79) { // 79 is top of cursor
            if (box1On) {if (box_num == 4) lane1 = 1;
            else if (box_num ==3) lane2 = 2;
            else if (box_num == 2) lane3 = 3;
            else if (box_num == 1) lane4 = 4;
            }

            if (box2On) {if (boxTwo== 4) lane1 = 1;
            else if (boxTwo ==3) lane2 = 2;
            else if (boxTwo == 2) lane3 = 3;
            else if (boxTwo == 1) lane4 = 4;
            }
          }

          if (lane1 == currCol){
            if (box_num == 4) {
              drawfillrectME(left-58,40,left,320,0xffff);
              box1On = 0;
            }
            else {
              drawfillrectME(leftTwo-58,40,leftTwo,320,0xffff);
              box2On = 0;
            } 
            currScore++;
            lane1 = 0;
            drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
          }
          else if (lane2 == currCol) {
            if (box_num == 3) {
              drawfillrectME(left-58,40,left,320,0xffff);
              box1On = 0;
            }
            else {
              drawfillrectME(leftTwo-58,40,leftTwo,320,0xffff);
              box2On = 0;
            }
            currScore++;
            lane2 = 0;
            drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
          }
          else if (lane3 == currCol) {
            if (box_num == 2) {
              drawfillrectME(left-58,40,left,320,0xffff);
              box1On = 0;
            }
            else {
              drawfillrectME(leftTwo-58,40,leftTwo,320,0xffff);
              box2On = 0;
            }
            currScore++;
            lane3 = 0;
            drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
          }
          else if (lane4 == currCol) {
            if (box_num == 1) {
              drawfillrectME(left-58,40,left,320,0xffff);
              box1On = 0;
            }
            else {
              drawfillrectME(leftTwo-58,40,leftTwo,320,0xffff);
              box2On = 0;
            }
            currScore++;
            lane4 = 0;
            drawfillrectME(0, 0,59, 39, 0x0000);
          }
          if (!box1On && !box2On) {
            break;
          }
          if (currScore > highscore) highscore = currScore;
          snprintf(string, sizeof(string), "%4d%4d", highscore, currScore);
          print(string);
        }
      }
      else if (tileNum == 1) {
        box_num = (rand() % 4)+1;
        left = box_num * 60 - 1; // left pixel of box at box_num * 60 - 1
        lane1 = 0;
        lane2 = 0;
        lane3 = 0;
        lane4 = 0;
        // loop to shift tiles down
        int goLeft=0;
        int goRight=0;
        for (int l = 320; l >= 0; l--) { 
          // cursor move check
          if (joyVal < 2900) goLeft = 1;
          if (joyVal > 3700) goRight = 1;

          prevCol = currCol;
          if (!(l%10) && goLeft) { // if left
            currCol--;
            if (currCol < 1) { currCol = 1; }
            goLeft=0;
          }
          else if (!(l%10) && goRight) { // else if right
            currCol++;
            if (currCol > 4) { currCol = 4; }
            goRight=0;
          }
          if (prevCol != currCol) { // if moved
            if (prevCol == 4) drawfillrectME(0, 0, 59, 39, 0xffff);
            else drawfillrectME(60*(4-prevCol) + 1, 0, 60*(4-prevCol) - 1 + 60, 39, 0xffff);
            if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
            else drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
          }


          // move tile down 1 pixel 
          drawfillrectME(left-58,l-50,left,l,0x0000); 
          nano_wait(time * 3);
          drawlineME(left-58,l,left,l,0xffff);

          if (l <= 0) { // game ends 
            if ((lane1 == 1) || (lane2 == 2) || (lane3 == 3) || (lane4 == 4)){
              game = 0;
              // highscore check
              if (currScore > highscore) highscore = currScore;
              break;
            }
          }
          // check for scoring 
          // check what lane the box is in
          if (l < 79) { // 79 is top of cursor
            if (box_num == 4) lane1 = 1;
            else if (box_num ==3) lane2 = 2;
            else if (box_num == 2) lane3 = 3;
            else if (box_num == 1) lane4 = 4;
          }

          if (lane1 == currCol){
            drawfillrectME(left-58,40,left,320,0xffff);
            currScore++;
            lane1 = 0;
            drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
            break;
          }
          else if (lane2 == currCol) {
            drawfillrectME(left-58,40,left,320,0xffff);
            currScore++;
            lane2 = 0;
            drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
            break;
          }
          else if (lane3 == currCol) {
            drawfillrectME(left-58,40,left,320,0xffff);
            currScore++;
            lane3 = 0;
            drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
            break;
          }
          else if (lane4 == currCol) {
            drawfillrectME(left-58,40,left,320,0xffff);
            currScore++;
            lane4 = 0;
            drawfillrectME(0, 0,59, 39, 0x0000);
            break;
          }
          
        }
      }
      if (currScore > highscore) highscore = currScore;
      snprintf(string, sizeof(string), "%4d%4d", highscore, currScore);
      print(string);
      timeIncrease += 1;
    }
  
  
  // write highscore to eeprom 
  // highscore=0; // for resetting high score to 0
  char writeScore[32];
  snprintf(writeScore, sizeof(writeScore), "%4d", highscore);
  eeprom_write(0, writeScore, 32);

  // write "GAME OVER" on tft
  clearME(0xffff);
  int color = 0x0000;
  int x = 48;
  int y = 160;
  // Flip the y-axis and mirror the x-axis for the correct origin (bottom-right corner)
  x = 240 - x;  // Mirror x-axis (from 0-240 to 240-0)
  y = 320 - y;  // Flip the y-axis (from 0-320 to 320-0)
  // Draw "G"
  drawlineME(x, y, x + 16, y, color);                // Top horizontal
  drawlineME(x + 16, y, x + 16, y - 24, color);                // Left vertical
  drawlineME(x, y - 24, x + 16, y - 24, color);      // Bottom horizontal
  drawlineME(x , y - 12, x + 8, y - 12, color);  // Middle horizontal
  drawlineME(x , y - 12, x , y - 24, color); // Right vertical (bottom half)
  // // Draw "A"
  drawlineME(x - 10, y, x -2, y - 24, color);            // Left diagonal (inverted)
  drawlineME(x - 10, y, x - 18, y - 24, color);       // Right diagonal (inverted)
  drawlineME(x -6, y - 12, x - 13, y - 12, color);  // Middle horizontal
  // Draw "M"
  drawlineME(x - 20, y, x - 20, y - 24, color);                // Left vertical
  drawlineME(x - 36, y, x - 36, y - 24, color);      // Right vertical
  drawlineME(x - 20, y, x - 28, y - 12, color);            // Left diagonal (inverted)
  drawlineME(x -36, y, x -28, y - 12, color);       // Right diagonal (inverted)
  // Draw "E"
  drawlineME(x -40, y, x - 40, y - 24, color);                // Left vertical
  drawlineME(x - 40, y, x - 56, y, color);                // Top horizontal
  drawlineME(x - 40, y - 12, x - 56, y - 12, color);      // Middle horizontal
  drawlineME(x - 40, y - 24, x - 56, y - 24, color);      // Bottom horizontal
  x -= 90;  // Extra space between words
  // Draw "O"
  drawlineME(x, y, x + 16, y, color);                // Top horizontal
  drawlineME(x, y, x, y - 24, color);                // right vertical
  drawlineME(x + 16, y, x + 16, y - 24, color);      // left vertical
  drawlineME(x, y - 24, x + 16, y - 24, color);      // Bottom horizontal
  // Draw "V"
  drawlineME(x - 2, y, x - 10, y - 24, color);            // Left diagonal (inverted)
  drawlineME(x - 18, y, x - 10, y - 24, color);       // Right diagonal (inverted)
  // Draw "E" (same as before)
  drawlineME(x - 22, y, x - 22, y - 24, color);                // Left vertical
  drawlineME(x - 22, y, x  - 38, y, color);                // Top horizontal
  drawlineME(x - 22, y - 12, x - 38, y - 12, color);      // Middle horizontal
  drawlineME(x - 22, y - 24, x - 38, y - 24, color);      // Bottom horizontal
  // Draw "R"
  drawlineME(x - 42, y, x - 42, y - 24, color);                // Left vertical
  drawlineME(x - 42, y, x - 54, y, color);                // Top horizontal
  drawlineME(x - 54, y, x - 54, y - 12, color);      // Right vertical (top half)
  drawlineME(x - 42, y - 12, x  - 54, y - 12, color);      // Middle horizontal
  drawlineME(x - 42, y - 12, x - 54, y - 24, color);      // Diagonal leg
  drawlineME(x - 42, y - 11, x - 54, y - 23, color);      // Diagonal leg
}
