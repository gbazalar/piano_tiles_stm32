/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Feb 7, 2024
  * @brief   ECE 362 Lab 7 student template
  ******************************************************************************
*/

/*******************************************************************************/

// Fill out your username!  Even though we're not using an autotest, 
// it should be a habit to fill out your username in this field now.
const char* username = "griff323";

/*******************************************************************************/ 

#include "stm32f0xx.h"
#include <stdint.h>

void internal_clock();
// void drawfillrect(int x1, int y1, int x2, int y2, int c);
// void nano_wait(unsigned int);
// void lcd_init(int argc, char *argv[]);
#include "lcd.h"
#include "commands.h"
#include "header.h"
// #include "lcd.h"
#include <stdio.h>
// from step 4:
#include "fifo.h"
#include "tty.h"

// TODO DMA data structures
#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;
void enable_ports();
void show_char(int n, char c);
void drive_column(int c);
int read_rows();
char rows_to_key(int rows);
uint8_t col = 0;
char c;
// Make it easier to access keymap
extern char keymap;
char* keymap_arr = &keymap;
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
  // commented out stuff is from lab3 for the keypad 
  // RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  // GPIOC->MODER |= GPIO_MODER_MODER4_0;
  // GPIOC->MODER &= ~GPIO_MODER_MODER4_1;
  // GPIOC->MODER |= GPIO_MODER_MODER5_0;
  // GPIOC->MODER &= ~GPIO_MODER_MODER5_1;
  // GPIOC->MODER |= GPIO_MODER_MODER6_0;
  // GPIOC->MODER &= ~GPIO_MODER_MODER6_1;
  // GPIOC->MODER |= GPIO_MODER_MODER7_0;
  // GPIOC->MODER &= ~GPIO_MODER_MODER7_1;
  // GPIOC->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_0 |GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER3_1);
  // GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR0_0;
  // GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;
  // GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR1_0;
  // GPIOC->PUPDR |= GPIO_PUPDR_PUPDR1_1;
  // GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR2_0;
  // GPIOC->PUPDR |= GPIO_PUPDR_PUPDR2_1;
  // GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR3_0;
  // GPIOC->PUPDR |= GPIO_PUPDR_PUPDR3_1;

  // i2c for eeprom
    // pa9 i2c1_scl, pa10 i2c1_sck
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  // GPIOA->MODER &= ~(GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0);
  // GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1; // alt func 10
  // GPIOA->AFR[1] |= 0x00000110; // AF4 0100 for 9, 10

  // lab 6 for spi - show current score
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  GPIOB->MODER &= ~(0xCF000000);
  GPIOB->MODER |= 0x45000000;
  GPIOB->BSRR = GPIO_BSRR_BS_12;
  GPIOB->BSRR = GPIO_BSRR_BR_13;
  
  // stuff to enable ports for button input 
  // enable GPIOA for button input 
  // PA0 left, PA1 right 
  GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER0_1); 
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR2_1;
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR2_0);

  // eeprom
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable port B
  GPIOB->MODER &= ~(0x0000F000); //clear PB6-7
  GPIOB->MODER |= 0x0000A000; //set to alt function mode for PB6-7

  GPIOB->AFR[0] |= 0x1 << (6 * 4); //set to AF1
  GPIOB->AFR[0] |= 0x1 << (7 * 4); //set to AF1
  // GPIOB->OTYPER |= 0x00001000;
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

// void init_spi1_slow(void) { 
//     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
//     RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
//     // PB3 SCK, PB4 MISO, PB5 MOSI 
//     GPIOB->MODER |= 0x00000A80; // alt fctn 10
//     GPIOB->AFR[0] &= ~(0x00FFF000); // AF0 - 0000
//     SPI1->CR1 &= ~SPI_CR1_SPE;
//     // baud rate max, master, ssm, ssi
//     SPI1->CR1 |=  SPI_CR1_BR | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
//     // data size 8. fifo reception 
//     SPI1->CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; 
//     SPI1->CR2 &= ~(SPI_CR2_DS_3);
//     SPI1->CR2 |= SPI_CR2_FRXTH;
//     SPI1->CR1 |= SPI_CR1_SPE;
// }
// initialize spi2
void init_spi2(void) { 
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    // now set GPIOB pins to alt funct ???? Huh???
    // PB12 (CS-NSS), PB13 (SCK), PB15 (SDI-MOSI) - 10 
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1;
    GPIOB->MODER &= ~(GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER15_0);
    GPIOB->AFR[1] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7);
    // GPIOB->MODER &= ~(0xFF000000);
    // GPIOB->MODER |= 0x8A000000;
    // GPIOB->AFR[1] &= ~(0xF0FF0000);
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



// from i2c lab 
// configure i2c1
// void init_i2c(void) {
//     RCC-> RCC_APB1ENR_I2C1EN;
// }
//from lab 1 for reading button value
int32_t readpin(int32_t pin_num);
int32_t readpin(int32_t pin_num) {
  // input: pin number
  // output: 1 (pressed) or 0
  int val = 1<<pin_num;
  if (GPIOA->IDR & val) { return 0x1; } // pin is high
  else { return 0x0; } // pin is low
}




// keypad functions from lab
void show_char(int n, char c) {
  // if ((n >= 0) && (n <= 7)) {
  //   GPIOB->ODR = font[c] | (n<<8);
  // }
  // else return;
}
void drive_column(int c) {
    GPIOC->BSRR |=  0xF<<(4 + 16); //bitshift 4 1s
  //c = 0b11 && c; //get least significant bit values
    GPIOC->BSRR |= 1<<((0x3 & c) + 4);
}
int read_rows() {//TA PLEASE CHECK IF THIS IS THE RIGHT IDEA
  int8_t row = 0; //TA how to initialize 4 bit int? is 8 fie cause it just reads the last 4 bits?
  for (int curr_row = 3; curr_row >= 0; curr_row--){
    int8_t temp = 1<<curr_row;
    if ((GPIOC->IDR) & temp) {
      row |= 1<<curr_row;
    }
    
  }
  return (row);

}
char rows_to_key(int rows) {
  // char c = 0;
    // for (int col = 7; col >= 4; col--) {
    //   int8_t temp = 1<<col;
    //   if (GPIOC->IDR & temp) {
    //     c = keymap_arr[(col)*4 + rows];
    //   }
    // }
    int row = 0;
    char c;
    if(rows & 0x1) row = 0;
    else if(rows & 0x2) row = 0x1;
    else if(rows & 0x4) row = 0x2;
    else if(rows & 0x8) row = 0x3;

    // if(rows & 0x4) row = 0x3;
    // else if (rows & 0x3) row = 0x2;
    // else if (rows & 0x2) row = 0x1;
    // else if (rows & 0x1) row = 0x0;
    
    if (row >= 0) {
      c = keymap_arr[(col)*4 + row];
    }
    return (c);
}
int __io_putchar(int c) {
    // TODO copy from STEP2
    if(c == '\n') {
        while(!(USART5->ISR & USART_ISR_TXE));
        USART5->TDR = '\r';
    }
    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

// TFT EXTRA
void init_spi1_slow(void) { 
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    // PB3 SCK, PB4 MISO, PB5 MOSI 

    GPIOB->MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
    GPIOB->MODER &= ~(GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0);
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5);
    

    // GPIOB->MODER &= ~(0xFF0);
    // GPIOB->MODER |= 0x00000A80; // alt fctn 10
    // GPIOB->AFR[0] &= ~(0x00FFF000); // AF0 - 0000

    //From SPI2:
    //GPIOB->MODER &= ~(0xFF000000);
    //GPIOB->MODER |= 0x8A000000;
    //GPIOB->AFR[1] &= ~(0xF0FF0000);
    //From Spi2
    
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

// initialize adc to use floating pin for srand seed - actually don't
//  based on lab 4
// void setup_adc(void);
// int read_adc(void);
// void setup_adc(void) {
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//     // pin associated with adc_in1 to analog
//     GPIOA->MODER |= 0x0000000C;
//     RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//     // enable adc by setting aden bit in CRC
//     ADC1->CR |= ADC_CR_ADEN;
//     // wait for adc 
//     while(!(ADC1->ISR & ADC_ISR_ADRDY));
//     // select channel for adc_in1 in chselr
//     ADC1->CHSELR |= (1<<1);
//     // wait for adc 
//     while(!(ADC1->ISR & ADC_ISR_ADRDY));
// }
// int read_adc(void) {
//     ADC1->CR |= ADC_CR_ADSTART; // enable to start conversion
//     while (!(ADC1->ISR & ADC_ISR_EOC)); // wait until conversion done
//     return ADC1->DR; // return int val read
// }

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
    // RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // I2C1->CR1 &= ~I2C_CR1_PE; //disable I2C first
    // // I2C1->CR1 |= I2C_CR1_ERRIE; //error interrupt enabled
    // // I2C1->CR1 |= I2C_CR1_ANFOFF; //disable analog filter

    // //CHANGE ABOVE

    // I2C1->CR1 |= I2C_CR1_NOSTRETCH; //disable clock stretching
    // I2C1->TIMINGR = 0;
    // I2C1->TIMINGR &= ~I2C_TIMINGR_PRESC;
    // I2C1->TIMINGR |= (5 << 28 | 3 << 20 | 3 << 16 | 3 << 8 | 9 << 0); //6 * 8Mhz = 48Mhz clock, 400 kHZ fast mode (supported by EEPROM)
    // //CHANGE ABOVE
    // I2C1->CR2 &= ~I2C_CR2_ADD10; //7-bit addressing
    // I2C1->CR2 &= ~I2C_CR2_AUTOEND; //send stop after last NBytes is transferred
    // I2C1->CR1 |= I2C_CR1_PE; //enable I2C




    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; //enable peripheral clock
    I2C1->CR1 &= ~I2C_CR1_PE; //peripheral disabled

    // I2C1->CR1 &= ~I2C_CR1_ANFOFF; //turn off analog noise filter
    // I2C1->CR1 &= ~I2C_CR1_ERRIE; //turn on error detection interrupts
    I2C1->CR1 |= I2C_CR1_NOSTRETCH; //disable clock stretching

    I2C1->TIMINGR &= ~(0xF << 28); //clear all registers
    I2C1->TIMINGR |= 0x5 << 28; //set prescalar to 5
    I2C1->TIMINGR |= 0x3 << 20; //set scldel
    I2C1->TIMINGR |= 0x3 << 16; //set sdadel
    I2C1->TIMINGR |= 0x3 << 8; //set sclh
    I2C1->TIMINGR |= 0x9; //set scll
    // I2C1->TIMINGR = (uint32_t)0x00B01A4B;
    
    I2C1->CR2 &= ~I2C_CR2_ADD10; //7-bit addressing mode
    //I2C1->CR2 |= I2C_CR2_AUTOEND; //automatically send a STOP condition after last byte of transmission ???
    I2C1->CR1 |= I2C_CR1_PE; //peripheral enabled
}

//===========================================================================
// Send a START bit.
//===========================================================================
void i2c_start(uint32_t targadr, uint8_t size, uint8_t dir) {
    // uint32_t tempCR2 = I2C1->CR2;
    // tempCR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP); //clearing NBytes, STOP, START, RD_WRN, SADD
    // if (dir){
    //     tempCR2 |= I2C_CR2_RD_WRN; //operation is a read
    // }
    // else {
    //     tempCR2 &= ~I2C_CR2_RD_WRN;
    // }
    // tempCR2 |= ((targadr<<1) & I2C_CR2_SADD) | ((size << 16) & I2C_CR2_NBYTES); //set target address and data size
    // tempCR2 |= I2C_CR2_START; //prepare start of read/write
    // I2C1->CR2 = tempCR2;





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
    //  //Check if stop bit is set
    // if (I2C1->ISR & I2C_ISR_STOPF) {
    //     return;
    // }
    // I2C1->CR2 |= I2C_CR2_STOP;
    // while (!(I2C1->ISR & I2C_ISR_STOPF)); //wait for STOP 
    // I2C1->ICR |= I2C_ICR_STOPCF; //clear stop flag




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
    // i2c_waitidle(); //wait for I2C to be idle
    // i2c_start(targadr, size, 0); //send START with write bit (dir = 0)
    // for (int i = 0; i < size; i++)
    // {
    //     int count = 0;
    //     while (!(I2C1->ISR & I2C_ISR_TXIS)) {
    //         count += 1;
    //         if (count > 1000000)
    //             return -1;
    //         if (i2c_checknack()) {
    //             i2c_clearnack();
    //             i2c_stop();
    //             return -1;
    //         }   
    //     }
    // I2C1->TXDR = data[i] & I2C_TXDR_TXDATA; //mask data[i] with TXDR_TXDATA
    // }
    // while ((!(I2C1->ISR & I2C_ISR_TC) && !(I2C1->ISR & I2C_ISR_NACKF))); //wait for these to be not set

    // //CHANGE ABOVE

    // // i2c_waitidle();


    // if (I2C1->ISR & I2C_ISR_NACKF){
    //     i2c_clearnack();
    //     i2c_stop();
    //     return -1;
    //     //CHANGE ABOVE


    //     // return -1; //write failed
    // }

    // // if (randRead == 0) {
    // //     i2c_stop(); //send STOP
    // // }
    // //CHANGE ABOVE

    // i2c_stop();
    // return 0; //0 for success





    //i2c_waitidle();
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
    //i2c_waitidle();

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
    // if (size <= 0 || data == 0){
    //     return -1;
    // }
    
    // i2c_waitidle(); //wait for I2C to be idle
    // i2c_start(targadr, size, 1); //send START with read bit (dir = 1)

    // uint8_t i;
    // for (i = 0; i < size; i++) {
    //     int count = 0;
    //     while ((I2C1->ISR & I2C_ISR_RXNE) == 0) {
    //         count += 1;
    //         if (count > 1000000)
    //             return -1;
    //         if (i2c_checknack()) {
    //             i2c_clearnack();
    //             i2c_stop();
    //             return -1;
    //         }
    //     }  
    //     ((uint8_t*)data)[i] = (I2C1->RXDR & I2C_RXDR_RXDATA);
    //     // ((uint8_t*)data)[i] = (I2C1->RXDR & I2C_RXDR_RXDATA); //store masked data
    // }

    // // while ((I2C1->ISR & I2C_ISR_TC) == 0 && (I2C1->ISR & I2C_ISR_NACKF) == 0);

    // // if((I2C1->ISR & I2C_ISR_NACKF) != 0){
    // //     return -1;
    // // }

    // // i2c_stop();

    // i2c_waitidle();
    // i2c_stop();
    // return 0;
    // //CHANGE ABOVE

    // // return 0;





    //i2c_waitidle();
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


int main() {
    internal_clock();
    // init_usart5();
    // enable_tty_interrupt();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    enable_ports();
    // init_exti();
    // command_shell();
    // init_spi1_slow();
    init_spi2();
    // drawfillrectME(0,0,200,200,0x0000);
    spi2_setup_dma();
    spi2_enable_dma();
    init_tim15();
    lcd_initME();
    nano_wait(5000);
    clearME(0xffff); // clear screen to start
    // make 4 columns (3 lines)
    drawlineME(60,0,60,320, 0xadd8e6);
    drawlineME(120,0,120,320, 0xadd8e6);
    drawlineME(180,0,180,320, 0xadd8e6);
    drawfillrectME(121,0,179,39,0x0000); 
    // setup_adc();
    srand(5);

    // joystick:
    setup_adc();
    init_tim2();

    // eeprom i2c:
    init_i2c();
    
    char string[32];

    // test i2c:
    // char writeArr[32];
    // snprintf(writeArr, sizeof(writeArr), "%4d%4d", 4, 3);
    // eeprom_write(0, writeArr, 32);
    // nano_wait(1000000);
    // char readArr[32];
    // eeprom_read(0, readArr, 32);
    // print(readArr);
    
    // snprintf(string, sizeof(string), "%4d%4d", highscore, 5);
    // print(string);

    // read last high score from eeprom
    char lasthigh[32];
    eeprom_read(0, lasthigh, 32);
    print(lasthigh);
    // use sscanf to convert char * lasthigh to int highscore? 
    sscanf(lasthigh, "%d", &highscore);

    // snprintf(string, sizeof(string), "%4d", highscore);
  
    // init_spi2();
    // // drawfillrectME(0,0,200,200,0x0000);
    // spi2_setup_dma();
    // spi2_enable_dma();
    // init_tim15();
    // lcd_initME();
    // init_lcd_spi();
    int game = 1;

    int time = 500000;
    int timeIncrease = 0;

    while (game) {
      time = time / 100;
       //DEVELOPING SCORING: ONLY 1 BOX 
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
          if (joyVal > 3200) goRight = 1;
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
            // if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
            drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
            // if (box_num == 3) box1On = 0;
            // if (boxTwo == 4) box2On = 0;
            // break;
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
            // if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
            drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
            // if (box_num == 2) box1On = 0;
            // if (boxTwo == 4) box2On = 0;
            // break;
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
            // if (box_num == 1) box1On = 0;
            // if (boxTwo == 4) box2On = 0;
            // break;
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
          // reset score check variables

          // cursor move check
          if (joyVal < 2900) goLeft = 1;
          if (joyVal > 3200) goRight = 1;

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
            // if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
            drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
            break;
          }
          else if (lane3 == currCol) {
            drawfillrectME(left-58,40,left,320,0xffff);
            currScore++;
            lane3 = 0;
            // if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
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

  // x += 20;  // Move to next letter position

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


// unused functions garbage pile: 

// void enable_tty_interrupt(void) {
//     // TODO
//     USART5->CR1 |= USART_CR1_RXNEIE;
//     // set proper bit in NVIC ISER - ??? 
//     NVIC_EnableIRQ(USART3_8_IRQn);
//     USART5->CR3 |= USART_CR3_DMAR; 
//     RCC->AHBENR |= RCC_AHBENR_DMA2EN;
//     DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
//     DMA2_Channel2->CCR &= ~DMA_CCawfillrect 0 R_EN;  // First make sure DMA is turned off
//     // The DMA channel 2 configuration goes here
//     DMA2_Channel2->CMAR = (uint32_t)&serfifo;
//     DMA2_Channel2->CPAR = (uint32_t)&(USART5->RDR);
//     DMA2_Channel2->CNDTR |= FIFOSIZE;
//     DMA2_Channel2->CCR &= ~(DMA_CCR_DIR | DMA_CCR_HTIE); // CHANGED FROM STEP 4 
//     DMA2_Channel2->CCR &= ~(DMA_CCR_MSIZE|DMA_CCR_PSIZE);
//     DMA2_Channel2->CCR |= DMA_CCR_MINC;
//     DMA2_Channel2->CCR &= ~DMA_CCR_PINC;
//     DMA2_Channel2->CCR |= DMA_CCR_CIRC;
//     DMA2_Channel2->CCR &= ~DMA_CCR_MEM2MEM;
//     DMA2_Channel2->CCR |= DMA_CCR_PL_1 | DMA_CCR_PL_0;
//     DMA2_Channel2->CCR |= DMA_CCR_TCIE; // CHANGED FROM STEP4
//     DMA2_Channel2->CCR |= DMA_CCR_EN;
// }

// // Works like line_buffer_getchar(), but does not check or clear ORE nor wait on new characters in USART
// char interrupt_getchar() {
//     // TODO
//     // Wait for a newline to complete the buffer.
//     while(fifo_newline(&input_fifo) == 0) {
//         asm volatile ("wfi"); // wait for an interrupt
//     }
//     // Return a character from the line buffer.
//     char ch = fifo_remove(&input_fifo);
//     return ch;
// }

// int __io_getchar(void) {
//     // TODO Use interrupt_getchar() instead of line_buffer_getchar()
//     char c = interrupt_getchar();
//     return c;
// }
// char get_keypress(void);
// char get_keypress() {
//     char event;
//     for(;;) {
//         // Wait for every button event...
//         event = get_key_event();
//         // ...but ignore if it's a release.
//         if (event & 0x80)
//             break;
//     }
//     return event & 0x7f;
// }
// char pop_queue(void);
// char pop_queue() {
//     char tmp = queue[qout];
//     queue[qout] = 0;
//     qout ^= 1;
//     return tmp;
// }
// char get_key_event(void);
// char get_key_event(void) {
//     for(;;) {
//         asm volatile ("wfi");   // wait for an interrupt
//         if (queue[qout] != 0)
//             break;
//     }
//     return pop_queue();
// }
// char get_keypress() {
//     char event;
//     for(;;) {
//         // Wait for every button event...
//         event = get_key_event();
//         // ...but ignore if it's a release.
//         if (event & 0x80)
//             break;
//     }
//     return event & 0x7f;
// }
// void enable_ports(void) {
//     // Only enable port C for the keypad
//     RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
//     GPIOC->MODER &= ~0xffff;
//     GPIOC->MODER |= 0x55 << (4*2);
//     GPIOC->OTYPER &= ~0xff;
//     GPIOC->OTYPER |= 0xf0;
//     GPIOC->PUPDR &= ~0xff;
//     GPIOC->PUPDR |= 0x55;
// }
