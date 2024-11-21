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

#include "lcd.h"
#include "commands.h"
#include "header.h"
#include <stdio.h>
// from step 4:
#include "fifo.h"
#include "tty.h"
int i2c_checknack(void);
void i2c_clearnack(void);
// DMA data structures
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


int main() {
    internal_clock();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    enable_ports();
    init_spi2();
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
    srand(5);

    // joystick:
    setup_adc();
    init_tim2();

    // eeprom i2c:
    init_i2c();
    
    char string[32];

    // read last high score from eeprom
    char lasthigh[32];
    eeprom_read(0, lasthigh, 32);
    // use sscanf to convert char * lasthigh to int highscore
    sscanf(lasthigh, "%d", &highscore);
    snprintf(string, sizeof(string), "%4d%4d", highscore, 0);
    print(string);

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