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
// Font array in assembly file as I am too lazy to convert it into C array
extern uint8_t font[];

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
int currScore = 0; // score


void enable_ports() {
  // enable GPIOA for button input 
  // PA0 left, PA1 right 
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER0_1); 
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR2_1;
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR2_0);
}

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
  if ((n >= 0) && (n <= 7)) {
    GPIOB->ODR = font[c] | (n<<8);
  }
  else return;
}
void drive_column(int c) {
    GPIOC->BSRR |=  0xF<<(4 + 16); //bitshift 4 1s
  //c = 0b11 && c; //get least significant bit values
    GPIOC->BSRR |= 1<<((0x3 & c) + 4);
}
int read_rows() {
  int8_t row = 0; //TA how to initialize 4 bit int? is 8 fie cause it just reads the last 4 bits? - I think it's more than 8? 32? 
  for (int curr_row = 3; curr_row >= 0; curr_row--){
    int8_t temp = 1<<curr_row;
    if ((GPIOC->IDR) & temp) {
      row |= 1<<curr_row;
    }
    
  }
  return (row);

}
char rows_to_key(int rows) {
    int row = 0;
    char c;
    if(rows & 0x1) row = 0;
    else if(rows & 0x2) row = 0x1;
    else if(rows & 0x4) row = 0x2;
    else if(rows & 0x8) row = 0x3;

    if (row >= 0) {
      c = keymap_arr[(col)*4 + row];
    }
    return (c);
}
void nano_wait(int t);
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
    GPIOB->MODER |= 0x00000A80; // alt fctn 10
    GPIOB->AFR[0] &= ~(0x00FFF000); // AF0 - 0000
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

// initialize adc to use floating pin for srand seed
//  based on lab 4
void setup_adc(void);
int read_adc(void);

void setup_adc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // pin associated with adc_in1 to analog
    GPIOA->MODER |= 0x0000000C;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    // enable adc by setting aden bit in CRC
    ADC1->CR |= ADC_CR_ADEN;
    // wait for adc 
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    // select channel for adc_in1 in chselr
    ADC1->CHSELR |= (1<<1);
    // wait for adc 
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
}
int read_adc(void) {
    ADC1->CR |= ADC_CR_ADSTART; // enable to start conversion
    while (!(ADC1->ISR & ADC_ISR_EOC)); // wait until conversion done
    return ADC1->DR; // return int val read
}
 

//-------------------------------
// Timer 7 ISR goes here
//-------------------------------
void TIM7_IRQHandler () {
  TIM7->SR &= ~TIM_SR_UIF;

  // FOR DEVELOPING SCORING: ONLY 1 BOX 
  // tileNum = (rand() % 2) + 1;
  tileNum = 1;
    if (tileNum == 2) {
      box_num = (rand() % 4)+1;
      boxTwo = (rand() % 4) + 1;
      left = box_num * 60 - 1; // left pixel of box at box_num * 60 - 1
      leftTwo = boxTwo * 60 - 1;
      
      // loop to shift tiles down
      for (int l = 320; l >= 0; l--) { 
        // cursor check
        prevCol = currCol;
        if (!(l%5) && readpin(0)) { // if left
          currCol--;
          if (currCol <= 0) { currCol = 1; }
        }
        else if (!(l%5) && readpin(2)) { // else if right
          currCol++;
          if (currCol >= 4) { currCol = 4; }
        }
        if (prevCol != currCol) {
          if (prevCol == 4) drawfillrectME(0, 0, 59, 39, 0xffff);
          else drawfillrectME(60*(4-prevCol) + 1, 0, 60*(4-prevCol) - 1 + 60, 39, 0xffff);
          if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
          else drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
        }
        
        // move tiles down 1 pixel
        drawfillrectME(left-58,l-50,left,l,0x0000); 
        drawfillrectME(leftTwo-58,l-50,leftTwo,l,0x0000); 
        nano_wait(500000);
        drawlineME(left-58,l,left,l,0xffff);
        drawlineME(leftTwo-58,l,leftTwo,l,0xffff);
      }
    }
    else if (tileNum == 1) {
      box_num = (rand() % 4)+1;
      left = box_num * 60 - 1; // left pixel of box at box_num * 60 - 1
      
      // loop to shift tiles down
      for (int l = 320; l >= 0; l--) { 
        // cursor check
        prevCol = currCol;
        if (!(l%5) && readpin(0)) { // if left
          currCol--;
          if (currCol <= 0) { currCol = 1; }
        }
        else if (!(l%5) && readpin(2)) { // else if right
          currCol++;
          if (currCol >= 4) { currCol = 4; }
        }
        if (prevCol != currCol) {
          if (prevCol == 4) drawfillrectME(0, 0, 59, 39, 0xffff);
          else drawfillrectME(60*(4-prevCol) + 1, 0, 60*(4-prevCol) - 1 + 60, 39, 0xffff);
          if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
          else drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
        }

        // check for scoring 
        // check what lane the box is in
        if (l < 79) { // 79 is top of cursor
          if (box_num == 1) lane1 = 1;
          if (box_num == 2) lane2 = 1;
          if (box_num == 3) lane3 = 1;
          if (box_num == 4) lane4 = 1;
        }

        if (lane1 == currCol){
          drawfillrectME(left-58,40,left,320,0xadd8e6);
          // l = 0;
          // currScore++;
          lane1 = 0;
          if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
          else drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
          break;
        }
        else if (lane2 == currCol) {
          drawfillrectME(left-58,40,left,320,0xadd8e6);
          // l = 0;
          // currScore++;
          lane2 = 0;
          if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
          else drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
          break;
        }
        else if (lane3 == currCol) {
          drawfillrectME(left-58,40,left,320,0xadd8e6);
          // l = 0;
          // currScore++;
          lane3 = 0;
          if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
          else drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
          break;
        }
        else if (lane4 == currCol) {
          drawfillrectME(left-58,40,left,320,0xadd8e6);
          // l = 0;
          // currScore++;
          lane4 = 0;
          if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
          else drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
          break;
        }
        drawfillrectME(left-58,l-50,left,l,0x0000); 
        // drawfillrectME(leftTwo-58,l-50,leftTwo,l,0x0000); 
        nano_wait(1500000);
        drawlineME(left-58,l,left,l,0xffff);
        // drawlineME(leftTwo-58,l,leftTwo,l,0xffff);
      }
      lane1 = 0;
      lane2 = 0;
      lane3 = 0;
      lane4 = 0;
    }
}


/**
 * @brief Setup timer 7 as described in lab handout
 * 
 */
void setup_tim7() {
  //TA HELP how value to call to enable RCC for TIM7
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

  TIM7->PSC = 48 - 1;
  TIM7->ARR = 11500 - 1;
  TIM7->DIER |= TIM_DIER_UIE;
  NVIC->ISER[0] |= 1<<TIM7_IRQn;
  TIM7->CR1 |= TIM_CR1_CEN;
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

    lcd_initME();
    nano_wait(5000);
    clearME(0xffff); // clear screen to start
    // make 4 columns (3 lines)
    drawlineME(60,0,60,320, 0xadd8e6);
    drawlineME(120,0,120,320, 0xadd8e6);
    drawlineME(180,0,180,320, 0xadd8e6);
    // char c;
    // int col;
    // int currCol = 2;
    // int prevCol = 2;
    drawfillrectME(121,0,179,39,0x0000); // GA: what does this do
    setup_adc();
    srand(read_adc());
    setup_tim7();
}
