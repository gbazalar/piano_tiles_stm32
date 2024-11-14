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

  // new stuff to enable ports for button input 
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
  // if (read_rows() != 0) {
  //   // handle_key(rows_to_key(read_rows()));
  //   c = rows_to_key(read_rows());
  //   prevCol = currCol;
  //   if (c == 'A') {
  //     currCol -= 1;
  //     if (currCol <= 0) currCol = 1;
  //   }
  //   else if (c == 'B') {
  //     currCol += 1;
  //     if (currCol >= 4) currCol = 4;
  //   }
  // }
  // // char dispChar = disp[col];
  // // show_char(col,dispChar);
  // if (prevCol != currCol) {
  //   if (prevCol == 4) drawfillrectME(0, 0, 59, 39, 0xffff);
  //   else drawfillrectME(60*(4-prevCol) + 1, 0, 60*(4-prevCol) - 1 + 60, 39, 0xffff);
  //   if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
  //   else drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
  // }
  // col++;
  // if (col > 7) col = 0;
  // drive_column(col);

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

        // if (l <= 79) {
        //   switch (box_num) {
        //     case 1:
        //       lane1 = 1;
        //       break;
        //     case 2:
        //       lane2 = 1;
        //       break;
        //     case 3:
        //       lane3 = 1;
        //       break;
        //     case 4:
        //       lane4 = 1;
        //       break;
        //   }

        //   switch (boxTwo) {
        //     case 1:
        //       lane1 = 1;
        //       break;
        //     case 2:
        //       lane2 = 1;
        //       break;
        //     case 3:
        //       lane3 = 1;
        //       break;
        //     case 4:
        //       lane4 = 1;
        //       break;
        //   }
        // }
        
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
    // for (;;) {
    //     if (read_rows() != 0) {
            // c = rows_to_key(read_rows());
            // prevCol = currCol;
            // if (c == 'A') {
            //   currCol -= 1;
            //   if (currCol <= 0) currCol = 1;
            //   }
            // else if (c == 'B') {
            //   currCol += 1;
            //   if (currCol >= 4) currCol = 4;
            //   }
            // else if (c == 'C') currCol =3;
            // else if (c == 'D') currCol = 4;
        // }
        
        // if (prevCol != currCol) {
        //     if (prevCol == 4) drawfillrectME(0, 0, 59, 39, 0xffff);
        //     else drawfillrectME(60*(4-prevCol) + 1, 0, 60*(4-prevCol) - 1 + 60, 39, 0xffff);
        //     if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
        //     else drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
        // }
        // col++;
        // if (col > 7) col = 0;
        // drive_column(col);
        // nano_wait(20000000);
    // }
    // int box_num;
    // int boxTwo;
    // int left;
    // int leftTwo;
    // int tileNum;
    // rand() % (max - min + 1) + min = (4-1+1)+1
    // use adc pin val as srand seed 
    // setup_adc();
    // srand(read_adc());

    // make 5 random boxes 
    // one box on the screen at once 
    // for (int i=0; i<=25; i++) {
    //     tileNum = (rand() % 2) + 1;
    //     if (tileNum == 2) {
    //         box_num = (rand() % 4)+1;
    //         boxTwo = (rand() % 4) + 1;
    //         // left side of box at box_num * 60 - 1
    //         left = box_num * 60 - 1;
    //         leftTwo = boxTwo * 60 - 1;
    //         for (int l = 320; l >= 0; l--) {
    //             drawfillrectME(left-58,l-50,left,l,0x0000); 
    //             drawfillrectME(leftTwo-58,l-50,leftTwo,l,0x0000); 
    //             nano_wait(5000000);
    //             drawlineME(left-58,l,left,l,0xffff);
    //             drawlineME(leftTwo-58,l,leftTwo,l,0xffff);
    //         }
    //     }
    //     else if (tileNum == 1) {
    //         box_num = (rand() % 4)+1;
    //         // boxTwo = (rand() % 4) + 1;
    //         // left side of box at box_num * 60 - 1
    //         left = box_num * 60 - 1;
    //         // leftTwo = boxTwo * 60 - 1;
    //         for (int l = 320; l >= 0; l--) {
    //             drawfillrectME(left-58,l-50,left,l,0x0000); 
    //             // drawfillrectME(leftTwo-58,l-50,leftTwo,l,0x0000); 
    //             nano_wait(10000000);
    //             drawlineME(left-58,l,left,l,0xffff);
    //             // drawlineME(leftTwo-58,l,leftTwo,l,0xffff);
    //         }
    //     }
    // }
    //     clearME(0x0000); // clear screen to end
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

// void USART3_8_IRQHandler(void) {
//     while(DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
//         if (!fifo_full(&input_fifo))
//             insert_echo_char(serfifo[seroffset]);
//         seroffset = (seroffset + 1) % sizeof serfifo;
//     }
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

// void drawfillrect(int x1, int y1, int x2, int y2, int c)
// {
//     lcddev.select(1);
//     _LCD_Fill(x1,y1,x2,y2,c);
//     lcddev.select(0);
// }

// exti2_3 was used for using key inputs as left/ right, switched to buttons 
// void EXTI2_3_IRQHandler() {
//   EXTI->PR = EXTI_PR_PR2 | EXTI_PR_PR3;
//   // togglexn(GPIOB, 10); call whatever thing we want
//   if (read_rows() != 0) {
//     // handle_key(rows_to_key(read_rows()));
//     c = rows_to_key(read_rows());
//     prevCol = currCol;
//     if (c == 'A') {
//       currCol -= 1;
//       if (currCol <= 0) currCol = 1;
//     }
//     else if (c == 'B') {
//       currCol += 1;
//       if (currCol >= 4) currCol = 4;
//     }
//   }
//   // char dispChar = disp[col];
//   // show_char(col,dispChar);
//   if (prevCol != currCol) {
//     if (prevCol == 4) drawfillrectME(0, 0, 59, 39, 0xffff);
//     else drawfillrectME(60*(4-prevCol) + 1, 0, 60*(4-prevCol) - 1 + 60, 39, 0xffff);
//     if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
//     else drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
//   }
//   col++;
//   if (col > 7) col = 0;
//   drive_column(col);
// }

// void init_exti() {
//   RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; 
//   SYSCFG->EXTICR[0] =  SYSCFG_EXTICR1_EXTI0_PA | SYSCFG_EXTICR1_EXTI1_PA;
//   // SYSCFG->EXTICR[1] = SYSCFG_EXTICR2_EXTI4_PB;
//   EXTI->FTSR |=  EXTI_FTSR_TR0 |EXTI_FTSR_TR1;
//   EXTI->IMR |= EXTI_IMR_IM0 | EXTI_IMR_IM1;
//   NVIC_EnableIRQ(EXTI0_1_IRQn); 
//   // NVIC_EnableIRQ(EXTI2_3_IRQn);
//   // NVIC_EnableIRQ(EXTI4_15_IRQn);
// }

// void EXTI0_1_IRQHandler() {
//   EXTI->PR = EXTI_PR_PR0 | EXTI_PR_PR1;
  
//   // move the cursor: 
//   // update prevCol and currCol
//   prevCol = currCol;
//   if (readpin(0)) { // if left
//     currCol--;
//     if (currCol <= 0) { currCol = 1; }
//   }
//   else if (readpin(1)) { // else if right
//     currCol++;
//     if (currCol >= 4) { currCol = 4; }
//   }
//   if (prevCol != currCol) {
//     if (prevCol == 4) drawfillrectME(0, 0, 59, 39, 0xffff);
//     else drawfillrectME(60*(4-prevCol) + 1, 0, 60*(4-prevCol) - 1 + 60, 39, 0xffff);
//     if (currCol == 4) drawfillrectME(0, 0,59, 39, 0x0000);
//     else drawfillrectME(60*(4-currCol) + 1, 0, 60*(4-currCol) - 1 + 60, 39, 0x0000);
//   }
//   col++;
//   if (col > 7) col = 0;
//   drive_column(col);
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

//  // move box for creating mult boxes 
// void move_box(int col, int * y_arr);
// void move_box(int col, int* y_arr) {
//     // determine what x to check based on col 
//     int x = col*60 - 1; // x is the x val of the left side of a box
//     for (int y = 320; y >= 0; y--) {
//         // check color at (x,y) using y array 
//         if (y_arr[y]) { // if 1 = black
//             // move box 
//             drawfillrectME(x-58,y-51,x,y-1,0x0000); 
//             drawlineME(x-58,y,x,y,0xffff);
//             nano_wait(5000000);
//             y_arr[y] = 0; // set y = 0 in arr
//             y -= 50; // jump y by box height
//             y_arr[y] = 1; // set this y = 1 in arr 
//         }
//     }
// }