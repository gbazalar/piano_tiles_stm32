#ifndef __HEADER_H__
#define __HEADER_H__

// struct commands_t {
//     const char *cmd;
//     void      (*fn)(int argc, char *argv[]);
// };

void lcd_initME(void);
void clearME(int value);
void drawlineME(int x1, int y1, int x2, int y2, int c);
void drawrectME(int x1, int y1, int x2, int y2, int c);
void drawfillrectME(int x1, int y1, int x2, int y2, int c);
// void internal_clock();


#endif /* __COMMANDS_H_ */