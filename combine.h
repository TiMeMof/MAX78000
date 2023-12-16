#include "camera.h"
#include "pb.h"
#include "tft_ili9341.h"
//#include "TFT\fthr\SansSerif16x16.c"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "cnn.h"

#define BOARD_FTHR_REVA
#define TFT_ENABLE

#define LED1 0
#define LED2 1

#define IMAGE_SIZE_X (64 * 2)
#define IMAGE_SIZE_Y (64 * 2)

#define TFT_X_START 100
#define TFT_Y_START 70

#define CAMERA_FREQ (5 * 1000 * 1000)
#define TFT_BUFF_SIZE 30 // TFT buffer size

//const char classes[CNN_NUM_OUTPUTS][10] = { "Cat", "Dog" };
extern int font_1,font_2,ret,dma_channel;
extern char buff[TFT_BUFF_SIZE];
extern uint8_t data565[IMAGE_SIZE_X * 2];
extern uint32_t input_0[IMAGE_SIZE_X * IMAGE_SIZE_Y];

void capture_process_camera(void);
void TFT_Print(char *str, int x, int y, int font, int length);
void TFT_init(void);
int Camera_DMA_init(void);
void myInit(void);
