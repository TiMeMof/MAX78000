#include "combine.h"
#include "TFT\fthr\SansSerif16x16.c"

int font_1 = (int)&SansSerif16x16[0];
int font_2 = (int)&SansSerif16x16[0];

uint8_t data565[IMAGE_SIZE_X * 2];
uint32_t input_0[IMAGE_SIZE_X * IMAGE_SIZE_Y]; // buffer for camera image

char buff[TFT_BUFF_SIZE];
int ret = 0;
int dma_channel;

void TFT_Print(char *str, int x, int y, int font, int length)
{
    // fonts id
    text_t text;
    text.data = str;
    text.len = length;
    MXC_TFT_PrintFont(x, y, font, &text, NULL);
}
int g_dma_channel_tft = 1;
static uint8_t *rx_data = NULL;

void setup_dma_tft(uint32_t *src_ptr, uint16_t byte_cnt)
{
    // TFT DMA
    while ((MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_STATUS)) {
        ;
    }

    MXC_DMA->ch[g_dma_channel_tft].status = MXC_F_DMA_STATUS_CTZ_IF; // Clear CTZ status flag
    MXC_DMA->ch[g_dma_channel_tft].dst = (uint32_t)rx_data; // Cast Pointer
    MXC_DMA->ch[g_dma_channel_tft].src = (uint32_t)src_ptr;
    MXC_DMA->ch[g_dma_channel_tft].cnt = byte_cnt;

    MXC_DMA->ch[g_dma_channel_tft].ctrl =
        ((0x1 << MXC_F_DMA_CTRL_CTZ_IE_POS) + (0x0 << MXC_F_DMA_CTRL_DIS_IE_POS) +
         (0x1 << MXC_F_DMA_CTRL_BURST_SIZE_POS) + (0x0 << MXC_F_DMA_CTRL_DSTINC_POS) +
         (0x1 << MXC_F_DMA_CTRL_DSTWD_POS) + (0x1 << MXC_F_DMA_CTRL_SRCINC_POS) +
         (0x1 << MXC_F_DMA_CTRL_SRCWD_POS) + (0x0 << MXC_F_DMA_CTRL_TO_CLKDIV_POS) +
         (0x0 << MXC_F_DMA_CTRL_TO_WAIT_POS) + (0x2F << MXC_F_DMA_CTRL_REQUEST_POS) + // SPI0 -> TFT
         (0x0 << MXC_F_DMA_CTRL_PRI_POS) + // High Priority
         (0x0 << MXC_F_DMA_CTRL_RLDEN_POS) // Disable Reload
        );

    MXC_SPI0->ctrl0 &= ~(MXC_F_SPI_CTRL0_EN);
    MXC_SETFIELD(MXC_SPI0->ctrl1, MXC_F_SPI_CTRL1_TX_NUM_CHAR,
                 (byte_cnt) << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);

    // Clear SPI master done flag
    MXC_SPI0->intfl = MXC_F_SPI_INTFL_MST_DONE;
    MXC_SETFIELD(MXC_SPI0->dma, MXC_F_SPI_DMA_TX_THD_VAL, 0x10 << MXC_F_SPI_DMA_TX_THD_VAL_POS);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_DMA_TX_EN);
    MXC_SPI0->ctrl0 |= (MXC_F_SPI_CTRL0_EN);
}

/* **************************************************************************** */
void start_tft_dma(uint32_t *src_ptr, uint16_t byte_cnt)
{
    while ((MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_STATUS)) {
        ;
    }

    if (MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_CTZ_IF) {
        MXC_DMA->ch[g_dma_channel_tft].status = MXC_F_DMA_STATUS_CTZ_IF;
    }

    MXC_DMA->ch[g_dma_channel_tft].cnt = byte_cnt;
    MXC_DMA->ch[g_dma_channel_tft].src = (uint32_t)src_ptr;

    // Enable DMA channel
    MXC_DMA->ch[g_dma_channel_tft].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
    MXC_Delay(1); // to fix artifacts in the image
    // Start DMA
    MXC_SPI0->ctrl0 |= MXC_F_SPI_CTRL0_START;
}

/* **************************************************************************** */
void tft_dma_display(int x, int y, int w, int h, uint32_t *data)
{
    // setup dma
    setup_dma_tft((uint32_t *)data, w * h * 2);

    // Send a line of captured image to TFT
    start_tft_dma((uint32_t *)data, w * h * 2);
}
//--------------------------------------------------------------//
void capture_process_camera(void)
{
    uint8_t *raw;
    uint32_t imgLen;
    uint32_t w, h;

    int cnt = 0;

    uint8_t r, g, b;
    uint16_t rgb;
    int j = 0;

    uint8_t *data = NULL;
    stream_stat_t *stat;

    camera_start_capture_image();

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);
//    printf("W:%d H:%d L:%d \n", w, h, imgLen);

#if defined(TFT_ENABLE) && defined(BOARD_FTHR_REVA)
    // Initialize FTHR TFT for DMA streaming
    MXC_TFT_Stream(TFT_X_START, TFT_Y_START, w, h);
#endif

    // Get image line by line
    for (int row = 0; row < h; row++) {
        // Wait until camera streaming buffer is full
        while ((data = get_camera_stream_buffer()) == NULL) {
            if (camera_is_image_rcv()) {
                break;
            }
        }

        //LED_Toggle(LED2);
#ifdef BOARD_EVKIT_V1
        j = IMAGE_SIZE_X * 2 - 2; // mirror on display
#else
        j = 0;
#endif
        for (int k = 0; k < 4 * w; k += 4) {
            // data format: 0x00bbggrr
            r = data[k];
            g = data[k + 1];
            b = data[k + 2];
            //skip k+3

            // change the range from [0,255] to [-128,127] and store in buffer for CNN
            input_0[cnt++] = ((b << 16) | (g << 8) | r) ^ 0x00808080;

            // convert to RGB656 for display
            rgb = ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | (b >> 3);
            data565[j] = (rgb >> 8) & 0xFF;
            data565[j + 1] = rgb & 0xFF;
#ifdef BOARD_EVKIT_V1
            j -= 2; // mirror on display
#else
            j += 2;
#endif
        }
#ifdef TFT_ENABLE

#ifdef BOARD_EVKIT_V1
        MXC_TFT_ShowImageCameraRGB565(TFT_X_START, TFT_Y_START + row, data565, w, 1);
#endif
#ifdef BOARD_FTHR_REVA
        tft_dma_display(TFT_X_START, TFT_Y_START + row, w, 1, (uint32_t *)data565);
#endif

#endif

        //LED_Toggle(LED2);
        // Release stream buffer
        release_camera_stream_buffer();
    }

    //camera_sleep(1);
    stat = get_camera_stream_statistic();

    if (stat->overflow_count > 0) {
        printf("OVERFLOW DISP = %d\n", stat->overflow_count);
        LED_On(LED2); // Turn on red LED if overflow detected
        while (1) {}
    }
}


void TFT_init(void){
	mxc_gpio_cfg_t tft_reset_pin = {MXC_GPIO0, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH};
	mxc_gpio_cfg_t tft_blen_pin = {MXC_GPIO0, MXC_GPIO_PIN_9, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH};
	MXC_TFT_Init(MXC_SPI0, 1, &tft_reset_pin, &tft_blen_pin);
	MXC_TFT_SetRotation(ROTATE_270);
	MXC_TFT_SetForeGroundColor(WHITE); // set chars to white
}

int Camera_DMA_init(void){
	Camera_Power(1);
	//DMA init
	MXC_DMA_Init();
	dma_channel = MXC_DMA_AcquireChannel();
	// Initialize camera.
	printf("Init Camera.\n");
	camera_init(CAMERA_FREQ);
	ret = camera_setup(IMAGE_SIZE_X, IMAGE_SIZE_Y, PIXFORMAT_RGB888, FIFO_THREE_BYTE, STREAMING_DMA,dma_channel);
	if (ret != STATUS_OK) {
	  printf("Error returned from setting up camera. Error %d\n", ret);
	  return -1;
	}
	camera_write_reg(0x11, 0x0);

	MXC_TFT_SetBackGroundColor(4);
	memset(buff, 32, TFT_BUFF_SIZE);

	printf("********** Press PB1(SW1) to capture an image **********\r\n");

	MXC_TFT_ClearScreen();
	MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CNN);
	memset(buff, 32, TFT_BUFF_SIZE);
	TFT_Print(buff, 55, 50, font_2, snprintf(buff, sizeof(buff), "ANALOG DEVICES"));
	TFT_Print(buff, 55, 90, font_1, snprintf(buff, sizeof(buff), "Cats-vs-Dogs Demo"));
	TFT_Print(buff, 30, 130, font_2, snprintf(buff, sizeof(buff), "PRESS PB1(SW1) TO START!"));

	return 0;
}

void myInit(void){
	TFT_init();
	Camera_DMA_init();
}
