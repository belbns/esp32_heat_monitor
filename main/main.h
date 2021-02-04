#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdint.h>
#include <time.h>

#ifdef CONFIG_SDCARD_HSPI_HOST  // HSPI_HOST (SPI2)
#define SPI_DMA_CHAN  1

#define PIN_NUM_MISO  2
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   13
#else                           // VSPI_HOST (SPI3)
#define SPI_DMA_CHAN  2

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS    5
#endif

#define MOUNT_POINT  "/sdcard"
#define WWW_DIR      "/sdcard/WWW"
#define INI_FILE     "/sdcard/INI/ble_init.txt"
#define INI_BAK_FILE "/sdcard/INI/ble_init.bak"
#define INDEX_FILE   "/index.htm"
#define FPATH_LEN   64
#define SCRATCH_BUFSIZE (10240)
// ble scan pauses, sec
#define SCAN_LONG_PAUSE     30
#define SCAN_SHORT_PAUSE    15
enum ble_cmd {
    SET_MODE = 0,
    SET_PERIOD,
    SET_NIGHT,
    SET_DIFF,
    SET_SCLK,
    SET_LOCK,
    SET_TIME
};

typedef struct {
        uint8_t hour;
        uint8_t min;
        uint8_t sec;
} tech_time;

typedef struct {
        uint8_t mode_set;
        uint8_t mode_rec;
        uint8_t delta_on_set;
        uint8_t delta_on_rec;
        uint8_t delta_off_set;
        uint8_t delta_off_rec;
        tech_time night_begin_set;
        tech_time night_begin_rec;
        tech_time night_end_set;
        tech_time night_end_rec;
        tech_time period_set;
        tech_time period_rec;
        tech_time ctrl_time;
        tech_time last_start;
        uint16_t sclk_set;
        uint16_t sclk_rec;
        uint8_t lock_set;
        uint8_t lock_rec;
        uint8_t state;
        uint8_t night;
        uint8_t pump_on;
        float t_main;
        float t_aux;
        float t_air;
        uint8_t err_main;
        uint8_t err_aux;
        uint8_t err_air;
        float pressure;
        float press_bar;
        uint16_t vref_cal;
        uint16_t vref_data;
        uint8_t rec_flags;
        uint8_t updated;
        uint8_t tm_updated;
        uint8_t upd_cnt;
} pump_struct;

typedef struct _command_item {
        uint8_t cmd;
        uint8_t param;
} command_item;

#define CMD_QUEUE_LEN	16

uint8_t put_to_cmd_queue(uint8_t pnum, uint8_t cmd, uint8_t param);
uint8_t save_params(void);
void lcd_string(char *st, uint8_t nst, uint16_t shift, uint16_t clrshift, uint16_t color);

#endif
