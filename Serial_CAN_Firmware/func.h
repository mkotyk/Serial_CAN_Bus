#ifndef __FUNC_H__
#define __FUNC_H__

#include <stdint.h>

// EEPROM
#define EEPADDR_SET     (0)
#define EEPADDR_SERIAL  (1)
#define SERIAL_DIVISOR  1200
#define EEPADDR_CANRATE (2)

#define EEPADDR_MASK0   (10)
#define EEPADDR_MASK1   (20)

#define EEPADDR_FILT0   (30)
#define EEPADDR_FILT1   (40)
#define EEPADDR_FILT2   (50)
#define EEPADDR_FILT3   (60)
#define EEPADDR_FILT4   (70)
#define EEPADDR_FILT5   (80)

#define EEPADDR_SPACING 10


const uint8_t LED_INDICATOR = 3;
const uint8_t CS_PIN = 9;


class HardwareSerial;
class MCP_CAN;

enum Mode {
  SETTINGS_MODE,
  DATA_MODE
};


struct ContextStruct {
    Mode mode;
    unsigned char *head;
    unsigned char buffer[30];
    HardwareSerial *serial;
    MCP_CAN* can;
    void (*send_can)(struct ContextStruct* ctx);
    void (*recv_can)(struct ContextStruct* ctx);
};

typedef ContextStruct Context;

#define BUF_LEN(ctx) (ctx->head - ctx->buffer)
#define NELEM(x) (sizeof(x)/sizeof(x[0]))

enum Result {
  OK = 0,
  ERROR
};


// EEPROM
extern void initEEPROM();
extern unsigned long readWord(int addr);
extern void writeWord(int addr, unsigned long dta);

// Forward declarations for commands
extern void cmd_get_masks(Context *ctx);
extern void cmd_get_filters(Context *ctx);
extern void cmd_get_can_rate(Context *ctx);
extern void cmd_get_serial_rate(Context *ctx);
static Result cmd_set_can_rate(Context *ctx, unsigned char *args);
static Result cmd_set_filters(Context *ctx, unsigned char *args);
static Result cmd_set_masks(Context *ctx, unsigned char *args);
static Result cmd_set_serial_rate(Context *ctx, unsigned char *args);

extern void handle_serial(Context* ctx);
extern void binary_send_can(Context* ctx);
extern void binary_recv_can(Context* ctx);
extern void hex_send_can(Context* ctx);
extern void hex_recv_can(Context* ctx);

extern uint32_t read_word(void *p, int (*read_func)(void *p, size_t idx));
extern void write_word(uint32_t data, void *p, void (*write_func)(uint8_t v, void *p, size_t index));
extern int eeprom_read(void *p, size_t idx);
extern void eeprom_write(uint8_t value, void *p, size_t idx);
extern void serial_write(uint8_t value, void *p, size_t idx);

#endif /* __FUNC_H__ */
