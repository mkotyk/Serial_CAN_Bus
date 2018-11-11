#include <Arduino.h>
#include <EEPROM.h>
#include <stdlib.h>
#include <ctype.h>
#include "mcp_can.h"
#include "func.h"

const unsigned int can_rates[19] = {0, 5, 10, 20, 25, 31, 33, 40, 50, 80, 83, 95, 100, 125, 200, 250, 500, 666, 10000};

int eeprom_read(void *p, size_t idx) {
    return EEPROM.read((size_t) p + idx);
}

void eeprom_write(uint8_t value, void *p, size_t idx) {
    EEPROM.write((size_t) p + idx, value);
}

void serial_write(uint8_t value, void *p, size_t idx) {
    ((Context *)p)->serial->write(value);
}

uint32_t read_word(void *p, int (*read_func)(void *p, size_t idx)) {
    uint32_t result;
    for(int index = 0, result = 0; index < sizeof(result); index++)
        result = result << 8 | (read_func(p, index) & 0xFF);
    return result;
}

void write_word(uint32_t data, void *p, void (*write_func)(uint8_t v, void *p, size_t index)) {
    for(int index = 0; index < sizeof(data); index++)
        write_func((data >> (sizeof(data) - (index + 1)) * 8) & 0xFF, p, index);
}

static int asc_dec(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    return -1;
}

static int asc_hex(char c) {
    c = toupper(c);
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static size_t hex_read(void *p, size_t t, unsigned char *ptr) {
    size_t idx = 0;
    int c;
    for(idx = 0; idx < t; idx++) {
        c = asc_hex(ptr[idx * 2]) << 4 | asc_hex(ptr[idx * 2 + 1]);
        if (c < 0) break;
        ((uint8_t *)p)[idx] = c;
    }
    return idx;
}

static void print_formatted(Context* ctx, const char *header, uint8_t idx, uint8_t frame_type, uint32_t data) {
    ctx->serial->print(header);
    ctx->serial->print(idx);
    ctx->serial->print(" - ");
    ctx->serial->print(frame_type);
    ctx->serial->print(": 0x");
    ctx->serial->println(data, HEX);
}

static void print_mask(Context* ctx, uint8_t mask_idx, uint8_t frame_type, uint32_t mask) {
    print_formatted(ctx, "Mask", mask_idx, frame_type, mask);
}

static void print_filter(Context* ctx, uint8_t filter_idx, uint8_t frame_type, uint32_t filter) {
    print_formatted(ctx, "Filt", filter_idx, frame_type, filter);
}


/** Commands **/
void cmd_get_masks(Context *ctx) {
    print_mask(ctx, 0, eeprom_read(EEPADDR_MASK0, 0), read_word(EEPADDR_MASK0 + 1, eeprom_read));
    print_mask(ctx, 0, eeprom_read(EEPADDR_MASK1, 0), read_word(EEPADDR_MASK1 + 1, eeprom_read));
}

void cmd_get_filters(Context *ctx) {
    for(int f = 0; f < 6; f++) {
        print_filter(ctx, f, eeprom_read(EEPADDR_FILT0, (f * EEPADDR_SPACING)),
                read_word(EEPADDR_FILT0 + (f * EEPADDR_SPACING) + 1, eeprom_read));
    }
}

void cmd_get_can_rate(Context *ctx) {
    ctx->serial->print("Rate: ");
    ctx->serial->println(can_rates[eeprom_read(EEPADDR_CANRATE, 0)]);
}

void cmd_get_serial_rate(Context *ctx) {
    ctx->serial->println(eeprom_read(EEPADDR_SERIAL, 0) * SERIAL_DIVISOR);
}

static Result cmd_set_can_rate(Context *ctx, unsigned char *args) {
    char *endptr = NULL;
    int rate = strtol((const char*) args, &endptr, 10);
    if (endptr == NULL || rate < 0 || rate >= NELEM(can_rates)) {
        return ERROR;
    }

    if(CAN_OK != ctx->can->begin(rate)) {
        return ERROR;
    }

    eeprom_write(rate, EEPADDR_CANRATE, 0);
    return OK;
}

// [0-5][0-1][XX]{4}
static Result cmd_set_filters(Context *ctx, unsigned char *args) {
    uint8_t filter_idx = asc_dec(args[0]);
    if (filter_idx > 5) {
        return ERROR;
    }

    uint8_t frame_type = asc_dec(args[1]);

    uint32_t filter = 0;
    if (hex_read(&filter, sizeof(filter), &args[2]) != sizeof(filter)) {
        return ERROR;
    }

    eeprom_write(frame_type, EEPADDR_FILT0, 10 * filter_idx);
    write_word(filter, EEPADDR_FILT0 + (10 * filter_idx) + 1, eeprom_write);

    ctx->can->init_Filt(filter_idx, eeprom_read(EEPADDR_FILT0, EEPADDR_SPACING * filter_idx),
            read_word(EEPADDR_FILT0 + (EEPADDR_SPACING * filter_idx) + 1, eeprom_read));
    print_filter(ctx, filter_idx, frame_type, filter);
    return OK;
}

// set mask AT+M=[0-1]{1}[0-1]{1}[0-9A-F]{8}
static Result cmd_set_masks(Context *ctx, unsigned char *args) {
    uint8_t mask_index = asc_dec(args[0]);
    if (mask_index > 1) {
        return ERROR;
    }

    uint8_t frame_type = asc_dec(args[1]);
    uint32_t mask = 0;
    if (hex_read(&mask, sizeof(mask), &args[2]) != sizeof(mask)) {
        return ERROR;
    }

    eeprom_write(frame_type, EEPADDR_MASK0, 10 * mask_index);
    write_word(mask, EEPADDR_MASK0 + (10 * mask_index) + 1, eeprom_write);

    print_mask(ctx, mask_index, frame_type, mask);
    ctx->can->init_Mask(mask_index, eeprom_read(EEPADDR_MASK0, 10 * mask_index),
            read_word(EEPADDR_MASK0 + (mask_index * 10) + 1, eeprom_read));
    return OK;
}

// set serial speed AT+S=[0-9]{4-6} - 1200-115200 baud
static Result cmd_set_serial_rate(Context *ctx, unsigned char *args) {
    char *endptr = NULL;
    uint32_t rate = strtol((const char *)args, &endptr, 10);
    if (endptr == NULL || rate % 1200 != 0  || rate > 115200) {
        return ERROR;
    }

    ctx->serial->begin(rate);
    eeprom_write(rate / SERIAL_DIVISOR, EEPADDR_SERIAL, 0);
    return OK;
}

/** End of Commands **/

static bool is_line_complete(Context *ctx) {
    return *ctx->head == '\n';
}

static bool is_offline_seq(Context *ctx) {
    return ((*ctx->head == '+') && BUF_LEN(ctx) > 2 &&
            (*ctx->head - 1 == '+') && (*ctx->head - 2 == '+'));
}

static void process_command(Context *ctx) {
    bool get_cmd;
    unsigned char *args = NULL;

    if (BUF_LEN(ctx) >= 2) {
        digitalWrite(LED_INDICATOR, LOW);

        Result result = OK;

        if (ctx->mode == DATA_MODE && ctx->buffer[0] == '+' && ctx->buffer[1] == '+') {
            ctx->mode = SETTINGS_MODE;
        } else if (ctx->mode == SETTINGS_MODE && toupper(ctx->buffer[0]) == 'A' && toupper(ctx->buffer[1] == 'T')) {
            switch(toupper(ctx->buffer[2])) {
                case 'I': // Information
                    ctx->serial->println("Serial CAN V2.0");
                    break;
                case 'O':  // Online.  Don't print OK/ERROR
                    ctx->mode = DATA_MODE;
                    return;
                case '+':
                    // Extended commands
                    switch(toupper(ctx->buffer[2])) {
                        get_cmd = BUF_LEN(ctx) < 4 || ctx->buffer[3] == '?';
                        if (!get_cmd && BUF_LEN(ctx) > 4) {
                            args = &ctx->buffer[4];
                        }
                        case 'B':
                        ctx->send_can = &binary_send_can;
                        ctx->recv_can = &binary_recv_can;
                        ctx->serial->println("Binary mode selected.");
                        break;
                        case 'C': // CAN Bus rate
                        if (get_cmd) {
                            cmd_get_can_rate(ctx);
                        } else {
                            result = cmd_set_can_rate(ctx, args);
                        }
                        break;
                        case 'F': // Filter
                        if (get_cmd) {
                            cmd_get_filters(ctx);
                        } else {
                            result = cmd_set_filters(ctx, args);
                        }
                        break;
                        case 'H':
                        ctx->send_can = &hex_send_can;
                        ctx->recv_can = &hex_recv_can;
                        ctx->serial->println("Hex mode selected.");
                        break;
                        case 'M': // Mask
                        if (get_cmd) {
                            cmd_get_masks(ctx);
                        } else {
                            result = cmd_set_masks(ctx, args);
                        }
                        break;
                        case 'S': // Serial Baud rate
                        if (get_cmd) {
                            cmd_get_serial_rate(ctx);
                        } else {
                            result = cmd_set_serial_rate(ctx, args);
                        }
                        break;
                        default:
                        result = ERROR;
                        break;
                    }
                    break;
                default:
                    result = ERROR;
                    break;
            }
        } else if (ctx->mode == DATA_MODE) {
            ctx->send_can(ctx);
        }

        if (result == OK) {
            ctx->serial->println("OK");
            digitalWrite(LED_INDICATOR, HIGH);
        } else {
            ctx->serial->println("ERROR");
        }
    }
}

void handle_serial(Context* ctx)
{
    while(ctx->serial->available()) {
        if(BUF_LEN(ctx) < sizeof(ctx->buffer)) {
            *ctx->head= ctx->serial->read();
            if (is_line_complete(ctx) || is_offline_seq(ctx)) {
                *ctx->head = '\0';
                process_command(ctx);
                ctx->head = ctx->buffer;
            } else {
                ctx->head++;
            }
        }
    }
}

void binary_send_can(Context* ctx)
{
    if(DATA_MODE != ctx->mode) return;
    if(14 != BUF_LEN(ctx)) return;

    unsigned long id  = 0;
    for(int i=0; i<4; i++)
    {
        id <<= 8;
        id += ctx->buffer[i];
    }

    if(ctx->buffer[4] > 1 || ctx->buffer[5] > 1) return;
    ctx->can->sendMsgBuf(id, ctx->buffer[4], ctx->buffer[5], 8, &ctx->buffer[6]);
}

void binary_recv_can(Context* ctx)
{
    if(DATA_MODE != ctx->mode)return;

    unsigned char len = 0;
    unsigned char buf[8];

    if(CAN_MSGAVAIL == ctx->can->checkReceive())              // check if data coming
    {
        ctx->can->readMsgBuf(&len, buf);                      // read data,  len: data length, buf: data buf
        unsigned long id = ctx->can->getCanId();
        write_word(id, ctx, serial_write);
        for(int i = 0; i < NELEM(buf); i++) {
            ctx->serial->write(buf[i]);
        }
    }
}


void hex_send_can(Context* ctx)
{
    if(DATA_MODE != ctx->mode) return;
    int bytes_read = 0;
    unsigned long id;
    uint8_t a;
    uint8_t b;
    uint8_t data[8];
    uint8_t offset = 0;

    bytes_read = hex_read(&id, sizeof(id), &ctx->buffer[offset]);
    if(bytes_read != sizeof(id)) return;
    offset += bytes_read;

    bytes_read += hex_read(&a, sizeof(a), &ctx->buffer[offset]);
    if(bytes_read != sizeof(id)) return;
    offset += bytes_read;

    bytes_read += hex_read(&b, sizeof(b), &ctx->buffer[offset]);
    if(bytes_read != sizeof(id)) return;
    offset += bytes_read;

    bytes_read += hex_read(&data, sizeof(data), &ctx->buffer[offset]);
    if(bytes_read != sizeof(id)) return;
    offset += bytes_read;

    ctx->can->sendMsgBuf(id, a, b, sizeof(data), data);
}

void hex_recv_can(Context* ctx)
{
    if(DATA_MODE != ctx->mode)return;

    unsigned char len = 0;
    unsigned char buf[8];

    if(CAN_MSGAVAIL == ctx->can->checkReceive())              // check if data coming
    {
        ctx->can->readMsgBuf(&len, buf);                      // read data,  len: data length, buf: data buf
        unsigned long id = ctx->can->getCanId();
        ctx->serial->print(id, HEX);
        for(int i = 0; i < NELEM(buf); i++) {
            ctx->serial->print(buf[i], HEX);
        }
        ctx->serial->println();
    }
}
