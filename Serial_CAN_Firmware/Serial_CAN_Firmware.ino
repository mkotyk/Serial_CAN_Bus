/*************************************************************************************************
  Firmware of Serial can bus module

Nov 10th, 2018 - Mark Kotyk - Refactored original.
 - Reduced global variables by adding context structure
 - Eliminated input timeout
 - Added Hex mode for send/recv can messages
 - Changed AT+Q to be ATO as per Hayse standard
 - Various cleanups (DRY, single responsibility principle, consistent naming)
 - Combined and eliminated some files

***************************************************************************************************/
#include <SPI.h>
#include <EEPROM.h>
#include "mcp_can.h"
#include "func.h"

MCP_CAN CAN(CS_PIN);
Context context;

void sys_init(Context *ctx)
{
    pinMode(LED_INDICATOR, OUTPUT);

    if(eeprom_read(EEPADDR_SET, 0) != 0x55) {
        // Initialize EEPROM
        eeprom_write(0x55, EEPADDR_SET, 0);
        eeprom_write(115200L / SERIAL_DIVISOR, EEPADDR_SERIAL, 0);
        eeprom_write(16, EEPADDR_CANRATE, 0); // 500Kbps

        for(int i=10; i<90; i++) {
            eeprom_write(0, i, 0);
        }
    }

    ctx->serial->begin((long) eeprom_read(EEPADDR_SERIAL, 0) * SERIAL_DIVISOR);

    while (CAN_OK != ctx->can->begin(eeprom_read(EEPADDR_CANRATE, 0)))    // init can bus : baudrate = 500k
    {
        ctx->serial->println("CAN BUS Shield init fail");
        ctx->serial->println(" Init CAN BUS Shield again");
        delay(100);
        digitalWrite(LED_INDICATOR, !digitalRead(LED_INDICATOR));
    }

    /*
     * set mask, set both the mask to 0x3ff
     */
    ctx->can->init_Mask(0, eeprom_read(EEPADDR_MASK0, 0),
                        read_word(EEPADDR_MASK0 + 1, eeprom_read));
    ctx->can->init_Mask(1, eeprom_read(EEPADDR_MASK1, 0),
                        read_word(EEPADDR_MASK1 + 1, eeprom_read));

    /*
     * set filter, we can receive id from 0x04 ~ 0x09
     */
    ctx->can->init_Filt(0, eeprom_read(EEPADDR_FILT0, 0),
                        read_word(EEPADDR_FILT0 + 1, eeprom_read));
    ctx->can->init_Filt(1, eeprom_read(EEPADDR_FILT1, 0),
                        read_word(EEPADDR_FILT1 + 1, eeprom_read));
    ctx->can->init_Filt(2, eeprom_read(EEPADDR_FILT2, 0),
                        read_word(EEPADDR_FILT2 + 1, eeprom_read));
    ctx->can->init_Filt(3, eeprom_read(EEPADDR_FILT3, 0),
                        read_word(EEPADDR_FILT3 + 1, eeprom_read));
    ctx->can->init_Filt(4, eeprom_read(EEPADDR_FILT4, 0),
                        read_word(EEPADDR_FILT4 + 1, eeprom_read));
    ctx->can->init_Filt(5, eeprom_read(EEPADDR_FILT5, 0),
                        read_word(EEPADDR_FILT5 + 1, eeprom_read));

    cmd_get_serial_rate(ctx);
    cmd_get_can_rate(ctx);
    cmd_get_masks(ctx);
    cmd_get_filters(ctx);

    ctx->serial->println("CAN INIT OK");
    digitalWrite(LED_INDICATOR, HIGH);
}


void setup()
{
    context.head = context.buffer;
    context.serial = &Serial;
    context.can = &CAN;
    context.send_can = &hex_send_can;
    context.recv_can = &hex_recv_can;

    sys_init(&context);
}

void loop()
{
    handle_serial(&context);
    context.recv_can(&context);
}
