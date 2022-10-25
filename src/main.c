#include <stdio.h>
#include "freertos\FreeRTOS.h"
#include "freertos\task.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "driver/spi_master.h"

#define CLK_PIN 16 // GPIO16 como clock
#define MOSI_PIN 4 // GPIO4 como Data

// Head values
#define HEAD_W_TO_INSTRUC_REGISTER 0xF8; // 1111-1000b
#define HEAD_W_TO_DATA_REGISTER 0xFA;    // 1111-1010b

// ST7920 instructions
#define LCD_CLEAR_SCREEN 0x01    // Clear screen
#define LCD_ADDRESS_RESET 0x02   // The address counter is reset
#define LCD_BASIC_FUNCTION 0x30  // Basic instruction set
#define LCD_EXTEND_FUNCTION 0x34 // Extended instruction set

spi_device_handle_t spi2;

static void spi_init()
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = MOSI_PIN,
        .sclk_io_num = CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 10, // tengo duda sobre este valor si deberia ser 3 o esta bien en 10. originalmente lo ponian en 32.
    };
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    ESP_ERROR_CHECK(ret);
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, // 1 MHz
        .mode = 0,                 // SPI mode 0
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi2));
};

static void write_reg(uint8_t head, uint8_t value)
{
    uint8_t valuehalf1, valuehalf2;

    valuehalf1 = value & 0xf0;        // operacion "and" entre "value" y el valor 11110000b para obtener el 1er byte de datos a enviar
    valuehalf2 = (value << 4) & 0xf0; // operacion de desplazar 4 bit a la izquierda el valor contenido en value y luego hacer
                                      // una operacion "and" entre dicho resultado y el valor 1111000b para obtener el 2do byte de datos a enviar

    uint8_t tx_data[3] = {head, valuehalf1, valuehalf2};
    spi_transaction_t t = {
        .tx_buffer = tx_data,
        .length = 3 * 8};
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t));
    vTaskDelay(1); // retardo que se espera hasta completar la instruccion. puede que sea exagerado
}

static void ST7920_init_sequence()
{
    int8_t h = HEAD_W_TO_INSTRUC_REGISTER;

    write_reg(h, LCD_BASIC_FUNCTION); // Function set
    write_reg(h, LCD_CLEAR_SCREEN);   // Display clear
    write_reg(h, 0x06);               // Entry mode set  (cursor moves right, address counter (AC) is increased by 1)
    write_reg(h, 0x0C);               // Display control (display ON, cursor OFF, cursor position blink OFF).
}

void app_main()
{
    int8_t Head1 = 0;

    vTaskDelay(1000);

    spi_init();             // se inicializa el bus SPI
    ST7920_init_sequence(); // se realiza la configuracion inicial del LCD.

    // escribir caracteres en la pantalla de tamaño 8X16

    Head1 = HEAD_W_TO_DATA_REGISTER;

    write_reg(Head1, 'H');
    write_reg(Head1, 'O');
    write_reg(Head1, 'L');
    write_reg(Head1, 'A');

    Head1 = HEAD_W_TO_INSTRUC_REGISTER;
    write_reg(Head1, 0x88); // se coloca el cursor al comienzo de la 3era linea del display
                            //(en modo texto muestra 4 lienas en total de texto con 16 caracteres por linea)

    vTaskDelay(100); // retardo entre lineas

    Head1 = HEAD_W_TO_DATA_REGISTER;
    write_reg(Head1, 'L');
    write_reg(Head1, 'u');
    write_reg(Head1, 'c');
    write_reg(Head1, 'a');
    write_reg(Head1, 's');
}