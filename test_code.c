/*****************************************************
Based on code of Gabriel from www.Electronics-Base.com for ATmega8 (3/19/2012).
Thanks Gabriel.
*****************************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "test_code.h"

#define sensor_th_data_PIN_NUMBER 1
#define sensor_th_data_NAME C
#define sensor_th_clk_PIN_NUMBER 1
#define sensor_th_clk_NAME C

#define sensor_th_data_PORT def_port_reg(sensor_th_data_NAME)
#define sensor_th_data_DDR def_ddr_reg(sensor_th_data_NAME)
#define sensor_th_data_PIN def_pin_reg(sensor_th_data_NAME)
#define sensor_th_clk_PORT def_port_reg(sensor_th_clk_NAME)
#define sensor_th_clk_DDR def_ddr_reg(sensor_th_clk_NAME)
#define sensor_th_clk_PIN def_pin_reg(sensor_th_clk_NAME)

#define SH7x_CMD_MASK 0x1f
#define SH7x_ADD_MASK 0xe0
#define SH7x_CMD_MEASURE_TEMPERATURE 0x03
#define SH7x_CMD_MEASURE_HUMIDITY 0x05
#define SH7x_CMD_READ_STATUS 0x07
#define SH7x_CMD_WRITE_STATUS 0x06
#define SH7x_CMD_SOFT_RESET 0x1e

uint8_t command;
bool new_command;

inline void SH7x_data_as_output (void){
    SetBit(sensor_th_data_DDR, sensor_th_data_PIN_NUMBER);
}
inline void SH7x_data_as_input (void){
    ClearBit(sensor_th_data_DDR, sensor_th_data_PIN_NUMBER);
}
inline void SH7x_clk_as_output (void){
    SetBit(sensor_th_clk_DDR, sensor_th_clk_PIN_NUMBER);
}

void SH7x_setup(void) {
    SH7x_data_as_output();
    SH7x_clk_as_output();
}

void SH7x_read_byte(uint8_t* data, bool ack){
    uint8_t i,val=0;
    SH7x_data_as_input();
    for (i=(1<<7); i>0; i/=2){ //shift bit for masking
        SetBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER); //clk for SENSI-BUS
        _delay_us(2);
        if (IsBitSet(sensor_th_data_PIN, sensor_th_data_PIN_NUMBER)){
            val |= (1 << i);
        }
        _delay_us(2);
        ClearBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER);
    }
    SH7x_data_as_output();
    if(ack) ClearBit(sensor_th_data_PORT, sensor_th_data_PIN_NUMBER); //in case of "ack==1" pull down DATA-Line
    SetBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER); //clk #9 for ack
    _delay_us(5); //pulswith approx. 5 us
    ClearBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER);
    SetBit(sensor_th_data_PORT, sensor_th_data_PIN_NUMBER); //release DATA-line  //ADD BY LUBING
    *data = val;
}

retval_t SH7x_write_byte(uint8_t data){
    retval_t rv = RV_SUCCESS;
    uint8_t i;
    SH7x_data_as_output();
    for(i=(1<<7); i>0; i/=2){ //shift bit for masking
        if(i & data){
            SetBit(sensor_th_data_PORT, sensor_th_data_PIN_NUMBER);
        } else {
            ClearBit(sensor_th_data_PORT, sensor_th_data_PIN_NUMBER);
        }

        SetBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER); //clk for SENSI-BUS
        _delay_us(5); //pulswith approx. 5 us
        ClearBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER);
    }

    SetBit(sensor_th_data_PORT, sensor_th_data_PIN_NUMBER); //release dataline
    SH7x_data_as_input();
    SetBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER); //clk #9 for ack
    _delay_us(2);
    if(IsBitSet(sensor_th_data_PIN, sensor_th_data_PIN_NUMBER)) rv = RV_ERROR; //check ack (DATA will be pulled down by SHT11)
    _delay_us(2);
    ClearBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER);
    return rv;
}

//----------------------------------------------------------------------------------
// generates a transmission start
//       _____         ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______
//----------------------------------------------------------------------------------
void SH7x_start_transmission(void)
{
    SH7x_data_as_output();
    SetBit(sensor_th_data_PORT, sensor_th_data_PIN_NUMBER);
    ClearBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER);
    _delay_us(2);
    SetBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER);
    _delay_us(2);
    ClearBit(sensor_th_data_PORT, sensor_th_data_PIN_NUMBER);
    _delay_us(2);
    ClearBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER);
    _delay_us(5);
    SetBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER);
    _delay_us(2);
    SetBit(sensor_th_data_PORT, sensor_th_data_PIN_NUMBER);
    _delay_us(2);
    ClearBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER);
    SH7x_data_as_input();
}

//----------------------------------------------------------------------------------
// communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
//       _____________________________________________________         ________
// DATA:                                                      |_______|
//          _    _    _    _    _    _    _    _    _        ___     ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
//----------------------------------------------------------------------------------
void SH7x_hard_reset(void)
{
    uint8_t i;
    SH7x_data_as_output();
    SetBit(sensor_th_data_PORT, sensor_th_data_PIN_NUMBER);
    ClearBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER);

    for(i=0;i<9;i++){ //9 SCK cycles
        SetBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER);
        _delay_us(1);
        ClearBit(sensor_th_clk_PORT, sensor_th_clk_PIN_NUMBER);
        _delay_us(1);
    }
    SH7x_start_transmission();
    SH7x_data_as_input();
}

retval_t SH7x_checksum(uint8_t* data, uint8_t checksum){
    return RV_SUCCESS;
}

retval_t SH7x_measure_relative_humidity(uint16_t* hum){
    retval_t rv;
    uint8_t aux[2];
    uint8_t checksum;

    SH7x_start_transmission();
    rv = SH7x_write_byte(SH7x_CMD_MEASURE_HUMIDITY);
    if(RV_SUCCESS != rv){
        SH7x_hard_reset();
        return rv;
    }
    SH7x_data_as_input();
    // TODO: Add timeout here. This can hang out if chip never answer 
    while (IsBitSet(sensor_th_data_PIN, sensor_th_data_PIN_NUMBER)){}//wait until sensor has finished the measurement

    SH7x_read_byte(&aux[0], true);
    SH7x_read_byte(&aux[1], true);
    SH7x_read_byte(&checksum, false);
    rv = SH7x_checksum(aux, checksum);
    if(RV_SUCCESS == rv){
        *hum = (aux[0]<<8) | aux[1];
    }

    return rv;
}

retval_t SH7x_measure_temperature(uint16_t* temp){
    retval_t rv;
    uint8_t aux[2];
    uint8_t checksum;

    SH7x_start_transmission();
    rv = SH7x_write_byte(SH7x_CMD_MEASURE_TEMPERATURE);
    if(RV_SUCCESS != rv){
        SH7x_hard_reset();
        return rv;
    }
    SH7x_data_as_input();
    // TODO: Add timeout here. This can hang out if chip never answer 
    while (IsBitSet(sensor_th_data_PIN, sensor_th_data_PIN_NUMBER)){}//wait until sensor has finished the measurement

    SH7x_read_byte(&aux[0], true);
    SH7x_read_byte(&aux[1], true);
    SH7x_read_byte(&checksum, false);
    rv = SH7x_checksum(aux, checksum);
    if(RV_SUCCESS == rv){
        *temp = (aux[0]<<8) | aux[1];
    }

    return rv;
}

retval_t SH7x_read_status(uint8_t* status){
    retval_t rv;
    uint8_t checksum;
    SH7x_start_transmission();
    rv = SH7x_write_byte(SH7x_CMD_READ_STATUS);
    if(RV_SUCCESS != rv){
        SH7x_hard_reset();
        return rv;
    }
    SH7x_read_byte(status, true);
    SH7x_read_byte(&checksum, false);
    return SH7x_checksum(status, checksum);
}

retval_t SH7x_write_status(uint8_t status){
    unsigned char error=0;
    retval_t rv;
    SH7x_start_transmission();
    rv = SH7x_write_byte(SH7x_CMD_WRITE_STATUS);
    if(RV_SUCCESS != rv){
        SH7x_hard_reset();
        return rv;
    }
    return SH7x_write_byte(status);
}

retval_t SH7x_soft_reset(uint8_t status){
    SH7x_hard_reset();
    return SH7x_write_byte(SH7x_CMD_SOFT_RESET);
}

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

void USART_Init( unsigned int ubrr)
{
    /*Set baud rate */
    UBRRH = (uint8_t) (ubrr>>8);
    UBRRL = (uint8_t) ubrr;
    UCSRB = (1<<TXCIE)|(1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
    /* Set frame format: 8data, 1stop bit */
    UCSRC = (1<<URSEL)|(3<<UCSZ0);
}

void USART_Transmit( unsigned char data )
{
    /* Wait for empty transmit buffer */
    while ( !( UCSRA & (1<<UDRE)) )
    ;
    /* Put data into buffer, sends the data */
    UDR = data;
}

int main(void){
    uint8_t aux8;
    uint16_t aux16;
    retval_t rv;

    USART_Init(MYUBRR);
    SH7x_setup();

    for(;;){
        _delay_ms(100);
        if(new_command){
            new_command = false;
            switch(command){
                case '1':
                    rv = SH7x_read_status(&aux8);
                    USART_Transmit(rv);
                    if(RV_SUCCESS == rv){
                        USART_Transmit(aux8);
                    }
                    break;
                case '2':
                    SH7x_measure_temperature(&aux16);
                    USART_Transmit(rv);
                    if(RV_SUCCESS == rv){
                        USART_Transmit((uint8_t) aux16);
                        USART_Transmit((uint8_t) (aux16 >> 8));
                    }
                    break;
                case '3':
                    SH7x_measure_relative_humidity(&aux16);
                    USART_Transmit(rv);
                    if(RV_SUCCESS == rv){
                        USART_Transmit((uint8_t) aux16);
                        USART_Transmit((uint8_t) (aux16 >> 8));
                    }
                    break;
                default:
                    USART_Transmit(RV_NOTDEF);
            }
        }
    }
    return 0;
}

ISR(USART_RXC_vect) {
	uint8_t c;

    if(new_command != true){
    	command = UDR;
        new_command = true;
    }
}
