/*
 *  Author: Slava
 */ 

#include "defines.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "lcd.h"
#include "hd44780.h"
//------------------------------------------------------------------------------------------------------------------
//states
#define BLOCK   						0x00
#define WAIT	  						0x01
#define WORK							0x02
#define REST							0x03

#define SETING							0x10
#define MANUAL_START					0x11
#define CONTROL							0x12

//#define ENABLE_DISABLE					0x13
#define VOLTAGE_TIMER					0x13
#define TEMPERATURE_TIMER				0x14
#define GENERATOR						0x15
#define BATTERY_VOLTAGE					0x16
#define TEMPERATURE_LOW					0x17
#define TEMPERATURE_HIGH				0x18
#define ENABLE_DISABLE					0x19

//#define ENABLE_DISABLE_SETPOINT			0x1A
#define VOLTAGE_WORKING_TIME			0x1A
#define VOLTAGE_RESTING_TIME			0x1B
#define TEMPERATURE_WORKING_TIME		0x1C
#define TEMPERATURE_RESTING_TIME		0x1D
#define GENERATOR_TYPE					0x1E
#define BATTERY_LOWER_LIMIT				0x1F
#define TEMPERATURE_LOWER_LIMIT			0x20
#define TEMPERATURE_HIGH_LIMIT			0x21
#define ENABLE_DISABLE_SETPOINT_BAT_VOLTAGE			0x22
#define ENABLE_DISABLE_SETPOINT_TEMP_LOW			0x23
#define ENABLE_DISABLE_SETPOINT_TEMP_HIGHT			0x24
#define ENABLE_DISABLE_SETPOINT_STORED				0x25

#define MANUALLY_STARTED				0x26
#define CALIBRATION						0x27
//key
#define _ok					0x1e
//#define _esc				0x1f	
#define _left				0x1b	//0x0f	
#define _up					0x0f	//0x17
#define _down				0x1d	//0x1b	
#define _right				0x17	//0x1d
//
#define INTERVAL_MEAS_TEMP	2
#define INTERVAL_MEAS_BAT	1	
#define TIME_LIGHT_OFF		4*60		//4 min
//------------------------------------------------------------------------------------------------------------------
//eeprom setpoints timers battery

uint32_t ee_work_bat_setp_time_bin_1 __attribute__((section(".eeprom"))) = 60;	//0x00;

uint32_t ee_rest_bat_setp_time_bin_1 __attribute__((section(".eeprom"))) = 60;	//0x00;

uint32_t ee_work_temperat_setp_time_bin_1  __attribute__((section(".eeprom"))) = 60;	//0x00;

uint32_t ee_rest_temperat_setp_time_bin_1 __attribute__((section(".eeprom"))) = 60;	//0x00;

uint16_t ee_setp_low_voltage_bat_bin_1 __attribute__((section(".eeprom"))) = 420;	//0x00;

//eeprom setpoints temperature

uint16_t ee_setp_low_temp_bin_1 __attribute__((section(".eeprom"))) = 50;	//0x00;

uint16_t ee_setp_hight_temp_bin_1 __attribute__((section(".eeprom"))) = 450;	//0x00;

//------------------------------------------------------------------------------------------------------------------
uint16_t coefficient1 __attribute__((section(".eeprom"))) = 932;				//1000;		//

//-------------------------------------------------------------------------------------------------------------------
uint8_t ee_type_generator1 __attribute__((section(".eeprom"))) = 0;

//-------------------------------------------------------------------------------------------------------------------
uint8_t ee_enable_battery_voltage __attribute__((section(".eeprom"))) = 1;
//-------------------------------------------------------------------------------------------------------------------
uint8_t ee_enable_temperature_low __attribute__((section(".eeprom"))) = 1;
//-------------------------------------------------------------------------------------------------------------------
uint8_t ee_enable_temperature_hight __attribute__((section(".eeprom"))) = 1;
//-------------------------------------------------------------------------------------------------------------------


uint8_t flag_meas_bat=0;
uint8_t flag_meas_temp=0;
uint8_t flag_calibration=0;
uint8_t mul=1;
volatile static uint8_t cnt_light=0;

volatile static uint8_t index_cursor;
volatile static uint8_t state;
volatile static  uint8_t key;
uint8_t wate_time[9];
static  uint8_t buf_key[8];
uint8_t flag=0;



#define STOPPED         0
#define RUNING          1
#define EVENT           2

typedef struct time_setp  {

	uint8_t string_setp_time[9];
	uint32_t setp_time_bin;
	uint8_t string_time[9];
	uint32_t time_bin;
	uint8_t status;						// 0-not work , 1-run , 2-end work
}timer;
timer time_rest_bat;
timer time_work_bat;
timer time_rest_temperat;
timer time_work_temperat;
//-----------------------------------------------------------------------------------
typedef struct meas  {
	uint8_t enable_low;			//1 - enable 0 - disable
	uint8_t enable_hight;
	uint8_t string_setp_value_hight[7];
	uint16_t setp_value_hight_bin;
	uint8_t string_setp_value_low[7];
	uint16_t setp_value_low_bin;
	uint8_t string_current_value[7];
	uint16_t current_value_bin;
	uint16_t sig;					//0 +     1 -					
}val;

val temperature;

uint8_t enable_battery_voltage;			//1 - enable 0 - disable
uint8_t battery_voltage_string_setp_value_low[5];		//!![7];
uint16_t battery_voltage_setp_value_low_bin;
uint8_t battery_voltage_string_current_value[5];	//[7];
uint16_t battery_voltage_current_value_bin;
uint16_t battery_voltage_coefficient;	

uint8_t type_generator;






uint8_t test_battery(void);
uint8_t test_temperature(void);

//func
static void build_windou_mains_on(void);
static void build_windou_mains_off(void);
static void build_windou_work(void);
static void build_windou_rest(void);

static void build_menu_setings(void);
static void build_menu_control(void);
static void build_menu_manual_start(void);

static void build_menu_voltage_timer(void);
static void build_menu_temperature_timer(void);
static void build_menu_generator(void);
static void build_menu_battery_voltage(void);
static void build_menu_temperature_low(void);
static void build_menu_temperature_hight(void);
static void build_menu_enable_disable(void);

static void build_menu_voltage_working_time(void);
static void build_menu_voltage_resting_time(void);
static void build_menu_temperature_working_time(void);
static void build_menu_temperature_resting_time(void);
static void build_menu_generator_type(void);
static void build_menu_battery_lower_limit(void);
//static void build_menu_battery_high_limit(void);
static void build_menu_temperature_lower_limit(void);
static void build_menu_temperature_high_limit(void);

static void build_menu_manually_started(void);

static void build_windou_stored(void);

static void build_windou_calobration(void);

static void build_windou_warning(void);

static void clr_key(void);
static void clr_blink(void);
static void store_setp(void);

static void blinc_cursor(uint8_t n);
static void blinc_cursor_0(void);
static void blinc_cursor_1(void);
static void blinc_cursor_2(void);
static void blinc_cursor_3(void);
static void blinc_cursor_4(void);
//static void blinc_cursor_5(void);
static void blinc_cursor_10(void);

static uint8_t control_key_val(uint8_t* array_str);
static uint8_t control_key_time(uint8_t* array_str);

static void RTC_init(void);
static void init_ADC(void);
static uint16_t meas_ADC(uint8_t channelADC);
static void read_eeprom_setp(void);

//int
void inc_comp(timer* time);
void inc_wait_timer(void);

//------------------------------------------------------------------------------------------------------------------

#include <math.h>


/* Определяем куда подключен датчик */
#define THERM_PORT PORTA
#define THERM_DDR DDRA
#define THERM_PIN PINA
#define THERM_DQ PA6
/* Макросы для "дергания ноги" и изменения режима ввода/вывода */
#define THERM_INPUT_MODE() THERM_DDR&=~(1<<THERM_DQ)
#define THERM_OUTPUT_MODE() THERM_DDR|=(1<<THERM_DQ)
#define THERM_LOW() THERM_PORT&=~(1<<THERM_DQ)
#define THERM_HIGH() THERM_PORT|=(1<<THERM_DQ)

// сброс датчика
uint8_t therm_reset(){
	uint8_t i;
	// опускаем ногу вниз на 480uS
	THERM_LOW();
	THERM_OUTPUT_MODE();
	_delay_us(480);             // замените функцию задержки на свою
	// подымаем линию на 60uS
	THERM_INPUT_MODE();
	_delay_us(60);
	// получаем значение на линии в период 480uS
	i=(THERM_PIN & (1<<THERM_DQ));
	_delay_us(420);
	// возвращаем значение (0=OK, 1=датчик не найден)
	return i;
}


// функция отправки бита
void therm_write_bit(uint8_t bit){
	// опускаем на 1uS
	THERM_LOW();
	THERM_OUTPUT_MODE();
	_delay_us(1);
	// если хотим отправить 1, поднимаем линию (если нет, оставляем как есть)
	if(bit) THERM_INPUT_MODE();
	// ждем 60uS и поднимаем линию
	_delay_us(60);
	THERM_INPUT_MODE();
}

// чтение бита
uint8_t therm_read_bit(void){
	uint8_t bit=0;
	// опускаем на 1uS
	THERM_LOW();
	THERM_OUTPUT_MODE();
	_delay_us(1);
	// поднимаем на 14uS
	THERM_INPUT_MODE();
	_delay_us(14);
	// читаем состояние
	if(THERM_PIN&(1<<THERM_DQ)) bit=1;
	// ждем 45 мкс и возвращаем значение
	_delay_us(45);
	return bit;
}

// читаем байт
uint8_t therm_read_byte(void){
	uint8_t i=8, n=0;
	while(i--){
		// сдвигаем в право на 1 и сохраняем следующее значение
		n>>=1;
		n|=(therm_read_bit()<<7);
	}
	return n;
}

// отправляем байт
void therm_write_byte(uint8_t byte){
	uint8_t i=8;
	while(i--){
		// отправляем бит и сдвигаем вправо на 1
		therm_write_bit(byte&1);
		byte>>=1;
	}
}


// команды управления датчиком
#define THERM_CMD_CONVERTTEMP 0x44
#define THERM_CMD_RSCRATCHPAD 0xbe
#define THERM_CMD_WSCRATCHPAD 0x4e
#define THERM_CMD_CPYSCRATCHPAD 0x48
#define THERM_CMD_RECEEPROM 0xb8
#define THERM_CMD_RPWRSUPPLY 0xb4
#define THERM_CMD_SEARCHROM 0xf0
#define THERM_CMD_READROM 0x33
#define THERM_CMD_MATCHROM 0x55
#define THERM_CMD_SKIPROM 0xcc
#define THERM_CMD_ALARMSEARCH 0xec

#define THERM_DECIMAL_STEPS_12BIT 625 //.0625

//------------------------------------------------------------------------------------------------------------------
// init RTC
//------------------------------------------------------------------------------------------------------------------
static void RTC_init(void)
{
	
	//Ожидание завершения стабилизации внешнего
	//!! кварцевого генератора часов RTC:
	for (uint8_t i=0; i < 0x40; i++)
	{
		for (int j=0; j < 0xFFFF; j++)	wdt_reset ();
	}
	cli();
	// Отключаем прерывания Таймера 2.
	TIMSK &= ~(_BV(TOIE2) | _BV(OCIE2));
	
	//Настройка Timer/counter0 на работу асинхронно по отношению
	// к тактированию ядра, от внешнего кварцевого резонатора
	// 32768 Гц:
	ASSR |= (1 << AS2);
	//Сброс таймера:
	TCNT2 =0;
	//Настройка прескалера таймера, чтобы получить частоту
	// деления на 128, и получить 1-секундный интервал
	// между прерываниями по переполнению таймера:
	TCCR2 =(1 << CS20)|(1 << CS22);
	//Ожидание завершения обновления TC0:
	while (ASSR & ((1 << TCN2UB)|(1 << OCR2UB)|(1 << TCR2UB)))
	{wdt_reset ();}
	//Разрешение прерывания по переполнению 8-разрядного
	// Timer/Counter2 (Overflow Interrupt Enable):
	TIMSK |= (1 << TOIE2);
	//Общее разрешение прерываний:
	sei();
	//Выбор режиме энергопотребления (power save mode),
	// когда микроконтроллер переходит в режим сна (sleep mode):	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	//Разрешение входа в режим сна:	sleep_enable();
}
//------------------------------------------------------------------------------------------------------------------
ISR(TIMER2_OVF_vect)
{
static uint8_t interval_meas_bat=0;
static uint8_t interval_meas_temp=0;	
	//1 sec
	//--------------------------------------
	cnt_light++;
	if(cnt_light>=TIME_LIGHT_OFF){
		OFF_LCD_BACKLIGHT;
		cnt_light=TIME_LIGHT_OFF;
	}
	//--------------------------------------
	interval_meas_bat++;
	if(interval_meas_bat >= INTERVAL_MEAS_BAT){
		interval_meas_bat=0;
		flag_meas_bat=1;
	}
	interval_meas_temp++;
	if(interval_meas_temp >= INTERVAL_MEAS_TEMP){
		interval_meas_temp=0;
		flag_meas_temp=1;
	}
	
	//timer
	if(time_work_bat.status == RUNING){
		
		inc_comp(&time_work_bat);
	}
	if(time_rest_bat.status == RUNING){
		
		inc_comp(&time_rest_bat);		
	}
	if(time_work_temperat.status == RUNING){
		
		inc_comp(&time_work_temperat);
	}
	if(time_rest_temperat.status == RUNING){
		
		inc_comp(&time_rest_temperat);
	}
	if (state==WAIT)
	{
		inc_wait_timer();	
	}
}
//------------------------------------------------------------------------------------------------------------------
/*ISR( TIMER1_COMPB_vect )
{
	;

}*/
//------------------------------------------------------------------------------------------------------------------
/*ISR( TIMER1_COMPA_vect )
{
	;
}*/
//**************************************************************************
/*ISR(USART_RXC_vect)
{
//uint8_t sreg;
//sreg = SREG;

/////////////////////////////////////////
// 	uint8_t receivedData;
	
	//get addres device
	//addr_devise;
///	wdt_reset ();
//	receivedData = UDR;//( uint8_t )UDR;	//Collect data.
	
//SREG = sreg;	
}*/
//------------------------------------------------------------------------------------------------------------------
static void init_ADC(void)
{
	ACSR = (1<<ACBG);
	ADMUX = (1<<REFS1)|(1<<REFS0);
	//ADMUX = (1<<REFS0);
	ADCSRA = (1<<ADEN);
}
//------------------------------------------------------------------------------------------------------------------
static uint16_t meas_ADC(uint8_t channelADC)
{
	//N=N&0x0f;
	uint16_t adcval;
	uint8_t nn;
	//start convert
	nn=16;
	adcval = 0;
	ADMUX=0xc0+channelADC;
	//ADMUX=0x40+channelADC;
	for	(uint16_t i=0;i<nn;i++)	{
		ADCSRA = _BV(ADEN)|_BV(ADPS0)|_BV(ADPS1)|_BV(ADPS2);
		ADCSRA |= _BV(ADSC);
		loop_until_bit_is_clear(ADCSRA,ADSC);
		adcval += ADCW;
	}
	adcval=adcval/nn;
	//  ADCSRA &= ~_BV(ADEN);		// disable ADC  //
	return adcval;
	
}
//------------------------------------------------------------------------------------------------------------------
void inc_wait_timer(void){	
	//inc wate_time[9];
	wate_time[7]++;
	if(wate_time[7]>=0x3a){
		wate_time[7]=0x30;
		wate_time[6]++;
		if(wate_time[6]>=0x36){
			wate_time[6]=0x30;
			wate_time[4]++;
			if(wate_time[4]>=0x3a){
				wate_time[4]=0x30;
				wate_time[3]++;
				if(wate_time[3]>=0x36){
					wate_time[3]=0x30;
					wate_time[1]++;
					if(wate_time[1]>=0x3a){
						wate_time[1]=0x30;
						wate_time[0]++;
						if(wate_time[0]>=0x3a){
							wate_time[0]=0x30;
						}
					}
				}
			}
		}
	}
}
//------------------------------------------------------------------------------------------------------------------
//static void int 
void inc_comp(timer* time){
	//inc
	time->string_time[7]++;
	if(time->string_time[7]>=0x3a){
		time->string_time[7]=0x30;
		time->string_time[6]++;
		if(time->string_time[6]>=0x36){
			time->string_time[6]=0x30;
			time->string_time[4]++;
			if(time->string_time[4]>=0x3a){
				time->string_time[4]=0x30;
				time->string_time[3]++;
				if(time->string_time[3]>=0x36){
					time->string_time[3]=0x30;
					time->string_time[1]++;
					if(time->string_time[1]>=0x3a){
						time->string_time[1]=0x30;
						time->string_time[0]++;
						if(time->string_time[0]>=0x3a){
							time->string_time[0]=0x30;
						}
					}
				}
			}
		}
	}
	///compare  
	time->time_bin= 36000*(time->string_time[0]&0x0F)+3600*(time->string_time[1]&0x0F)+600*(time->string_time[3]&0x0F)+60*(time->string_time[4]&0x0F)+10*(time->string_time[6]&0x0F)+(time->string_time[7]&0x0F);
	time->setp_time_bin= 36000*(time->string_setp_time[0]&0x0F)+3600*(time->string_setp_time[1]&0x0F)+600*(time->string_setp_time[3]&0x0F)+60*(time->string_setp_time[4]&0x0F)+10*(time->string_setp_time[6]&0x0F)+(time->string_setp_time[7]&0x0F);
	
		if (time->time_bin >= time->setp_time_bin)
		{
			time->status=EVENT;
//			return 1;
		} 
		else
		{
//			return 0;
		}
}
//------------------------------------------------------------------------------------------------------------------
//return 1 if cursor output left
//return 2 if cursor output right
//return 3 update LCD

static uint8_t control_key_time(uint8_t* array_str){
if(key== _right){
	index_cursor++;
	if(index_cursor>=11){
		//clr_blink();
		clr_key();
		return 2;
	}
	else
	{
		if(index_cursor==2){
			index_cursor++;
		}
		if (index_cursor>=5){
			index_cursor=10;
			}
		//clr_blink();
		blinc_cursor(index_cursor);
		clr_key();
	}
}else
if(key== _up){
	array_str[index_cursor]++;
	if(index_cursor==3){
		if (array_str[index_cursor]>=0x36)
		{
			array_str[index_cursor]=0x35;
		}
	}
	if((index_cursor==0)||(index_cursor==1)||(index_cursor==4)){
		if (array_str[index_cursor]>=0x3a)
		{
			array_str[index_cursor]=0x39;
		}
	}
	clr_key();
	return 3;
}else
if(key==_down){
	if((index_cursor==0)||(index_cursor==1)||(index_cursor==3)||(index_cursor==4)){
		if(array_str[index_cursor]==0x30){
			;
			}else{
			array_str[index_cursor]--;
		}
	}
	if(index_cursor==10){
		store_setp();
		read_eeprom_setp();
		clr_blink();
		build_windou_stored();
		wdt_reset ();
		_delay_ms(1000);
		wdt_reset ();
		//blinc_cursor(index_cursor);
	}
	clr_key();
	return 3;
}else
if(key==_left){
	if(index_cursor==0){
		//clr_blink();
		clr_key();
		return 1;
	}
	else
		{
		if(index_cursor==10){
			index_cursor=4;
		}else	if(index_cursor==3){
									index_cursor=1;
									}else{index_cursor--;}
		//clr_blink();
		blinc_cursor(index_cursor);
		clr_key();
		}
	}
	return 0;
}
//-----------------------------------------------------------------------------------------------------------------
//return 1 if cursor output left
//return 2 if cursor output right

static uint8_t control_key_val(uint8_t* array_str){
if(key== _right){
	index_cursor++;
	if(index_cursor>10){
		//clr_blink();
		clr_key();
		return 2;
	}
	else
	{
	
		if(index_cursor==2){
			index_cursor=3;
		}
		if (index_cursor>=4){
			index_cursor=10;
		}
	}
	//clr_blink();
	blinc_cursor(index_cursor);
	clr_key();		
}else
if(key== _up){
	array_str[index_cursor]++;
	if(index_cursor<=3){
		if (array_str[index_cursor]>=0x3a)
		{
			array_str[index_cursor]=0x39;
		}
	}
	clr_key();
	return 3;
}else
if(key==_down){
	if(index_cursor<=3){
		if(array_str[index_cursor]==0x30){
			;
		}else
		{
			array_str[index_cursor]--;
		}
	}
	if(index_cursor==10){
		
			if ((state==TEMPERATURE_LOWER_LIMIT)||(state==TEMPERATURE_HIGH_LIMIT))
			{
				
				temperature.setp_value_low_bin=100*(temperature.string_setp_value_low[0]&0x0F)+10*(temperature.string_setp_value_low[1]&0x0F)+(temperature.string_setp_value_low[3]&0x0F);	//##.# - ###   *10;
				temperature.setp_value_hight_bin=100*(temperature.string_setp_value_hight[0]&0x0F)+10*(temperature.string_setp_value_hight[1]&0x0F)+(temperature.string_setp_value_hight[3]&0x0F);	//##.# - ###   *10;
				if (temperature.setp_value_low_bin>=temperature.setp_value_hight_bin)
				{
					clr_blink();
					build_windou_warning();
					wdt_reset ();
					_delay_ms(1000);
					wdt_reset ();
					_delay_ms(500);
					wdt_reset ();
					
				} 
				else
				{
					store_setp();
					read_eeprom_setp();
					clr_blink();
					build_windou_stored();
					wdt_reset ();
					_delay_ms(1000);
					wdt_reset ();
					
				}
				
					
			} 
			else
			{
					store_setp();
					read_eeprom_setp();
					clr_blink();
					build_windou_stored();
					wdt_reset ();
					_delay_ms(1000);
					wdt_reset ();
					
			}
	}
	clr_key();
	return 3;
}else
if(key==_left){
		if(index_cursor==0){
			clr_key();
			return 1;
		}
		else
		{
			if(index_cursor==10){
				index_cursor=3;	
			}else	if(index_cursor==3){
				index_cursor=1;
			}else{	
				index_cursor--;
			}
			blinc_cursor(index_cursor);
			clr_key();
		}
}
		return 0;
}
//------------------------------------------------------------------------------------------------------------------
static void build_windou_mains_on(void)
{
	lcd_1_strin();
	printf("AC ON   %s ",wate_time);
	lcd_2_strin();
	printf("%s v   ",battery_voltage_string_current_value);
	if(temperature.sig) printf("-%s ",temperature.string_current_value);
	else printf(" %s ",temperature.string_current_value);
}
//------------------------------------------------------------------------------------------------------------------
static void build_windou_mains_off(void)
{
	lcd_1_strin();
	printf("AC OFF  %s ",wate_time);
	lcd_2_strin();
	printf("%s v   ",battery_voltage_string_current_value);
	if(temperature.sig) printf("-%s ",temperature.string_current_value);
	else printf(" %s ",temperature.string_current_value);
}
//------------------------------------------------------------------------------------------------------------------
static void build_windou_work(void)
{
	lcd_1_strin();
	printf("Bat:%s ",battery_voltage_string_current_value);
	if(temperature.sig) printf("-%s",temperature.string_current_value);
	else printf(" %s",temperature.string_current_value);
	lcd_2_strin();
	if(time_work_bat.status==1)	printf("Working:%s ",time_work_bat.string_time);
	if(time_work_temperat.status==1)	printf("Working:%s ",time_work_temperat.string_time);
}
//------------------------------------------------------------------------------------------------------------------
static void build_windou_rest(void)
{
	lcd_1_strin();
	printf("Bat:%s ",battery_voltage_string_current_value);
	if(temperature.sig) printf("-%s ",temperature.string_current_value);
	else printf(" %s ",temperature.string_current_value);
	lcd_2_strin();
	if(time_rest_bat.status==1)	printf("Resting:%s ",time_rest_bat.string_time);
	if(time_rest_temperat.status==1)	printf("Resting:%s ",time_rest_temperat.string_time);
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_setings(void)
{
	lcd_1_strin();
	printf_P(PSTR("    SETINGS     "));
	lcd_2_strin();
	printf_P(PSTR("<-exit   enter->"));
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_control(void)
{
	lcd_1_strin();
	printf_P(PSTR("    CONTROL     "));
	lcd_2_strin();
	printf_P(PSTR("<-exit   enter->"));
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_manual_start(void)
{
	lcd_1_strin();
	printf_P(PSTR("  MANUAL START  "));
	lcd_2_strin();
	printf_P(PSTR("<-exit   enter->"));
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_voltage_timer(void)
{
	lcd_1_strin();
	printf_P(PSTR(" Voltage Timer  "));
	lcd_2_strin();
	printf_P(PSTR("<-exit   enter->"));
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_temperature_timer(void)
{
	lcd_1_strin();
	printf_P(PSTR(" Temper. Timer  "));
	lcd_2_strin();
	printf_P(PSTR("<-exit   enter->"));
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_generator(void)
{
	lcd_1_strin();
	printf_P(PSTR("   Generator    "));
	lcd_2_strin();
	printf_P(PSTR("<-exit   enter->"));
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_battery_voltage(void)
{
	lcd_1_strin();
	printf_P(PSTR("Battery voltage "));
	lcd_2_strin();
	printf_P(PSTR("<-exit   enter->"));
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_temperature_low(void)
{
	lcd_1_strin();
	printf_P(PSTR("Low  temperature"));
	lcd_2_strin();
	printf_P(PSTR("<-exit   enter->"));
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_temperature_hight(void)
{
	lcd_1_strin();
	printf_P(PSTR("Hight temperat. "));
	lcd_2_strin();
	printf_P(PSTR("<-exit   enter->"));
}
//------------------------------------------------------------------------------------------------------------------
void build_menu_enable_disable(void)
{
	lcd_1_strin();
	printf_P(PSTR(" Enable/Disable "));
	lcd_2_strin();
	printf_P(PSTR("<-exit   enter->"));
}

//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------

static void build_menu_voltage_working_time(void)
{
	lcd_1_strin();
	printf_P(PSTR(" Working Time   "));
	lcd_2_strin();
	printf("%.5s     Save   ",time_work_bat.string_setp_time);
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_voltage_resting_time(void)
{
	lcd_1_strin();
	printf_P(PSTR(" Resting Time   "));
	lcd_2_strin();
	printf("%.5s     Save   ",time_rest_bat.string_setp_time);
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_temperature_working_time(void)
{
	lcd_1_strin();
	printf_P(PSTR(" Working Time   "));
	lcd_2_strin();
	printf("%.5s     Save   ",time_work_temperat.string_setp_time);
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_temperature_resting_time(void)
{
	lcd_1_strin();
	printf_P(PSTR(" Resting Time   "));
	lcd_2_strin();
	printf("%.5s     Save   ",time_rest_temperat.string_setp_time);
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_generator_type(void)
{
	lcd_1_strin();
	printf_P(PSTR(" Generator Type "));
	lcd_2_strin();
	printf_P(PSTR("Perkins   Yanmar"));
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_battery_lower_limit(void)
{
	lcd_1_strin();
	printf_P(PSTR("Bat. lower limit"));
	lcd_2_strin();
	printf("%s      Save  ",battery_voltage_string_setp_value_low);
}
//------------------------------------------------------------------------------------------------------------------
//static void build_menu_battery_high_limit(void);
//------------------------------------------------------------------------------------------------------------------
static void build_menu_temperature_lower_limit(void)
{
	lcd_1_strin();
	printf_P(PSTR("Temp. low limit "));
	lcd_2_strin();
	printf("%s    Save  ",temperature.string_setp_value_low);
}
//------------------------------------------------------------------------------------------------------------------
static void build_menu_temperature_high_limit(void)
{
	lcd_1_strin();
	printf_P(PSTR("Temp. high limit"));
	lcd_2_strin();
	printf("%s    Save  ",temperature.string_setp_value_hight);
}
//------------------------------------------------------------------------------------------------------------------
void build_menu_enable_disable_setpoint_bat_voltage(void)
{
	lcd_1_strin();
	printf_P(PSTR("Setp. bat. volt."));
	lcd_2_strin();
	if (enable_battery_voltage)
	{
		printf_P(PSTR("   Enabled      "));
	} 
	else
	{
		printf_P(PSTR("   Disabled     "));
	}
}
//------------------------------------------------------------------------------------------------------------------

void build_menu_enable_disable_setpoint_temp_low(void)
{
	lcd_1_strin();
	printf_P(PSTR("Setp. temp. low "));
	lcd_2_strin();
	if (temperature.enable_low)
	{
		printf_P(PSTR("   Enabled      "));
	}
	else
	{
		printf_P(PSTR("   Disabled     "));
	}
}
//------------------------------------------------------------------------------------------------------------------
void build_menu_enable_disable_setpoint_temp_hight(void)
{
	lcd_1_strin();
	printf_P(PSTR("Setp. temp. high"));
	lcd_2_strin();
	if (temperature.enable_hight)
	{
		printf_P(PSTR("   Enabled      "));
	}
	else
	{
		printf_P(PSTR("   Disabled     "));
	}
}
//------------------------------------------------------------------------------------------------------------------
void build_menu_enable_disable_setpoint_stored(void)
{
	lcd_1_strin();
	printf_P(PSTR("  Save changes  "));
	lcd_2_strin();
	printf_P(PSTR("  press down    "));
}
//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------
static void build_menu_manually_started(void)
{
	lcd_1_strin();
	printf_P(PSTR("Manually Started"));
	lcd_2_strin();
	printf_P(PSTR("                "));
}
//------------------------------------------------------------------------------------------------------------------
static void build_windou_stored(void){
	lcd_1_strin();
	printf_P(PSTR("     Stored     "));
	lcd_2_strin();
	printf_P(PSTR("                "));
}
//------------------------------------------------------------------------------------------------------------------
static void build_windou_calobration(void){
	lcd_1_strin();
	printf("Battery: %s   ",battery_voltage_string_current_value);
	lcd_2_strin();
	printf("Coeff. :%0.4u %0.3u",battery_voltage_coefficient, mul);	
}
//------------------------------------------------------------------------------------------------------------------
static void build_windou_warning(void){
	lcd_1_strin();
	printf_P(PSTR("Warning setpoint"));
	lcd_2_strin();
	printf_P(PSTR("  Low  >  Hight "));
}
//------------------------------------------------------------------------------------------------------------------


void build_menu_setpoint_disabled(void)
{
	lcd_1_strin();
	printf_P(PSTR("    Setpoint    "));
	lcd_2_strin();
	printf_P(PSTR("    disabled!   "));
	_delay_ms(1000);
}

//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------
static void blinc_cursor(uint8_t n){
	if(n==0) blinc_cursor_0();
	else if(n==1) blinc_cursor_1();
	else if(n==2) blinc_cursor_2();
	else if(n==3) blinc_cursor_3();
	else if(n==4) blinc_cursor_4();
	else if(n==10) blinc_cursor_10();
	else {
		index_cursor=0;
		blinc_cursor_0();
	}
}

static void blinc_cursor_0(void)
{
hd44780_wait_ready();
hd44780_outcmd(HD44780_DDADDR(0x40));
hd44780_wait_ready();
hd44780_outcmd(HD44780_DISPCTL(1, 1, 1));
}
//------------------------------------------------------------------------------------------------------------------
static void blinc_cursor_1(void)
{
hd44780_wait_ready();
hd44780_outcmd(HD44780_DDADDR(0x41));
hd44780_wait_ready();
hd44780_outcmd(HD44780_DISPCTL(1, 1, 1));
}
//------------------------------------------------------------------------------------------------------------------
static void blinc_cursor_2(void)
{
hd44780_wait_ready();
hd44780_outcmd(HD44780_DDADDR(0x42));
hd44780_wait_ready();
hd44780_outcmd(HD44780_DISPCTL(1, 1, 1));
}
//------------------------------------------------------------------------------------------------------------------
static void blinc_cursor_3(void)
{
hd44780_wait_ready();
hd44780_outcmd(HD44780_DDADDR(0x43));
hd44780_wait_ready();
hd44780_outcmd(HD44780_DISPCTL(1, 1, 1));
}
//------------------------------------------------------------------------------------------------------------------
static void blinc_cursor_4(void)
{
hd44780_wait_ready();
hd44780_outcmd(HD44780_DDADDR(0x44));
hd44780_wait_ready();
hd44780_outcmd(HD44780_DISPCTL(1, 1, 1));
}
//------------------------------------------------------------------------------------------------------------------
/*static void blinc_cursor_5(void)
{
hd44780_wait_ready();
hd44780_outcmd(HD44780_DDADDR(0x47));
hd44780_wait_ready();
hd44780_outcmd(HD44780_DISPCTL(1, 1, 1));
}*/
//------------------------------------------------------------------------------------------------------------------
static void blinc_cursor_10(void)
{
hd44780_wait_ready();
hd44780_outcmd(HD44780_DDADDR(0x4a));
hd44780_wait_ready();
hd44780_outcmd(HD44780_DISPCTL(1, 1, 1));
}
//------------------------------------------------------------------------------------------------------------------
/*void
blinc_cursor_7(void)
{
	hd44780_wait_ready();
	hd44780_outcmd(HD44780_DDADDR(0x48));
	hd44780_wait_ready();
	hd44780_outcmd(HD44780_DISPCTL(1, 1, 1));
}*/
//------------------------------------------------------------------------------------------------------------------

static void clr_blink(void)
{
hd44780_wait_ready();
hd44780_outcmd(HD44780_DISPCTL(1, 1, 0));

}
//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------
static void clr_key(void)
{
_delay_ms(500);
wdt_reset ();
key=0x1f;
}
//------------------------------------------------------------------------------------------------------------------

static uint8_t test_key(void)
{
volatile static uint8_t index=0;
//-------------------------------------
buf_key[index]=(PINA&0x1f);

if(buf_key[index]!=0x1f){
	 cnt_light=0;
	 ON_LCD_BACKLIGHT;
}
index++;
//-------------------------------------
if (index>=8)	index=0;
	//test eqwal
	if ((buf_key[0]==buf_key[1])&&(buf_key[0]==buf_key[2])&&(buf_key[0]==buf_key[3])&&(buf_key[0]==buf_key[4])&&(buf_key[0]==buf_key[5])&&(buf_key[0]==buf_key[6])&&(buf_key[0]==buf_key[7]))
		{
			if (buf_key[0]==0x1f) 
			{
			flag=0;
			}
			//
			if(flag==0) 
			{

				if (buf_key[0]!=0x1f)
					{
					flag=1;
					return buf_key[0];
					}
				else
					{
					return 0x1f;
					}
			}
			else 
			{
			return 0x1f;
			}
		}
	else 
	{
	return 0x1f;
	}
}
//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------
static void read_eeprom_setp(void)
{
uint32_t val1=0;	
uint32_t val2=0;
//uint32_t val3=0;
	
wdt_reset ();	
//eeprom_busy_wait();
time_work_bat.setp_time_bin	=	eeprom_read_dword	(&ee_work_bat_setp_time_bin_1);
//wdt_reset ();	

val1=time_work_bat.setp_time_bin;
val2=val1/36000;
time_work_bat.string_setp_time[0] = ((uint8_t)val2)|0x30;		//старший разряд часа
val1-=val2*36000;
val2=val1/3600;
time_work_bat.string_setp_time[1] = ((uint8_t)val2)|0x30;
time_work_bat.string_setp_time[2] = 0x3a;		// :
val1-=val2*3600;
val2=val1/600;
time_work_bat.string_setp_time[3] = ((uint8_t)val2)|0x30;
val1-=val2*600;
val2=val1/60;
time_work_bat.string_setp_time[4] = ((uint8_t)val2)|0x30;
time_work_bat.string_setp_time[5] = 0x3a;		// :
time_work_bat.string_setp_time[6] = 0x30;
time_work_bat.string_setp_time[7] = 0x30;
time_work_bat.string_setp_time[8] = 0;

//----------------------------------------------------------------------------
wdt_reset ();
//eeprom_busy_wait();
time_rest_bat.setp_time_bin	=	eeprom_read_dword	(&ee_rest_bat_setp_time_bin_1);
//wdt_reset ();

val1=time_rest_bat.setp_time_bin;
val2=val1/36000;
time_rest_bat.string_setp_time[0] = (uint8_t)val2|0x30;		//старший разряд часа
val1-=val2*36000;
val2=val1/3600;
time_rest_bat.string_setp_time[1] = (uint8_t)val2|0x30;
time_rest_bat.string_setp_time[2] = 0x3a;		// :
val1-=val2*3600;
val2=val1/600;
time_rest_bat.string_setp_time[3] = (uint8_t)val2|0x30;
val1-=val2*600;
val2=val1/60;
time_rest_bat.string_setp_time[4] = (uint8_t)val2|0x30;
time_rest_bat.string_setp_time[5] = 0x3a;		// :
time_rest_bat.string_setp_time[6] = 0x30;
time_rest_bat.string_setp_time[7] = 0x30;
time_rest_bat.string_setp_time[8] = 0;

//-----------------------------------------------------------------------------------
wdt_reset ();
//eeprom_busy_wait();
time_work_temperat.setp_time_bin =	eeprom_read_dword	(&ee_work_temperat_setp_time_bin_1);
//wdt_reset ();

val1=time_work_temperat.setp_time_bin;
val2=val1/36000;
time_work_temperat.string_setp_time[0] = (uint8_t)val2|0x30;		//старший разряд часа
val1-=val2*36000;
val2=val1/3600;
time_work_temperat.string_setp_time[1] = (uint8_t)val2|0x30;
time_work_temperat.string_setp_time[2] = 0x3a;		// :
val1-=val2*3600;
val2=val1/600;
time_work_temperat.string_setp_time[3] = (uint8_t)val2|0x30;
val1-=val2*600;
val2=val1/60;
time_work_temperat.string_setp_time[4] = (uint8_t)val2|0x30;
time_work_temperat.string_setp_time[5] = 0x3a;		// :
time_work_temperat.string_setp_time[6] = 0x30;
time_work_temperat.string_setp_time[7] = 0x30;
time_work_temperat.string_setp_time[8] = 0;

//------------------------------------------------------------------------------------
wdt_reset ();
//eeprom_busy_wait();
time_rest_temperat.setp_time_bin =	eeprom_read_dword	(&ee_rest_temperat_setp_time_bin_1);
//wdt_reset ();

val1=time_rest_temperat.setp_time_bin;
val2=val1/36000;
time_rest_temperat.string_setp_time[0] = (uint8_t)val2|0x30;		//старший разряд часа
val1-=val2*36000;
val2=val1/3600;
time_rest_temperat.string_setp_time[1] = (uint8_t)val2|0x30;
time_rest_temperat.string_setp_time[2] = 0x3a;		// :
val1-=val2*3600;
val2=val1/600;
time_rest_temperat.string_setp_time[3] = (uint8_t)val2|0x30;
val1-=val2*600;
val2=val1/60;
time_rest_temperat.string_setp_time[4] = (uint8_t)val2|0x30;
time_rest_temperat.string_setp_time[5] = 0x3a;		// :
time_rest_temperat.string_setp_time[6] = 0x30;
time_rest_temperat.string_setp_time[7] = 0x30;
time_rest_temperat.string_setp_time[8] = 0;

//-------------------------------------------------------------------------------------
wdt_reset ();
//eeprom_busy_wait();
battery_voltage_setp_value_low_bin	=	(uint32_t)	eeprom_read_word (&ee_setp_low_voltage_bat_bin_1);
//wdt_reset ();

val1=battery_voltage_setp_value_low_bin;
val2=val1/100;
battery_voltage_string_setp_value_low[0] = (uint8_t)val2|0x30;
val1-=val2*100;
val2=val1/10;
battery_voltage_string_setp_value_low[1] = (uint8_t)val2|0x30;
battery_voltage_string_setp_value_low[2] = 0x2e;
val1-=val2*10;
battery_voltage_string_setp_value_low[3] = (uint8_t)val1|0x30;	
battery_voltage_string_setp_value_low[4] = 0;

//--------------------------------------------------------------------------------------------
wdt_reset ();
//eeprom_busy_wait();
temperature.setp_value_low_bin	=	(uint32_t)	eeprom_read_word	(&ee_setp_low_temp_bin_1);
//wdt_reset ();

val1=temperature.setp_value_low_bin;
val2=val1/100;
temperature.string_setp_value_low[0] = (uint8_t)val2|0x30;
val1-=val2*100;
val2=val1/10;
temperature.string_setp_value_low[1] = (uint8_t)val2|0x30;
temperature.string_setp_value_low[2] = 0x2e;
val1-=val2*10;
temperature.string_setp_value_low[3] = (uint8_t)val1|0x30;
temperature.string_setp_value_low[4] = 0xdf;
temperature.string_setp_value_low[5] = 0x43;
temperature.string_setp_value_low[6] = 0;

//
wdt_reset ();
//eeprom_busy_wait();
temperature.setp_value_hight_bin	=	(uint32_t)	eeprom_read_word	(&ee_setp_hight_temp_bin_1);
//wdt_reset ();

val1=temperature.setp_value_hight_bin;
val2=val1/100;
temperature.string_setp_value_hight[0] = (uint8_t)val2|0x30;
val1-=val2*100;
val2=val1/10;
temperature.string_setp_value_hight[1] = (uint8_t)val2|0x30;
temperature.string_setp_value_hight[2] = 0x2e;
val1-=val2*10;
temperature.string_setp_value_hight[3] = (uint8_t)val1|0x30;
temperature.string_setp_value_hight[4] = 0xdf;
temperature.string_setp_value_hight[5] = 0x43;
temperature.string_setp_value_hight[6] = 0;
//--------------------------------------------------------------------------------
wdt_reset ();
//eeprom_busy_wait();
battery_voltage_coefficient	=	(uint32_t)	eeprom_read_word	(&coefficient1);
//wdt_reset ();

if(battery_voltage_coefficient>2000){
	//eeprom_busy_wait();
	eeprom_write_word(&coefficient1,2000);
	//eeprom_busy_wait();
	battery_voltage_coefficient=eeprom_read_word(&coefficient1);
}
//eeprom_is_ready 	( );
//------------------------------------------------------------------------------------
wdt_reset ();
//eeprom_busy_wait();
type_generator	=	eeprom_read_byte	(&ee_type_generator1);
//eeprom_busy_wait();
enable_battery_voltage = eeprom_read_byte	(&ee_enable_battery_voltage);
//eeprom_busy_wait();
temperature.enable_low = eeprom_read_byte	(&ee_enable_temperature_low);
//eeprom_busy_wait();
temperature.enable_hight = eeprom_read_byte	(&ee_enable_temperature_hight);
wdt_reset ();
//eeprom_is_ready 	( );


}
//------------------------------------------------------------------------------------------------------------------
static void store_setp(void)
{
	wdt_reset ();
	time_work_bat.setp_time_bin=36000*(time_work_bat.string_setp_time[0]&0x0F)+3600*(time_work_bat.string_setp_time[1]&0x0F)+600*(time_work_bat.string_setp_time[3]&0x0F)+60*(time_work_bat.string_setp_time[4]&0x0F)+10*(time_work_bat.string_setp_time[6]&0x0F)+(time_work_bat.string_setp_time[7]&0x0F);
	//eeprom_busy_wait();
	eeprom_write_dword(&ee_work_bat_setp_time_bin_1,time_work_bat.setp_time_bin);
	//-----------------------------------------------------------------
	wdt_reset ();
	time_rest_bat.setp_time_bin=36000*(time_rest_bat.string_setp_time[0]&0x0F)+3600*(time_rest_bat.string_setp_time[1]&0x0F)+600*(time_rest_bat.string_setp_time[3]&0x0F)+60*(time_rest_bat.string_setp_time[4]&0x0F)+10*(time_rest_bat.string_setp_time[6]&0x0F)+(time_rest_bat.string_setp_time[7]&0x0F);
	//eeprom_busy_wait();
	eeprom_write_dword(&ee_rest_bat_setp_time_bin_1,time_rest_bat.setp_time_bin);
	//-----------------------------------------------------------------
	wdt_reset ();
	time_work_temperat.setp_time_bin=36000*(time_work_temperat.string_setp_time[0]&0x0F)+3600*(time_work_temperat.string_setp_time[1]&0x0F)+600*(time_work_temperat.string_setp_time[3]&0x0F)+60*(time_work_temperat.string_setp_time[4]&0x0F)+10*(time_work_temperat.string_setp_time[6]&0x0F)+(time_work_temperat.string_setp_time[7]&0x0F);
	//eeprom_busy_wait();
	eeprom_write_dword(&ee_work_temperat_setp_time_bin_1,time_work_temperat.setp_time_bin);
	//-----------------------------------------------------------------
	wdt_reset ();
	time_rest_temperat.setp_time_bin=36000*(time_rest_temperat.string_setp_time[0]&0x0F)+3600*(time_rest_temperat.string_setp_time[1]&0x0F)+600*(time_rest_temperat.string_setp_time[3]&0x0F)+60*(time_rest_temperat.string_setp_time[4]&0x0F)+10*(time_rest_temperat.string_setp_time[6]&0x0F)+(time_rest_temperat.string_setp_time[7]&0x0F);
	//eeprom_busy_wait();
	eeprom_write_dword(&ee_rest_temperat_setp_time_bin_1,time_rest_temperat.setp_time_bin);
	//-----------------------------------------------------------------
	wdt_reset ();
	battery_voltage_setp_value_low_bin=100*(battery_voltage_string_setp_value_low[0]&0x0F)+10*(battery_voltage_string_setp_value_low[1]&0x0F)+(battery_voltage_string_setp_value_low[3]&0x0F);
	//eeprom_busy_wait();
	eeprom_write_word(&ee_setp_low_voltage_bat_bin_1,battery_voltage_setp_value_low_bin);
	//-----------------------------------------------------------------
	wdt_reset ();
	temperature.setp_value_low_bin=100*(temperature.string_setp_value_low[0]&0x0F)+10*(temperature.string_setp_value_low[1]&0x0F)+(temperature.string_setp_value_low[3]&0x0F);	//##.# - ###   *10;
	//eeprom_busy_wait();
	eeprom_write_word(&ee_setp_low_temp_bin_1,temperature.setp_value_low_bin);
	//-----------------------------------------------------------------
	wdt_reset ();
	temperature.setp_value_hight_bin=100*(temperature.string_setp_value_hight[0]&0x0F)+10*(temperature.string_setp_value_hight[1]&0x0F)+(temperature.string_setp_value_hight[3]&0x0F);	//##.# - ###   *10;
	//eeprom_busy_wait();
	eeprom_write_word(&ee_setp_hight_temp_bin_1,temperature.setp_value_hight_bin);
	//-----------------------------------------------------------------
	wdt_reset ();
	//eeprom_busy_wait();
	eeprom_write_word(&coefficient1,battery_voltage_coefficient);
	//---------------------------------------------------------------
	wdt_reset ();
	//eeprom_busy_wait();
	eeprom_write_byte(&ee_type_generator1,type_generator);
	//eeprom_busy_wait();
	eeprom_write_byte(&ee_enable_battery_voltage,enable_battery_voltage);
	//eeprom_busy_wait();
	eeprom_write_byte(&ee_enable_temperature_low,temperature.enable_low);
	//eeprom_busy_wait();	
	eeprom_write_byte(&ee_enable_temperature_hight,temperature.enable_hight);	
}
//!!static inline ------------------------------------------------------------------
static uint8_t test_runing_rest_timers(void){
	if(time_rest_bat.status==RUNING){
	return 1;
	}else if(time_rest_temperat.status==1){
	return 1;
	}else{
	return 0;
	}	
}
//------------------------------------------------
static void start_timer_run_bat(void){
	time_work_bat.setp_time_bin=36000*(time_work_bat.string_setp_time[0]&0x0F)+3600*(time_work_bat.string_setp_time[1]&0x0F)+600*(time_work_bat.string_setp_time[3]&0x0F)+60*(time_work_bat.string_setp_time[4]&0x0F)+10*(time_work_bat.string_setp_time[6]&0x0F)+(time_work_bat.string_setp_time[7]&0x0F);
	time_work_bat.string_time[0] = 0x30;
	time_work_bat.string_time[1] = 0x30;
	time_work_bat.string_time[2] = 0x3a;		// :
	time_work_bat.string_time[3] = 0x30;
	time_work_bat.string_time[4] = 0x30;
	time_work_bat.string_time[5] = 0x3a;		// :
	time_work_bat.string_time[6] = 0x30;
	time_work_bat.string_time[7] = 0x30;
	time_work_bat.string_time[8] = 0x00;
	time_work_bat.time_bin=0;
	time_work_bat.status=RUNING;	
}
//------------------------------------------------
//!!static inline 
static void start_timer_run_temperat(void){
	time_work_temperat.setp_time_bin=36000*(time_work_temperat.string_setp_time[0]&0x0F)+3600*(time_work_temperat.string_setp_time[1]&0x0F)+600*(time_work_temperat.string_setp_time[3]&0x0F)+60*(time_work_temperat.string_setp_time[4]&0x0F)+10*(time_work_temperat.string_setp_time[6]&0x0F)+(time_work_temperat.string_setp_time[7]&0x0F);
	time_work_temperat.string_time[0] = 0x30;
	time_work_temperat.string_time[1] = 0x30;
	time_work_temperat.string_time[2] = 0x3a;		// :
	time_work_temperat.string_time[3] = 0x30;
	time_work_temperat.string_time[4] = 0x30;
	time_work_temperat.string_time[5] = 0x3a;		// :
	time_work_temperat.string_time[6] = 0x30;
	time_work_temperat.string_time[7] = 0x30;
	time_work_temperat.string_time[8] = 0x00;	
	time_work_temperat.time_bin=0;
	time_work_temperat.status=RUNING;
}
//------------------------------------------------
//!!static inline 
static void start_timer_rest_bat(void){
	time_rest_bat.setp_time_bin=36000*(time_rest_bat.string_setp_time[0]&0x0F)+3600*(time_rest_bat.string_setp_time[1]&0x0F)+600*(time_rest_bat.string_setp_time[3]&0x0F)+60*(time_rest_bat.string_setp_time[4]&0x0F)+10*(time_rest_bat.string_setp_time[6]&0x0F)+(time_rest_bat.string_setp_time[7]&0x0F);
	time_rest_bat.string_time[0] = 0x30;
	time_rest_bat.string_time[1] = 0x30;
	time_rest_bat.string_time[2] = 0x3a;		// :
	time_rest_bat.string_time[3] = 0x30;
	time_rest_bat.string_time[4] = 0x30;
	time_rest_bat.string_time[5] = 0x3a;		// :
	time_rest_bat.string_time[6] = 0x30;
	time_rest_bat.string_time[7] = 0x30;
	time_rest_bat.string_time[8] = 0x00;
	time_rest_bat.time_bin=0;
	time_rest_bat.status=RUNING;
}
//------------------------------------------------
//!!static inline 
static void start_timer_rest_temperat(void){
	time_rest_temperat.setp_time_bin=36000*(time_rest_temperat.string_setp_time[0]&0x0F)+3600*(time_rest_temperat.string_setp_time[1]&0x0F)+600*(time_rest_temperat.string_setp_time[3]&0x0F)+60*(time_rest_temperat.string_setp_time[4]&0x0F)+10*(time_rest_temperat.string_setp_time[6]&0x0F)+(time_rest_temperat.string_setp_time[7]&0x0F);
	time_rest_temperat.string_time[0] = 0x30;
	time_rest_temperat.string_time[1] = 0x30;
	time_rest_temperat.string_time[2] = 0x3a;		// :
	time_rest_temperat.string_time[3] = 0x30;
	time_rest_temperat.string_time[4] = 0x30;
	time_rest_temperat.string_time[5] = 0x3a;		// :
	time_rest_temperat.string_time[6] = 0x30;
	time_rest_temperat.string_time[7] = 0x30;
	time_rest_temperat.string_time[8] = 0x00;
	time_rest_temperat.time_bin=0;
	time_rest_temperat.status=RUNING;
}
//------------------------------------------------
static void start_timer_wait(void){
	wate_time[0] = 0x30;
	wate_time[1] = 0x30;
	wate_time[2] = 0x3a;		// :
	wate_time[3] = 0x30;
	wate_time[4] = 0x30;
	wate_time[5] = 0x3a;		// :
	wate_time[6] = 0x30;
	wate_time[7] = 0x30;
	wate_time[8] = 0x00;
	
}
//------------------------------------------------
//!!static inline 
//static 
void meas_bat(void){
	if(enable_battery_voltage){
uint32_t	val=((uint32_t)meas_ADC(7)*(uint32_t)battery_voltage_coefficient)/1000;
	//!!battery_voltage.current_value_bin=(battery_voltage.current_value_bin+(uint16_t)val)/2;
	battery_voltage_current_value_bin=(uint16_t)val;
	if(battery_voltage_current_value_bin>=999) battery_voltage_current_value_bin=999;
	
	battery_voltage_string_current_value[0] = (battery_voltage_current_value_bin/100);
	battery_voltage_string_current_value[1] = ((battery_voltage_current_value_bin-(battery_voltage_string_current_value[0]*100))/10);
	battery_voltage_string_current_value[2] = 0x2e;
	battery_voltage_string_current_value[3] = (battery_voltage_current_value_bin-((battery_voltage_string_current_value[0]*100)+(battery_voltage_string_current_value[1]*10)));
	battery_voltage_string_current_value[4] = 0x00;	

	battery_voltage_string_current_value[0] |=0x30;
	battery_voltage_string_current_value[1] |=0x30;
	battery_voltage_string_current_value[2] = 0x2e;
	battery_voltage_string_current_value[3] |=0x30;
	}
	else{
			battery_voltage_string_current_value[0] ='-';
			battery_voltage_string_current_value[1] ='-';
			battery_voltage_string_current_value[2] = 0x2e;
			battery_voltage_string_current_value[3] ='-';
			battery_voltage_string_current_value[4] = 0x00;
	}
}
//------------------------------------------------
//!!static inline
static void meas_temp(void){

// читаем температуру с датчика
	uint32_t tmp;
	uint8_t temperatur[2];

	therm_reset();
	
	while(!therm_read_bit());
	
	therm_reset();
	therm_write_byte(THERM_CMD_SKIPROM);
	therm_write_byte(THERM_CMD_RSCRATCHPAD);
	
	temperatur[0]=therm_read_byte();
	temperatur[1]=therm_read_byte();
	therm_reset();

	
	
	
	 tmp = ((uint32_t)temperatur[1]<<8)|temperatur[0];
	if(temperatur[1]&0xf8){
		temperature.sig=1;
		//tmp = ~tmp + 1;  
	}else{
		temperature.sig=0;
	}
//	
	if(temperature.sig)
	{
		//-  selected_x_bits_inverted = (x & ~mask) | (~x & mask);
		//temperature.current_value_bin = (temperature.current_value_bin^0xFFFF)+1;	//(temperature.current_value_bin & ~0xFFFF) | (~temperature.current_value_bin & 0xFFFF); 
		//temperature.current_value_bin = temperature.current_value_bin+1;
		tmp +=0xFFFF0000;
		tmp = ~tmp + 1;
		tmp*=THERM_DECIMAL_STEPS_12BIT;
		temperature.current_value_bin=tmp/1000;
		
		temperature.string_current_value[0] = (temperature.current_value_bin/100);
		temperature.string_current_value[1] = ((temperature.current_value_bin-(temperature.string_current_value[0]*100))/10);
		temperature.string_current_value[2] = 0x2e;
		temperature.string_current_value[3] = (temperature.current_value_bin-((temperature.string_current_value[0]*100)+(temperature.string_current_value[1]*10)));
		temperature.string_current_value[4] = 0xdf;		//0xb0;
		temperature.string_current_value[5] = 0x43;
		temperature.string_current_value[6] = 0x00;

		temperature.string_current_value[0] |= 0x30;
		temperature.string_current_value[1] |= 0x30;
		temperature.string_current_value[3] |= 0x30;
	}
	else
	{
		//+
			tmp*=THERM_DECIMAL_STEPS_12BIT;
			temperature.current_value_bin=tmp/1000;
			if(temperature.current_value_bin>=1000){
				if(temperature.sig==0){
					temperature.string_current_value[0] = 0x3E;
					temperature.string_current_value[1] = 0x31;
					temperature.string_current_value[2] = 0x30;
					temperature.string_current_value[3] = 0X30;
				}
				
				
				temperature.string_current_value[4]	= 0xdf;		//0xb0;
				temperature.string_current_value[5] = 0x43;
				temperature.string_current_value[6] = 0x00;
				
			}
			else{
				temperature.string_current_value[0] = (temperature.current_value_bin/100);
				temperature.string_current_value[1] = ((temperature.current_value_bin-(temperature.string_current_value[0]*100))/10);
				temperature.string_current_value[2] = 0x2e;
				temperature.string_current_value[3] = (temperature.current_value_bin-((temperature.string_current_value[0]*100)+(temperature.string_current_value[1]*10)));
				temperature.string_current_value[4] = 0xdf;		//0xb0;
				temperature.string_current_value[5] = 0x43;
				temperature.string_current_value[6] = 0x00;

				temperature.string_current_value[0] |= 0x30;
				temperature.string_current_value[1] |= 0x30;
				temperature.string_current_value[3] |= 0x30;
			}
		
	}
	
/*	
	tmp*=THERM_DECIMAL_STEPS_12BIT;
	temperature.current_value_bin=tmp/1000;
	if(temperature.current_value_bin>=1000){
		if(temperature.sig==0){
			temperature.string_current_value[0] = 0x3E;
			temperature.string_current_value[1] = 0x31;
			temperature.string_current_value[2] = 0x30;
			temperature.string_current_value[3] = 0X30;
		}
			
		
		temperature.string_current_value[4]	= 0xdf;		//0xb0;
		temperature.string_current_value[5] = 0x43;
		temperature.string_current_value[6] = 0x00;
			
	}
	else{
		temperature.string_current_value[0] = (temperature.current_value_bin/100);
		temperature.string_current_value[1] = ((temperature.current_value_bin-(temperature.string_current_value[0]*100))/10);
		temperature.string_current_value[2] = 0x2e;
		temperature.string_current_value[3] = (temperature.current_value_bin-((temperature.string_current_value[0]*100)+(temperature.string_current_value[1]*10)));
		temperature.string_current_value[4] = 0xdf;		//0xb0;
		temperature.string_current_value[5] = 0x43;
		temperature.string_current_value[6] = 0x00;

		temperature.string_current_value[0] |= 0x30;
		temperature.string_current_value[1] |= 0x30;
		temperature.string_current_value[3] |= 0x30;
	}
*/




	
	therm_write_byte(THERM_CMD_SKIPROM);
	therm_write_byte(THERM_CMD_CONVERTTEMP);
	
	
	if((temperature.enable_low==0)&&(temperature.enable_hight==0))	{
		temperature.string_current_value[0] = '-';
		temperature.string_current_value[1] = '-';
		temperature.string_current_value[2] = 0x2e;
		temperature.string_current_value[3] = '-';
		temperature.string_current_value[4] = 0xdf;		//0xb0;
		temperature.string_current_value[5] = 0x43;
		temperature.string_current_value[6] = 0x00;
	}
	
}
//------------------------------------------------------------------------------------------------------------------
//1 - temperature OK 0 - not OK
uint8_t test_temperature(void){
//--------------------------------------------------------------------------
if((temperature.enable_low==0)&&(temperature.enable_hight==0)){
	return 1;
}
	
//--------------------------------------------------------------------------		
if((temperature.enable_low==1)&&(temperature.enable_hight==0)){	
	if(temperature.sig==1) return 0;
	if (temperature.current_value_bin>temperature.setp_value_low_bin)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
//--------------------------------------------------------------------------
if((temperature.enable_low==0)&&(temperature.enable_hight==1)){	
	if(temperature.current_value_bin<temperature.setp_value_hight_bin)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
//--------------------------------------------------------------------------
if((temperature.enable_low==1)&&(temperature.enable_hight==1)){


if(temperature.current_value_bin<temperature.setp_value_hight_bin)
{
	if (temperature.current_value_bin>temperature.setp_value_low_bin)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}else return 0;	
}


	
//	
return 1;	
/*	
if(temperature.sig==1) return 0;
if(temperature.current_value_bin<temperature.setp_value_hight_bin)
{
	if (temperature.current_value_bin>temperature.setp_value_low_bin)
	{
		return 1;
	} 
	else
	{
		return 0;
	}
}else return 0;	
*/
}
//------------------------------------------------------------------------------------------------------------------
//1 - voltage OK 0 - not OK
uint8_t test_battery(void){
if(enable_battery_voltage==0) return 1;
if(battery_voltage_current_value_bin<battery_voltage_setp_value_low_bin) return 0;
else return 1;
}

//------------------------------------------------------------------------------------------------------------------
/*
 * Initialize the UART to 9600 Bd, tx/rx, 8N1.
 */
void uart_init(void)
{
#if F_CPU < 2000000UL && defined(U2X)
  UCSRA = _BV(U2X);             /* improve baud rate error by using 2x clk */
  UBRRL = (F_CPU / (8UL * UART_BAUD)) - 1;
#else
  UBRRL = (F_CPU / (16UL * UART_BAUD)) - 1;
#endif
  UCSRB = _BV(TXEN) | _BV(RXEN); /* tx/rx enable */
 // UCSRC = (1<<URSEL)|(3<<UCSZ0);
}
//------------------------------------------------------------------------------------------------------------------
static void ioinit(void)
{
	DDRA=0x00;
	PORTA=0x1f;
	DDRB=0x80;
	PORTB=0x00;
	DDRD=0x7f;
	PORTD=0x00;
	DDRC=0x70;
	PORTC=0x00;
}

//FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
FILE lcd_str = FDEV_SETUP_STREAM(lcd_putchar, NULL, _FDEV_SETUP_WRITE);


int main(void)
{

wdt_reset ();
wdt_enable( WDTO_2S);

	ioinit();

//!!
	uart_init();
//!!
	lcd_init();
//!!

	RTC_init();
wdt_reset ();	
	init_ADC();
wdt_reset ();
	read_eeprom_setp();
wdt_reset ();	
//----- test calibration ---------------------------	
	flag_calibration=(PINA&0x1f);
	if(flag_calibration==_left)
	{
		_delay_ms(200);
		wdt_reset ();
		flag_calibration=(PINA&0x1f);
		
		if (flag_calibration==_left)
		{
			flag_calibration=1;
		}
		else
		{
			flag_calibration=0;
		}
	}
	else
	{
		flag_calibration=0;
	}
//----------------------------------------------------
 stdout =   stderr = &lcd_str;
//===========================================
ON_LCD_BACKLIGHT;
//bild welcome
	lcd_reset();
	clr_blink();
	lcd_1_strin();
	printf_P(PSTR("GTS ELECTRONICS "));
	lcd_2_strin();
	printf_P(PSTR("  gts-el.com    "));
	//lcd_1_strin();
	wdt_reset ();
	_delay_ms(1000);
	wdt_reset ();
	_delay_ms(1000);
	wdt_reset ();
//	clr_lcd();
//	lcd_1_strin();
//	printf_P(PSTR("GTS ELECTRONICS "));
	lcd_2_strin();
	printf_P(PSTR("   Ver. 01.20   "));
	lcd_1_strin();
	_delay_ms(1000);
wdt_reset ();
//===========================================
	meas_bat();
	meas_temp();
	_delay_ms(1000);
wdt_reset ();	
	meas_bat();	
	meas_temp();

	if(type_generator==1){
		ON_RELAY_2;
		}else{
		OFF_RELAY_2;
	}
	

//----- test calibration ---------------------------
if(flag_calibration==1)
{
	flag_calibration=(PINA&0x1f);
	if(flag_calibration==_left)
	{
		flag_calibration=1;
	}
	else
	{
		flag_calibration=0;
	}
}

//----------------------------------------------------	
if (flag_calibration==1)
{
	state=CALIBRATION;
	
} 
else
{
	state=WAIT;
	start_timer_wait();
}

	//clr_key();	
	clr_key();
	lcd_reset();
	
	sei();

	
//--------------------------------------------------------------------
  for (;;)
    {
//######################################################################################		
wdt_reset ();		
//####################### meas temp and U bat ##########################################	
if(flag_meas_bat==1){
	meas_bat();			
	flag_meas_bat=0;
}
if(flag_meas_temp==1){
	meas_temp();
	flag_meas_temp=0;
}
//################ test RELAY  #########################################################
if((state==WORK)||(state==MANUALLY_STARTED))
{
	if(type_generator==1){
			OFF_RELAY_2;	
	}else{
			ON_RELAY_2;	
	}
}
else
{
	if(type_generator==1){
			ON_RELAY_2;
	}else{
			OFF_RELAY_2;
	}
}
//################ test KEY ############################################################
	key=test_key();
//######################################################################################
if(state==WAIT)
{
	//uint8_t	ac=PINA&0x20;	//0-AC ON    0x20 - AC OFF 
 	if(PINA&0x20){
	 	  build_windou_mains_off();
	 	}else{
	 	  build_windou_mains_on();
 	}
 
//---------- test KEY ------------------------------------
if(key== _right){
	state=SETING;
	clr_key();
} 
// logic waite -----------------------------------------------------------------
if(time_rest_bat.status==EVENT) time_rest_bat.status=STOPPED;
if(time_rest_temperat.status==EVENT) time_rest_temperat.status=STOPPED;
// Test AC
if(PINA&0x20){
	//AC OFF
	//Test Battery
	//if(battery_voltage_current_value_bin<battery_voltage_setp_value_low_bin)
	if(test_battery()==0)
	{
		//Battery NOT OK
		if(test_temperature())	//if((temperature.sig==1)||(temperature.current_value_bin<temperature.setp_value_hight_bin))
		{
			//Temperature OK    Battery NOT OK
			if( test_runing_rest_timers()){
				state=REST;
				}else{
				start_timer_run_bat();
				state=WORK;
			}
			}else{
			//Temperature NOT OK  Battery NOT OK
			if( test_runing_rest_timers()){
				state=REST;
				}else{
				start_timer_run_bat();
				start_timer_run_temperat();
				state=WORK;
			}
		}
		
		}else{
		//Battery OK
		if(test_temperature())	//if((temperature.sig==1)||(temperature.current_value_bin<temperature.setp_value_hight_bin))
		{
			//Temperature OK   Battery OK
//			state=WAIT;
//			start_timer_wait();
			}else{
			//Temperature NOT OK   Battery OK
			if( test_runing_rest_timers()){
				state=REST;
				}else{
				start_timer_run_temperat();
				state=WORK;
			}
		}
	}
} 
 
 
}else
//############################################################
if(state==WORK)
{
	  build_windou_work();
	  //	  clr_blink();
	  //----------- test AC ------------------------------------
	  if(!(PINA&0x20)){
		  state=WAIT;
		  start_timer_wait();
		  clr_key();
	  }
	
//logic WORK -------------------------------------------------------------------
	// Test AC
	if(PINA&0x20){
		//AC OFF
		if(time_work_bat.status==STOPPED){
			//timer_work_bat STOPPED
			if(time_work_temperat.status==STOPPED){
				//timer_work_bat STOPPED    timer_run_temperat  STOPPED  ERROR
				time_work_bat.status=STOPPED;
				time_work_temperat.status=STOPPED;
				state=WAIT;
				start_timer_wait();
				}else if(time_work_temperat.status==RUNING){
				//timer_work_bat STOPPED    time_work_temperat   RUNING
				state=WORK;
				}else if(time_work_temperat.status==EVENT){
				//timer_work_bat STOPPED    time_work_temperat   EVENT
				//Test Temperature
				if(test_temperature())	//if((temperature.sig==1)||(temperature.current_value_bin<temperature.setp_value_hight_bin))
				{
					//Temperature OK test battery
					//if(battery_voltage_current_value_bin<battery_voltage_setp_value_low_bin){
					if(test_battery()==0){
						//battery NOT OK
						start_timer_run_bat();
						state=WORK;
						}else{
						//battery OK
						start_timer_rest_temperat();
						time_work_bat.status=STOPPED;
						time_work_temperat.status=STOPPED;
						state=REST;
					}
					}else{
					//Temperature NOT OK restart timer
					start_timer_run_temperat();
					state=WORK;
				}
				}else{
				//error
			}
		
			}else if(time_work_bat.status==RUNING){
			state=WORK;
			}else if(time_work_bat.status==EVENT){
					if(time_work_temperat.status==STOPPED){
						//timer_run_bat EVET    timer_run_temperat  STOPPED
						//Test Battery
						//if(battery_voltage_current_value_bin<battery_voltage_setp_value_low_bin)
						if(test_battery()==0)
						{
							//battery NOT OK
							start_timer_run_bat();
							state=WORK;
						}else{
								//battery OK
								//Test Temperature
								if(test_temperature())	//if((temperature.sig==1)||(temperature.current_value_bin<temperature.setp_value_hight_bin))
								{
									//temperat OK
									start_timer_rest_bat();
									time_work_bat.status=STOPPED;
									time_work_temperat.status=STOPPED;
									state=REST;
								}else{
										//temperat NOT OK
										start_timer_run_temperat();
										state=WORK;
									}
							}
				}else if(time_work_temperat.status==RUNING){
						//timer_run_bat EVENT    time_run_temperat   RUNING
						state=WORK;
						}else if(time_work_temperat.status==EVENT)
								{
								//timer_run_bat EVENT    time_run_temperat   EVENT
								//Test Battery
								//if(battery_voltage_current_value_bin<battery_voltage_setp_value_low_bin){
								if(test_battery()==0){	
									//battery NOT OK
									start_timer_run_bat();
									state=WORK;
								}else{
										//battery OK
										//Test Temperature
										if(test_temperature())	//if((temperature.sig==1)||(temperature.current_value_bin<temperature.setp_value_hight_bin))
										{
											//temperat OK
											start_timer_rest_bat();
											start_timer_rest_temperat();
											time_work_bat.status=STOPPED;
											time_work_temperat.status=STOPPED;
											state=REST;
										}else{
												//temperat NOT OK
												start_timer_run_temperat();
												state=WORK;
											}
									}
								}else{
										//error
									 }
						}else{
								//error
							 }
					}else{
							time_work_bat.status=STOPPED;
							time_work_temperat.status=STOPPED;
							state=WAIT;
							start_timer_wait();
						}

	  //---------- test KEY ------------------------------------
	  if(key== _right){
		  state=SETING;
		  clr_key();
	  }	
}else
//############################################################
if(state==REST)
{
	  build_windou_rest();
	  //	  clr_blink();
 //----------- test AC ------------------------------------
	  if(!(PINA&0x20)){
		  state=WAIT;
		  start_timer_wait();
		  clr_key();
	  }
 

//lognc REST -------------------------------------------------------------------
	// Test AC
	if(PINA&0x20){
		//AC OFF
		if(time_rest_bat.status==STOPPED){
			if(time_rest_temperat.status==STOPPED){
				state=WAIT;
				start_timer_wait();
				}else if(time_rest_temperat.status==RUNING){
				state=REST;
				}else if(time_rest_temperat.status==EVENT){
				state=WAIT;
				start_timer_wait();
				}else{
				//error
					}
		
			}else if(time_rest_bat.status==RUNING){
						state=REST;
					}else if(time_rest_bat.status==EVENT){
								if(time_rest_temperat.status==STOPPED){
								state=WAIT;
								start_timer_wait();
							}else if(time_rest_temperat.status==RUNING){
										state=REST;
									}else if(time_rest_temperat.status==EVENT){
												state=WAIT;
												start_timer_wait();
											}else{
													//error
												}
								}else{
										//error
									}
						}else{
								state=WAIT;
								start_timer_wait();
								}
								
	//---------- test KEY ------------------------------------
	if(key== _right){
		 state=SETING;
		 clr_key();
	}									
}else
//############################################################
if(state==SETING)
{
	  build_menu_setings();
	  //	  clr_blink();
	  
	  //---------- test KEY ------------------------------------
	  if(key== _right){
		  state=VOLTAGE_TIMER;
		  clr_key();
	  }else
	   if(key== _up){
		  state=CONTROL;
		  clr_key();
	  }else
	  if(key==_down){
		  state=MANUAL_START;
		  clr_key();
	  }else
	  if(key==_left){
		 state=WAIT;
		 start_timer_wait();
		 clr_key(); 
	  }
	
}else
//############################################################
if(state==CONTROL)
{
	build_menu_control();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		state=WAIT;
		start_timer_wait();
		clr_key();
	}else
	if(key== _up){
		state=MANUAL_START;
		clr_key();
	}else
	if(key==_down){
		state=SETING;
		clr_key();
	}else
	if(key==_left){
		state=WAIT;
		start_timer_wait();
		clr_key();
	}
	
}else
//############################################################
if(state==MANUAL_START)
{
	build_menu_manual_start();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		state=MANUALLY_STARTED;
		clr_key();
	}else
	if(key== _up){
		state=SETING;
		clr_key();
	}else
	if(key==_down){
		state=CONTROL;
		clr_key();
	}else
	if(key==_left){
		state=WAIT;
		start_timer_wait();
		clr_key();
	}
	
}else
//############################################################
//############################################################
if(state==VOLTAGE_TIMER)
{
	build_menu_voltage_timer();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		if (enable_battery_voltage==1)
		{
			state=VOLTAGE_WORKING_TIME;
			build_menu_voltage_working_time();
			index_cursor=0;
			blinc_cursor(index_cursor);
			clr_key();	
		}
		else
		{
			build_menu_setpoint_disabled();
//			_delay_ms(1000);			
		}
	}else
	if(key== _up){
		state=ENABLE_DISABLE;
		clr_key();
	}else
	if(key==_down){
		state=TEMPERATURE_TIMER;
		clr_key();
	}else
	if(key==_left){
		state=SETING;
		clr_key();
	}
	
}else
//############################################################
if(state==TEMPERATURE_TIMER)
{
	build_menu_temperature_timer();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		if ((temperature.enable_low==0)&&(temperature.enable_hight==0))
		{
			build_menu_setpoint_disabled();
//			_delay_ms(1000);
		}
		else
		{
			state=TEMPERATURE_WORKING_TIME;
			build_menu_temperature_working_time();
			index_cursor=0;
			blinc_cursor(index_cursor);
			clr_key();
		}
		
	}else
	if(key== _up){
		state=VOLTAGE_TIMER;
		clr_key();
	}else
	if(key==_down){
		state=GENERATOR;	//BATTERY_VOLTAGE;	
		clr_key();
	}else
	if(key==_left){
		state=SETING;
		clr_key();
	}
	
}else
//############################################################
if(state==GENERATOR)
{
	build_menu_generator();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		state=GENERATOR_TYPE;
		build_menu_generator_type();
		if (type_generator==0)
		{
			index_cursor=0;
		} 
		else
		{
			index_cursor=10;
		}
		blinc_cursor(index_cursor);
		clr_key();
	}else
	if(key== _up){
		state=TEMPERATURE_TIMER;
		clr_key();
	}else
	if(key==_down){
		state=BATTERY_VOLTAGE;
		clr_key();
	}else
	if(key==_left){
		state=SETING;
		clr_key();
	}
	
}else
//############################################################
if(state==BATTERY_VOLTAGE)
{

		build_menu_battery_voltage();
		//	  clr_blink();
	
		//---------- test KEY ------------------------------------
		if(key== _right){
			if(enable_battery_voltage){
				state=BATTERY_LOWER_LIMIT;
				build_menu_battery_lower_limit();
				index_cursor=0;
				blinc_cursor(index_cursor);
				clr_key();
			}
			else
			{
				build_menu_setpoint_disabled();
//				_delay_ms(1000);
			}
		}else
		if(key== _up){
			state=GENERATOR;	//TEMPERATURE_TIMER;
			clr_key();
		}else
		if(key==_down){
			state=TEMPERATURE_LOW;
			clr_key();
		}else
		if(key==_left){
			state=SETING;
			clr_key();
		}
	

	
}else
//############################################################
if(state==TEMPERATURE_LOW)
{
	build_menu_temperature_low();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		if(temperature.enable_low)
		{
			state=TEMPERATURE_LOWER_LIMIT;
			build_menu_temperature_lower_limit();
			index_cursor=0;
			blinc_cursor(index_cursor);
			clr_key();
		}
		else
		{
				build_menu_setpoint_disabled();
//				_delay_ms(1000);
		}
	}else
	if(key== _up){
		state=BATTERY_VOLTAGE;
		clr_key();
	}else
	if(key==_down){
		state=TEMPERATURE_HIGH;
		clr_key();
	}else
	if(key==_left){
		state=SETING;
		clr_key();
	}
	
}else
//############################################################
if(state==TEMPERATURE_HIGH)
{
	build_menu_temperature_hight();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		if(temperature.enable_hight)
		{
			state=TEMPERATURE_HIGH_LIMIT;
			build_menu_temperature_high_limit();
			index_cursor=0;
			blinc_cursor(index_cursor);
			clr_key();
		}
		else
		{
				build_menu_setpoint_disabled();
//				_delay_ms(1000);			
		}
	}else
	if(key== _up){
		state=TEMPERATURE_LOW;
		clr_key();
	}else
	if(key==_down){
		state=ENABLE_DISABLE;
		clr_key();
	}else
	if(key==_left){
		state=SETING;
		clr_key();
	}
	
}else
//############################################################
if(state==ENABLE_DISABLE)
{
	build_menu_enable_disable();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		
			state=ENABLE_DISABLE_SETPOINT_BAT_VOLTAGE;
			//build_menu_enable_disable_setpoint_bat_voltage();
			index_cursor=0;
			blinc_cursor(index_cursor);
			clr_key();
		
	}else
	if(key== _up){
		state=TEMPERATURE_HIGH;
		clr_key();
	}else
	if(key==_down){
		state=VOLTAGE_TIMER;
		clr_key();
	}else
	if(key==_left){
		state=SETING;
		clr_key();
	}	
	
	
}else
//############################################################
//############################################################
if(state==VOLTAGE_WORKING_TIME)
{
	//build_menu_voltage_working_time();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
uint8_t k=control_key_time(&time_work_bat.string_setp_time[0]);

	if (k==1)
	{
		state=VOLTAGE_TIMER;
		clr_blink();
	}
	if (k==2)
	{
		state=VOLTAGE_RESTING_TIME;
		index_cursor=0;
		build_menu_voltage_resting_time();
		blinc_cursor(index_cursor);
	}
	if(k==3){
		build_menu_voltage_working_time();
		blinc_cursor(index_cursor);
	}
	
}else
//############################################################
if(state==VOLTAGE_RESTING_TIME)
{
	//build_menu_voltage_resting_time();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
uint8_t	k=control_key_time(&time_rest_bat.string_setp_time[0]);
	if (k==1)
	{
		state=VOLTAGE_WORKING_TIME;
		build_menu_voltage_working_time();
		index_cursor=4;
		//clr_blink();
		blinc_cursor(index_cursor);
		
	}
	if (k==2)
	{
		state=VOLTAGE_WORKING_TIME;
		build_menu_voltage_working_time();
		index_cursor=0;
		//clr_blink();
		blinc_cursor(index_cursor);
		
	}
	if(k==3){
		build_menu_voltage_resting_time();
		blinc_cursor(index_cursor);
	}
}else
//############################################################
if(state==TEMPERATURE_WORKING_TIME)
{
	//build_menu_temperature_working_time();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
uint8_t	k=control_key_time(&time_work_temperat.string_setp_time[0]);
		if (k==1)
		{
			state=TEMPERATURE_TIMER;
			clr_blink();
		}
		if (k==2)
		{
			state=TEMPERATURE_RESTING_TIME;
			build_menu_temperature_resting_time();
			index_cursor=0;
			//clr_blink();
			blinc_cursor(index_cursor);
		}
		if(k==3){
			build_menu_temperature_working_time();
			blinc_cursor(index_cursor);
		}
		
		
}else
//############################################################
if(state==TEMPERATURE_RESTING_TIME)
{
	//build_menu_temperature_resting_time();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
uint8_t k =	control_key_time(&time_rest_temperat.string_setp_time[0]);
		if (k==1)
		{
			state=TEMPERATURE_WORKING_TIME;
			build_menu_temperature_working_time();
			index_cursor=4;
			//clr_blink();
			blinc_cursor(index_cursor);
		}
		if (k==2)
		{
			state=TEMPERATURE_WORKING_TIME;
			build_menu_temperature_working_time();
			index_cursor=0;
			//clr_blink();
			blinc_cursor(index_cursor);
			
		}
		if(k==3){
			build_menu_temperature_resting_time();
			blinc_cursor(index_cursor);
		}
}else
//############################################################
if(state==GENERATOR_TYPE)
{
	//build_menu_generator_type();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		if(index_cursor==0) index_cursor=10 ;
		build_menu_generator_type();
		blinc_cursor(index_cursor);
		
	}else
	if(key== _up){
		
	}else
	if(key==_down){
		if(index_cursor==0){
			type_generator=0;
		}
		if (index_cursor==10)
		{
			type_generator=1;
		}
		store_setp();
		read_eeprom_setp();
		clr_blink();
		build_windou_stored();
		wdt_reset ();
		_delay_ms(1000);
		wdt_reset ();
		build_menu_generator_type();
		blinc_cursor(index_cursor);
	}else
	if(key==_left){
		if(index_cursor==0){
			state=GENERATOR;
			clr_blink();
		}
		if (index_cursor==10)
		{
			index_cursor=0;
			build_menu_generator_type();
			blinc_cursor(index_cursor);
		}
	}
	
}else
//############################################################
if(state==BATTERY_LOWER_LIMIT)
{
	//build_menu_battery_lower_limit();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
uint8_t k =	control_key_val(&battery_voltage_string_setp_value_low[0]);
		if (k==1)
		{
			state=BATTERY_VOLTAGE;
			clr_blink();
			
		}
		if (k==2)
		{
			state=BATTERY_LOWER_LIMIT;
			build_menu_battery_lower_limit();
			//clr_blink();
			index_cursor=0;
			blinc_cursor(index_cursor);
		}
		if(k==3)
		{
			build_menu_battery_lower_limit();
			blinc_cursor(index_cursor);
		}
}else
//############################################################
if(state==TEMPERATURE_LOWER_LIMIT)
{
	//build_menu_temperature_lower_limit();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
uint8_t k =	control_key_val(&temperature.string_setp_value_low[0]);
		if (k==1)
		{
			state=TEMPERATURE_LOW;
			clr_blink();
			
		}
		if (k==2)
		{
			
			build_menu_temperature_lower_limit();
			index_cursor=0;
			//clr_blink();
			blinc_cursor(index_cursor);
		}
		if(k==3)
		{
			build_menu_temperature_lower_limit();
			blinc_cursor(index_cursor);
		}
}else
//############################################################
if(state==TEMPERATURE_HIGH_LIMIT)
{
	//build_menu_temperature_high_limit();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
uint8_t k =	control_key_val(&temperature.string_setp_value_hight[0]);
		if (k==1)
		{
			state=TEMPERATURE_HIGH;
			//build_menu_temperature_lower_limit();
			//index_cursor=3;
			//clr_blink();
			//blinc_cursor(index_cursor);
		/*	state=TEMPERATURE;
			clr_blink();*/
		}
		if (k==2)
		{
			build_menu_temperature_high_limit();
			index_cursor=0;
			//clr_blink();
			blinc_cursor(index_cursor);
		}
		if(k==3)
		{
			build_menu_temperature_high_limit();
			blinc_cursor(index_cursor);
		}
}else
//############################################################
if(state==ENABLE_DISABLE_SETPOINT_BAT_VOLTAGE)
{
	
	build_menu_enable_disable_setpoint_bat_voltage();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		
		state=ENABLE_DISABLE_SETPOINT_TEMP_LOW;
		
		//index_cursor=0;
		//blinc_cursor(index_cursor);
		clr_key();
		
	}else
	if(key== _up){
		enable_battery_voltage=1;
		clr_key();
	}else
	if(key==_down){
		enable_battery_voltage=0;
		clr_key();
	}else
	if(key==_left){
		state=ENABLE_DISABLE_SETPOINT_STORED;
		clr_key();
	}

}else
//############################################################
if(state==ENABLE_DISABLE_SETPOINT_TEMP_LOW)
{
	
	build_menu_enable_disable_setpoint_temp_low();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		
		state=ENABLE_DISABLE_SETPOINT_TEMP_HIGHT;
		
		//index_cursor=0;
		//blinc_cursor(index_cursor);
		clr_key();
		
	}else
	if(key== _up){
		temperature.enable_low=1;
		clr_key();
	}else
	if(key==_down){
		temperature.enable_low=0;
		clr_key();
	}else
	if(key==_left){
		state=ENABLE_DISABLE_SETPOINT_BAT_VOLTAGE;
		clr_key();
	}

}else
//############################################################
if(state==ENABLE_DISABLE_SETPOINT_TEMP_HIGHT)
{
	
	build_menu_enable_disable_setpoint_temp_hight();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		
		state=ENABLE_DISABLE_SETPOINT_STORED;
		clr_key();
		
	}else
	if(key== _up){
		temperature.enable_hight=1;
		clr_key();
	}else
	if(key==_down){
		temperature.enable_hight=0;
		clr_key();
	}else
	if(key==_left){
		state=ENABLE_DISABLE_SETPOINT_TEMP_LOW;
		clr_key();
	}

}else
//############################################################
if(state==ENABLE_DISABLE_SETPOINT_STORED)
{
	
	build_menu_enable_disable_setpoint_stored();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		read_eeprom_setp();
		state=ENABLE_DISABLE;
		clr_key();
		
	}else
	if(key== _up){
		read_eeprom_setp();
		state=ENABLE_DISABLE;
		clr_key();
	}else
	if(key==_down){
							store_setp();
							read_eeprom_setp();
							clr_blink();
							build_windou_stored();
							wdt_reset ();
							_delay_ms(1000);
							wdt_reset ();
		state=ENABLE_DISABLE;					
		clr_key();
	}else
	if(key==_left){
		read_eeprom_setp();
		state=ENABLE_DISABLE;
		clr_key();
	}

}else
//############################################################
//############################################################
if(state==MANUALLY_STARTED)
{
	build_menu_manually_started();
	//	  clr_blink();
	
	//---------- test KEY ------------------------------------
	if(key== _right){
		state=WAIT;
		start_timer_wait();
		clr_key();
	}else
	if(key== _up){
		state=WAIT;
		start_timer_wait();
		clr_key();
	}else
	if(key==_down){
		state=WAIT;
		start_timer_wait();
		clr_key();
	}else
	if(key==_left){
		state=WAIT;
		start_timer_wait();
		clr_key();
	}
	
}else
//#### CALIBRATION ###########################################
if(state==CALIBRATION)
{
	build_windou_calobration();

	
	//lcd_1_strin();
		//---------- test KEY ------------------------------------
		if(key== _right){
			//stored
			store_setp();
			read_eeprom_setp();
			clr_blink();
			build_windou_stored();
			wdt_reset ();
			_delay_ms(1000);
			wdt_reset ();
			//-------			
			state=WAIT;
			start_timer_wait();
			clr_key();
		}else
		if(key== _up){
			if(mul==1){
				battery_voltage_coefficient++;
				}else if(mul==10){
					battery_voltage_coefficient+=10;
				}else if(mul==100){
					battery_voltage_coefficient+=100;
				}else{
					mul=1;
				}
			if(battery_voltage_coefficient>2000) battery_voltage_coefficient=2000;
			clr_key();
		}else
		if(key==_down){
			if(battery_voltage_coefficient!=0){
				if(mul==1){
					battery_voltage_coefficient--;
				}else if(mul==10){
						if(battery_voltage_coefficient>=10){
							battery_voltage_coefficient-=10;
						}
					}else if(mul==100){
						if(battery_voltage_coefficient>=100){
							battery_voltage_coefficient=battery_voltage_coefficient-100;
						}
					}else{
						mul=1;
					}
				
			}
			clr_key();
		}else
		if(key==_left){
		if(mul==1){
			mul=10;
		}else if(mul==10){
			mul=100;
		}else if(mul==100){
			mul=1;
		}else{
			mul=1;
		}
			clr_key();
		}
	
	
	
	
}
//##########################################################
else
{
	state=WAIT;
	start_timer_wait();
}
//##########################################################

  }

}


