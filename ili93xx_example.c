#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "smc.h"
#include "arm_math.h"

/** Chip select number to be set */
#define ILI93XX_LCD_CS      1

struct ili93xx_opt_t g_ili93xx_display_opt;
#define pi 3.14926
#define PIN_PUSHBUTTON_1_MASK	PIO_PB3
#define PIN_PUSHBUTTON_1_PIO	PIOB
#define PIN_PUSHBUTTON_1_ID		ID_PIOB
#define PIN_PUSHBUTTON_1_TYPE	PIO_INPUT
#define PIN_PUSHBUTTON_1_ATTR	PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_FALL_EDGE

#define PIN_PUSHBUTTON_2_MASK	PIO_PC12
#define PIN_PUSHBUTTON_2_PIO	PIOC
#define PIN_PUSHBUTTON_2_ID		ID_PIOC
#define PIN_PUSHBUTTON_2_TYPE	PIO_INPUT
#define PIN_PUSHBUTTON_2_ATTR	PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_FALL_EDGE

/** Size of the receive buffer and transmit buffer. */
#define BUFFER_SIZE     (100)
/** Reference voltage for ADC,in mv. */
#define VOLT_REF        (3300)
/* Tracking Time*/
#define TRACKING_TIME    1
/* Transfer Period */
#define TRANSFER_PERIOD  1
/* Startup Time*/
#define STARTUP_TIME ADC_STARTUP_TIME_4

/** The maximal digital value */
#define MAX_DIGITAL     (4095)

/** adc buffer */
static int16_t gs_s_adc_values[BUFFER_SIZE] = { 0 };

#define ADC_POT_CHANNEL 5

/************************************************************************/
/* GLOBAL                                                                */
/************************************************************************/
int adc_value_old;
float v;
int vet[30];
float rad;

/************************************************************************/
/* HANDLER                                                              */
/************************************************************************/

static void push_button_handle(uint32_t id, uint32_t mask)
{
	adc_start(ADC);
}


/**
* \brief ADC interrupt handler.
*/
void ADC_Handler(void)
{
	uint32_t tmp;
	uint32_t status ;

	status = adc_get_status(ADC);
	
	/* Checa se a interrupção é devido ao canal 5 */
	static float rad_antes = 0;
	if ((status & ADC_ISR_EOC5)) {
		tmp = adc_get_channel_value(ADC, ADC_POT_CHANNEL);

			ili93xx_set_foreground_color(COLOR_WHITE);
			ili93xx_draw_filled_rectangle(9, 39, ILI93XX_LCD_WIDTH,55);
			v=3.3*((float)tmp)/4095.0;
			rad=2*pi*((float)tmp)/4095.0;
			ili93xx_draw_line(120,160,120+54*arm_cos_f32(rad_antes),160+54*arm_sin_f32(rad_antes));
			ili93xx_set_foreground_color(COLOR_BLACK);
			sprintf(vet, "Tensao: %f", v);
			ili93xx_draw_string(10, 40, vet);
			ili93xx_draw_line(120,160,120+54*arm_cos_f32(rad),160+54*arm_sin_f32(rad));
			rad_antes = rad;

	}
	

	
}

void TC0_Handler(void)
{	static b = 0;
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC0,0);
	adc_start(ADC);

}
/************************************************************************/
/* CONFIGs                                                              */
/************************************************************************/

void configure_lcd()
{
	/** Enable peripheral clock */
	pmc_enable_periph_clk(ID_SMC);

	/** Configure SMC interface for Lcd */
	smc_set_setup_timing(SMC, ILI93XX_LCD_CS, SMC_SETUP_NWE_SETUP(2)
	| SMC_SETUP_NCS_WR_SETUP(2)
	| SMC_SETUP_NRD_SETUP(2)
	| SMC_SETUP_NCS_RD_SETUP(2));
	smc_set_pulse_timing(SMC, ILI93XX_LCD_CS, SMC_PULSE_NWE_PULSE(4)
	| SMC_PULSE_NCS_WR_PULSE(4)
	| SMC_PULSE_NRD_PULSE(10)
	| SMC_PULSE_NCS_RD_PULSE(10));
	smc_set_cycle_timing(SMC, ILI93XX_LCD_CS, SMC_CYCLE_NWE_CYCLE(10)
	| SMC_CYCLE_NRD_CYCLE(22));
	#if ((!defined(SAM4S)) && (!defined(SAM4E)))
	smc_set_mode(SMC, ILI93XX_LCD_CS, SMC_MODE_READ_MODE
	| SMC_MODE_WRITE_MODE
	| SMC_MODE_DBW_8_BIT);
	#else
	smc_set_mode(SMC, ILI93XX_LCD_CS, SMC_MODE_READ_MODE
	| SMC_MODE_WRITE_MODE);
	#endif
	/** Initialize display parameter */
	g_ili93xx_display_opt.ul_width = ILI93XX_LCD_WIDTH;
	g_ili93xx_display_opt.ul_height = ILI93XX_LCD_HEIGHT;
	g_ili93xx_display_opt.foreground_color = COLOR_BLACK;
	g_ili93xx_display_opt.background_color = COLOR_WHITE;

	/** Switch off backlight */
	aat31xx_disable_backlight();

	/** Initialize LCD */
	ili93xx_init(&g_ili93xx_display_opt);

	/** Set backlight level */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);

	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(0, 0, ILI93XX_LCD_WIDTH,
	ILI93XX_LCD_HEIGHT);
	/** Turn on LCD */
	ili93xx_display_on();
	ili93xx_set_cursor_position(0, 0);
}

void configure_botao(void)
{
	pmc_enable_periph_clk(PIN_PUSHBUTTON_1_ID);
	pio_set_input(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_MASK, PIN_PUSHBUTTON_1_ATTR);
	pio_set_debounce_filter(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_MASK, 10);
	pio_handler_set(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_ID,PIN_PUSHBUTTON_1_MASK, PIN_PUSHBUTTON_1_ATTR ,push_button_handle);
	pio_enable_interrupt(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_MASK);
	NVIC_SetPriority((IRQn_Type) PIN_PUSHBUTTON_1_ID, 0);
	NVIC_EnableIRQ((IRQn_Type) PIN_PUSHBUTTON_1_ID);
}

static void configure_tc(void)
{
	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	
	// [main_tc_configure]
	/*
	* Aqui atualizamos o clock da cpu que foi configurado em sysclk init
	*
	* O valor atual est'a em : 120_000_000 Hz (120Mhz)
	*/
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	
	/****************************************************************
	* Ativa o clock do periférico TC 0
	*****************************************************************
	*
	* Parametros :
	*  1 - ID do periferico
	*
	*
	*****************************************************************/
	pmc_enable_periph_clk(ID_TC0);

	/*****************************************************************
	* Configura TC para operar no modo de comparação e trigger RC
	*****************************************************************
	*
	* Configura TC para operar no modo de comparação e trigger RC
	* devemos nos preocupar com o clock em que o TC irá operar !
	*
	* Cada TC possui 3 canais, escolher um para utilizar.
	*
	* No nosso caso :
	
	*
	*	MCK		= 120_000_000
	*	SLCK	= 32_768		(rtc)
	*
	* Uma opção para achar o valor do divisor é utilizar a funcao, como ela
	* funciona ?
	* tc_find_mck_divisor()
	*
	*
	* Parametros
	*   1 - TC a ser configurado (TC0,TC1, ...)
	*   2 - Canal a ser configurado (0,1,2)
	*   3 - Configurações do TC :
	*
	*	* Configurações de modo de operação :
	*	    TC_CMR_ABETRG  : TIOA or TIOB External Trigger Selection
	*	    TC_CMR_CPCTRG  : RC Compare Trigger Enable
	*	    TC_CMR_WAVE    : Waveform Mode
	*
	*     Configurações de clock :
	*	    TC_CMR_TCCLKS_TIMER_CLOCK1 : Clock selected: internal MCK/2 clock signal
	*	    TC_CMR_TCCLKS_TIMER_CLOCK2 : Clock selected: internal MCK/8 clock signal
	*	    TC_CMR_TCCLKS_TIMER_CLOCK3 : Clock selected: internal MCK/32 clock signal
	*	    TC_CMR_TCCLKS_TIMER_CLOCK4 : Clock selected: internal MCK/128 clock signal
	*	    TC_CMR_TCCLKS_TIMER_CLOCK5 : Clock selected: internal SLCK clock signal
	*
	*****************************************************************/
	tc_init(TC0, 0, TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK5);
	
	/*****************************************************************
	* Configura valor trigger RC
	*****************************************************************
	*
	* Aqui devemos configurar o valor do RC que vai trigar o reinicio da contagem
	* devemos levar em conta a frequência que queremos que o TC gere as interrupções
	* e tambem a frequencia com que o TC está operando.
	*
	* Devemos configurar o RC para o mesmo canal escolhido anteriormente.
	*
	*   ^
	*	|	Contador (incrementado na frequencia escolhida do clock)
	*   |
	*	|	 	Interrupcao
	*	|------#----------- RC
	*	|	  /
	*	|   /
	*	| /
	*	|-----------------> t
	*
	* Parametros :
	*   1 - TC a ser configurado (TC0,TC1, ...)
	*   2 - Canal a ser configurado (0,1,2)
	*   3 - Valor para trigger do contador (RC)
	*****************************************************************/
	tc_write_rc(TC0,0,32768*0.1);
	
	/*****************************************************************
	* Configura interrupção no TC
	*****************************************************************
	* Parametros :
	*   1 - TC a ser configurado
	*   2 - Canal
	*   3 - Configurações das interrupções
	*
	*        Essas configurações estão definidas no head : tc.h
	*
	*	        TC_IER_COVFS : 	Counter Overflow
	*	        TC_IER_LOVRS : 	Load Overrun
	*	        TC_IER_CPAS  : 	RA Compare
	*	        TC_IER_CPBS  : 	RB Compare
	*	        TC_IER_CPCS  : 	RC Compare
	*	        TC_IER_LDRAS : 	RA Loading
	*	        TC_IER_LDRBS : 	RB Loading
	*	        TC_IER_ETRGS : 	External Trigger
	*****************************************************************/
	tc_enable_interrupt(TC0,0,TC_IER_CPCS);
	
	/*****************************************************************
	* Ativar interrupção no NVIC*/
	
	
	NVIC_EnableIRQ(ID_TC0);
	
	/*****************************************************************
	* Inicializa o timer
	*****************************************************************
	*
	* Parametros :
	*   1 - TC
	*   2 - Canal
	*****************************************************************/
	tc_start(TC0,0);


	/************************************************************************/
	/* Main Code
	*/
	/************************************************************************/
}
void configure_adc(void)
{
	/* Enable peripheral clock. */
	pmc_enable_periph_clk(ID_ADC);
	/* Initialize ADC. */
	/*
	* Formula: ADCClock = MCK / ( (PRESCAL+1) * 2 )
	* For example, MCK = 64MHZ, PRESCAL = 4, then:
	* ADCClock = 64 / ((4+1) * 2) = 6.4MHz;
	*/
	/* Formula:
	*     Startup  Time = startup value / ADCClock
	*     Startup time = 64 / 6.4MHz = 10 us
	*/
	adc_init(ADC, sysclk_get_cpu_hz(), 6400000, STARTUP_TIME);
	/* Formula:
	*     Transfer Time = (TRANSFER * 2 + 3) / ADCClock
	*     Tracking Time = (TRACKTIM + 1) / ADCClock
	*     Settling Time = settling value / ADCClock
	*
	*     Transfer Time = (1 * 2 + 3) / 6.4MHz = 781 ns
	*     Tracking Time = (1 + 1) / 6.4MHz = 312 ns
	*     Settling Time = 3 / 6.4MHz = 469 ns
	*/
	adc_configure_timing(ADC, TRACKING_TIME	, ADC_SETTLING_TIME_3, TRANSFER_PERIOD);

	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);

	//adc_check(ADC, sysclk_get_cpu_hz());

	/* Enable channel for potentiometer. */
	adc_enable_channel(ADC, ADC_POT_CHANNEL);

	/* Enable the temperature sensor. */
	adc_enable_ts(ADC);

	/* Enable ADC interrupt. */
	NVIC_EnableIRQ(ADC_IRQn);

	/* Start conversion. */
	adc_start(ADC);

	//adc_read_buffer(ADC, gs_s_adc_values, BUFFER_SIZE);

	//adc_get_channel_value(ADC, ADC_POT_CHANNEL);

	/* Enable PDC channel interrupt. */
	adc_enable_interrupt(ADC, ADC_ISR_EOC5);
}

/************************************************************************/
/* MAIN                                                                 */
/************************************************************************/

int main(void)
{
	sysclk_init();
	board_init();

	configure_lcd();
	configure_botao();
	configure_adc();
	configure_tc();


	/** Draw text, image and basic shapes on the LCD */
	ili93xx_set_foreground_color(COLOR_BLACK);
	ili93xx_draw_string(10, 20, (uint8_t *)"14 - ADC");
	ili93xx_draw_filled_circle(120,160,60);
	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_circle(120,160,55);

	while (1) {
	}
}

