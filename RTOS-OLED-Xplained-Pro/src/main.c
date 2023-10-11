#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include <time.h>

//DEFINES DO MOTOR
#define IN1_PIO     PIOD
#define IN1_PIO_ID  ID_PIOD
#define IN1_PIO_PIN 30
#define IN1_PIO_PIN_MASK (1 << IN1_PIO_PIN)

#define IN2_PIO     PIOA
#define IN2_PIO_ID  ID_PIOA
#define IN2_PIO_PIN 6
#define IN2_PIO_PIN_MASK (1 << IN2_PIO_PIN)

#define IN3_PIO     PIOC
#define IN3_PIO_ID  ID_PIOC
#define IN3_PIO_PIN 19
#define IN3_PIO_PIN_MASK (1 << IN3_PIO_PIN)

#define IN4_PIO     PIOA
#define IN4_PIO_ID  ID_PIOA
#define IN4_PIO_PIN 2
#define IN4_PIO_PIN_MASK (1 << IN4_PIO_PIN)

//DEFINES DOS BOTÕES DA PLACA OLED
#define BUT_1_PIO				PIOD
#define BUT_1_PIO_ID			ID_PIOD
#define BUT_1_PIO_IDX			28
#define BUT_1_PIO_IDX_MASK		(1u << BUT_1_PIO_IDX)

#define BUT_2_PIO PIOC
#define BUT_2_PIO_ID ID_PIOC
#define BUT_2_PIO_IDX 31
#define BUT_2_PIO_IDX_MASK (1u << BUT_2_PIO_IDX)

#define BUT_3_PIO PIOA
#define BUT_3_PIO_ID ID_PIOA
#define BUT_3_PIO_IDX 19
#define BUT_3_PIO_IDX_MASK (1u << BUT_3_PIO_IDX)

#define BUZZER_PIO PIOD
#define BUZZER_PIO_ID ID_PIOD
#define BUZZER_PIO_IDX 20
#define BUZZER_PIO_IDX_MASK (1u << BUZZER_PIO_IDX)

/** RTOS  */
#define TASK_MODO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MODO_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_MOTOR_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MOTOR_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_COINS_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_COINS_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_PLAY_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_PLAY_STACK_PRIORITY            (tskIDLE_PRIORITY)

/** Notes */
#define NOTE_B5  988
#define NOTE_E6  1319

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************/
/* recursos RTOS                                                        */
/************************/

QueueHandle_t xQueueModo;
QueueHandle_t xQueueSteps;
QueueHandle_t xQueuePlay;
QueueHandle_t xQueueCoins;
SemaphoreHandle_t xSemaphoreRTT;
SemaphoreHandle_t xBtnSemaphore;


/** prototypes */
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
static void BUT_init(void);
static void MOTOR_init(void);
void fase0(void);
void fase1(void);
void fase2(void);
void fase3(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/************************/
/* RTOS application funcs                                               */
/************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************/
/* handlers / callbacks                                                 */
/************************/

void but1_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	int angulo = 180;
	xQueueSendFromISR(xQueueModo, &angulo, &xHigherPriorityTaskWoken);
}

void but2_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	int angulo = 90;
	xQueueSendFromISR(xQueueModo, &angulo, &xHigherPriorityTaskWoken);
}

void but3_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	int angulo = 45;
	xQueueSendFromISR(xQueueModo, &angulo, &xHigherPriorityTaskWoken);
}


void RTT_Handler(void){
	RTT_init(1000, 100, RTT_MR_ALMIEN);
	
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xSemaphoreGiveFromISR(xSemaphoreRTT, &xHigherPriorityTaskWoken);
	}
}
void set_buzzer() {
	BUZZER_PIO->PIO_SODR = BUZZER_PIO_IDX_MASK;
}

void clear_buzzer() {
	BUZZER_PIO->PIO_CODR = BUZZER_PIO_IDX_MASK;
}

void tone(int freq, int time) {
	int T = 1000000 / (2 * freq);
	int quant = (time * 1000) / (T * 2);

	for (int i = 0; i < quant; i++) {
		set_buzzer();
		delay_us(T);
		clear_buzzer();
		delay_us(T);
	}
}

/************************/
/* TASKS                                                                */
/************************/

static void task_modo(void *pvParameters) {
	
	RTT_init(1000, 5, RTT_MR_ALMIEN);
	
	gfx_mono_ssd1306_init();
	
	int angulo;
	char str[128];
	
	for (;;)  {
		if (xQueueReceive(xQueueModo, &(angulo), 1000)) {
			int passos = angulo / 0.17578125;
			
			sprintf(str, "%d", angulo); // (2)
			gfx_mono_draw_string(str, 50,16, &sysfont);
			
			xQueueSend(xQueueSteps, (void *)&passos, 10);
		}
	}
}

static void task_motor(void *pvParameters) {
	int passos = 0;
	for (;;)  {
		if (xQueueReceive(xQueueSteps, &passos, (TickType_t) 0)) {
			int cont = 0;
			printf("%d \n", passos);
			for(int i = 0; i < passos; i++){
				printf("%d \n", i);
				if (xSemaphoreTake(xSemaphoreRTT, 1000)) {
					if(cont == 0){
						fase0();
					} else if (cont == 1){
						fase1();
					} else if (cont == 2){
						fase2();
					} else if (cont == 3){
						fase3();
					}
					
					cont++;
					
					if(cont==4){
						cont = 0;
					}
				}
			}
		}
	}
}
	
void task_coins(void *pvParameters) {
	int coinValue;
	while (1) {
		if (xSemaphoreTake(xBtnSemaphore, portMAX_DELAY) == pdTRUE) {

			// Adicione 1 ao número gerado para obter um valor entre 1 e 3.
			int coinValue = 1;
			xQueueSend(xQueueCoins, &coinValue, portMAX_DELAY);
		}
	}
}

void task_play(void *pvParameters) {
	int coinValue;
	while (1) {
		if (xQueueReceive(xQueueCoins, &coinValue, portMAX_DELAY)) {
			tone(NOTE_B5,  80);
			for (int i = 0; i < coinValue+1; i++){
				tone(NOTE_B5,  80);
				tone(NOTE_E6, 640);
			}
		}
	}
}


/************************/
/* funcoes                                                              */
/************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_OLED_init(void){
	//Botão OLED 1
	pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_PIO_IDX_MASK, PIO_PULLUP);
	
	pio_handler_set(BUT_1_PIO,
	BUT_1_PIO_ID,
	BUT_1_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but1_callback);
	
	pio_enable_interrupt(BUT_1_PIO, BUT_1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT_1_PIO);
	pmc_enable_periph_clk(BUZZER_PIO_ID);

	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4); // Prioridade 4
	
	//Botão OLED 2
	pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_PIO_IDX_MASK, PIO_PULLUP);
	
	pio_handler_set(BUT_2_PIO,
	BUT_2_PIO_ID,
	BUT_2_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but2_callback);
	
	pio_enable_interrupt(BUT_2_PIO, BUT_2_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT_2_PIO);

	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4); // Prioridade 4
	
	//Botão OLED 3
	pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_PIO_IDX_MASK, PIO_PULLUP);
	
	pio_handler_set(BUT_3_PIO,
	BUT_3_PIO_ID,
	BUT_3_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but3_callback);
	
	pio_enable_interrupt(BUT_3_PIO, BUT_3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT_3_PIO);

	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4); // Prioridade 4
	
	NVIC_EnableIRQ(BUZZER_PIO_ID);
	NVIC_SetPriority(BUZZER_PIO_ID, 4); // Prioridade 4
}

static void MOTOR_init(void){
	pio_set_output(IN1_PIO, IN1_PIO_PIN_MASK, 0,0,0);
	pio_set_output(IN2_PIO, IN2_PIO_PIN_MASK, 0,0,0);
	pio_set_output(IN3_PIO, IN3_PIO_PIN_MASK, 0,0,0);
	pio_set_output(IN4_PIO, IN4_PIO_PIN_MASK, 0,0,0);
	pio_set_output(BUZZER_PIO,BUZZER_PIO_IDX_MASK, 0, 0, 0);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}

void fase0(void){
	pio_set(IN1_PIO, IN1_PIO_PIN_MASK);
	pio_clear(IN2_PIO, IN2_PIO_PIN_MASK);
	pio_clear(IN3_PIO, IN3_PIO_PIN_MASK);
	pio_clear(IN4_PIO, IN4_PIO_PIN_MASK);
}

void fase1(void){
	pio_clear(IN1_PIO, IN1_PIO_PIN_MASK);
	pio_set(IN2_PIO, IN2_PIO_PIN_MASK);
	pio_clear(IN3_PIO, IN3_PIO_PIN_MASK);
	pio_clear(IN4_PIO, IN4_PIO_PIN_MASK);
}

void fase2(void){
	pio_clear(IN1_PIO, IN1_PIO_PIN_MASK);
	pio_clear(IN2_PIO, IN2_PIO_PIN_MASK);
	pio_set(IN3_PIO, IN3_PIO_PIN_MASK);
	pio_clear(IN4_PIO, IN4_PIO_PIN_MASK);
}

void fase3(void){
	pio_clear(IN1_PIO, IN1_PIO_PIN_MASK);
	pio_clear(IN2_PIO, IN2_PIO_PIN_MASK);
	pio_clear(IN3_PIO, IN3_PIO_PIN_MASK);
	pio_set(IN4_PIO, IN4_PIO_PIN_MASK);
}

/************************/
/* main                                                                 */
/************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	BUT_OLED_init();
	

	/* Initialize the console uart */
	configure_console();
	
	xQueueModo = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueModo == NULL)
	printf("falha em criar a queue \n");
	
	xQueueCoins = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueCoins == NULL)
	printf("falha em criar a queue \n");

	xQueuePlay = xQueueCreate(32, sizeof(uint32_t));
	if (xQueuePlay == NULL)
	printf("falha em criar a queue \n");
	
	xQueueSteps = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueSteps == NULL)
	printf("falha em criar a queue \n");
	
	xBtnSemaphore = xSemaphoreCreateBinary();
	xSemaphoreRTT = xSemaphoreCreateBinary();
	if (xSemaphoreRTT == NULL)
	printf("falha em criar o semaforo \n");

	if (xTaskCreate(task_modo, "modo", TASK_MODO_STACK_SIZE, NULL, TASK_MODO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create modo task\r\n");
	}
	
	if (xTaskCreate(task_motor, "motor", TASK_MOTOR_STACK_SIZE, NULL, TASK_MOTOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create motor task\r\n");
	}
	if (xTaskCreate(task_coins, "coins", TASK_COINS_STACK_SIZE, NULL, TASK_COINS_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create coins task\r\n");
	}
	if (xTaskCreate(task_play, "play", TASK_PLAY_STACK_SIZE, NULL, TASK_PLAY_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create play task\r\n");
	}

	BUT_OLED_init();
	MOTOR_init();
	
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}