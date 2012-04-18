#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/adc.h>
#include <libopencm3/stm32/f1/scb.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f1/rtc.h>
#include <libopencm3/stm32/usart.h>

static const int logging_interval_s = 120;
static uint16_t meas_data[24000];
static unsigned int meas_data_n = 0;

/* Set up all the peripherals */
void setup(void)
{
	rcc_clock_setup_in_hsi_out_48mhz();

	rcc_peripheral_enable_clock(&RCC_APB2ENR, 
			RCC_APB2ENR_IOPAEN |
			RCC_APB2ENR_IOPCEN |
			RCC_APB2ENR_AFIOEN | 
			RCC_APB2ENR_USART1EN);

	/* GPIO pin for USART TX */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);

	/* GPIO pin for button */
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO5);
	exti_select_source(EXTI5, GPIOC);

	/* GPIO pin for Vcc switcher */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	/* force switcher on */
	gpio_set(GPIOC, GPIO13);

	/* GPIO pin for amplifier */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
	/* turn off */
	gpio_clear(GPIOC, GPIO1);

	/* Setup USART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

void setup_adc(void)
{
	rcc_peripheral_enable_clock(&RCC_APB2ENR, 
			RCC_APB2ENR_ADC1EN);

	/* Make sure the ADC doesn't run during config. */
	adc_off(ADC1);

	/* We configure everything for one single conversion. */
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_enable_discontinous_mode_regular(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_conversion_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC);

	uint8_t channel_array[16];
	/* Select the channel we want to convert. */
	channel_array[0] = 16;
	adc_set_regular_sequence(ADC1, 1, channel_array);
}

static void setup_rtc(void) 
{
	rtc_awake_from_off(LSE);
	rtc_set_prescale_val(0x7fff-1);

	EXTI_EMR |= EXTI5 | EXTI17;
	exti_set_trigger(EXTI5, EXTI_TRIGGER_RISING);
	exti_set_trigger(EXTI17, EXTI_TRIGGER_RISING);
}

/* Provide _write syscall used by libc */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++) {
			usart_send_blocking(USART1, ptr[i]);
		}
		return i;
	} else {
		errno = EIO;
		return -1;
	}
}

uint16_t measure(void)
{
	adc_on(ADC1);
	adc_enable_temperature_sensor(ADC1);

	/* tstab = 1 us
	 * tstart = 10 us */
	int i;
	for (i = 0; i < 1000; i++)
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);

	adc_on(ADC1);
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));
	uint16_t v = ADC_DR(ADC1);

	adc_disable_temperature_sensor(ADC1);
	adc_off(ADC1);

	return v;
}

/* Delay execution for some arbitrary amount of time */
void delay(void)
{
	int i;

	for (i = 0; i < 8000000; i++) {
		__asm__("nop");
	}
}

void sleep(void)
{
	/* wait for any USART transfer to finish, or the last
	 * character might get garbled when the CPU goes to 
	 * sleep */
	while ((USART_SR(USART1) & USART_SR_TXE) == 0);
	while ((USART_SR(USART1) & USART_SR_TC) == 0);

	SCB_SCR |= SCB_SCR_SLEEPDEEP;
	PWR_CR &= ~PWR_CR_PDDS;
	PWR_CR |= PWR_CR_LPDS;

	__asm__("wfe");

	rcc_clock_setup_in_hsi_out_48mhz();
}

void dump_data(void)
{
	unsigned int n;
	for(n = 0; n < meas_data_n; n++) {
		printf("%u\n", meas_data[n]);
	}
}

int main(void)
{
	setup();
	setup_adc();
	setup_rtc();

	uint32_t rtc_counter = rtc_get_counter_val();
	rtc_set_alarm_time(rtc_counter + logging_interval_s-1);

	while (1) {
		if(!rtc_check_flag(RTC_ALR)) {
			dump_data();
		} else {
			rtc_clear_flag(RTC_ALR);
			if(meas_data_n < sizeof(meas_data)) {
				meas_data[meas_data_n] = measure();
				meas_data_n++;
			}

			rtc_counter = rtc_get_counter_val();
			rtc_set_alarm_time(rtc_counter + logging_interval_s-1);

			//printf("%lu\n", rtc_counter);

		}

		sleep();
		//delay();
	}

	return 0;
}
