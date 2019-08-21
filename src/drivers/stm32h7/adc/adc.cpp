/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file adc.cpp
 *
 * Driver for the STM32H7 ADC.
 *
 * This is a low-rate driver, designed for sampling things like voltages
 * and so forth. It avoids the gross complexity of the NuttX ADC driver.
 */

#include <stm32_adc.h>
#include <stm32_gpio.h>

#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_config.h>
#include <px4_log.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/system_power.h>

using namespace time_literals;

#if defined(ADC_CHANNELS)

#define ADC_TOTAL_CHANNELS 		32

/*
 * Register accessors.
 * For now, no reason not to just use ADC1.
 */
#define REG(base, _reg) (*(volatile uint32_t *)((base) + (_reg)))

#define rCR(base)    REG((base), STM32_ADC_CR_OFFSET)
#define rISR(base)   REG((base), STM32_ADC_ISR_OFFSET)
#define rSMPR1(base) REG((base), STM32_ADC_SMPR1_OFFSET)
#define rSMPR2(base) REG((base), STM32_ADC_SMPR2_OFFSET)
#define rCFG(base)   REG((base), STM32_ADC_CFGR_OFFSET)
#define rCFG2(base)  REG((base), STM32_ADC_CFGR2_OFFSET)
#define rCCR(base)   REG((base), STM32_ADC_CCR_OFFSET)
#define rSQR1(base)  REG((base), STM32_ADC_SQR1_OFFSET)
#define rSQR2(base)  REG((base), STM32_ADC_SQR2_OFFSET)
#define rSQR3(base)  REG((base), STM32_ADC_SQR3_OFFSET)
#define rSQR4(base)  REG((base), STM32_ADC_SQR4_OFFSET)
#define rDR(base)    REG((base), STM32_ADC_DR_OFFSET)

#define ADC_SMPR_DEFAULT    ADC_SMPR_810p5
#define ADC_SMPR1_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR1_SMP0_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP1_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP2_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP3_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP4_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP5_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP6_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP7_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP8_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP9_SHIFT))
#define ADC_SMPR2_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR2_SMP10_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP11_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP12_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP13_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP14_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP15_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP16_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP17_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP18_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP19_SHIFT))


/* Assuming VDC 2.4 - 3.6 */

#define ADC_MAX_FADC 36000000

#if STM32_PLL2Q_FREQUENCY/2 <= ADC_MAX_FADC
#  define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV2
#elif STM32_PLL2Q_FREQUENCY/4 <= ADC_MAX_FADC
#  define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV4
#elif STM32_PLL2Q_FREQUENCY/6 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV6
#elif STM32_PLL2Q_FREQUENCY/8 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV8
#elif STM32_PLL2Q_FREQUENCY/12 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV12
#elif STM32_PLL2Q_FREQUENCY/16 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV16
#elif STM32_PLL2Q_FREQUENCY/32 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV32
#elif STM32_PLL2Q_FREQUENCY/64 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV64
#elif STM32_PLL2Q_FREQUENCY/128 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV128
#elif STM32_PLL2Q_FREQUENCY/256 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV256
#else
#  error "ADC STM32_PLL2Q_FREQUENCY too high - no divisor found "
#endif

class ADC : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	ADC(uint32_t base_address, uint32_t channels);
	~ADC();

	virtual int		init();

	virtual int		ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t		read(file *filp, char *buffer, size_t len);

protected:
	virtual int		open_first(struct file *filp);
	virtual int		close_last(struct file *filp);

private:
	void			Run() override;

	/**
	 * Sample a single channel and return the measured value.
	 *
	 * @param channel		The channel to sample.
	 * @return			The sampled value, or 0xffff if
	 *				sampling failed.
	 */
	uint16_t		sample(unsigned channel);

	void			update_adc_report(hrt_abstime now);
	void			update_system_power(hrt_abstime now);


	static const hrt_abstime	kINTERVAL{10_ms};	/**< 100Hz base rate */

	perf_counter_t			_sample_perf;

	unsigned			_channel_count{0};
	const uint32_t			_base_address;
	px4_adc_msg_t			*_samples{nullptr};	/**< sample buffer */

	uORB::Publication<adc_report_s>		_to_adc_report{ORB_ID(adc_report)};
	uORB::Publication<system_power_s>	_to_system_power{ORB_ID(system_power)};
};

ADC::ADC(uint32_t base_address, uint32_t channels) :
	CDev(ADC0_DEVICE_PATH),
	ScheduledWorkItem(px4::wq_configurations::hp_default),
	_sample_perf(perf_alloc(PC_ELAPSED, "adc_samples")),
	_base_address(base_address)
{
	/* always enable the temperature sensor */
	channels |= 1 << 16;

	/* allocate the sample array */
	for (unsigned i = 0; i < ADC_TOTAL_CHANNELS; i++) {
		if (channels & (1 << i)) {
			_channel_count++;
		}
	}

	if (_channel_count > PX4_MAX_ADC_CHANNELS) {
		PX4_ERR("PX4_MAX_ADC_CHANNELS is too small:is %d needed:%d", PX4_MAX_ADC_CHANNELS, _channel_count);
	}

	_samples = new px4_adc_msg_t[_channel_count];

	/* prefill the channel numbers in the sample array */
	if (_samples != nullptr) {
		unsigned index = 0;

		for (unsigned i = 0; i < ADC_TOTAL_CHANNELS; i++) {
			if (channels & (1 << i)) {
				_samples[index].am_channel = i;
				_samples[index].am_data = 0;
				index++;
			}
		}
	}
}

ADC::~ADC()
{
	if (_samples != nullptr) {
		delete _samples;
	}

	perf_free(_sample_perf);
}

int board_adc_init(uint32_t base_address)
{
	/* Perform ADC init once per ADC */

	static uint32_t once[SYSTEM_ADC_COUNT] {};

	uint32_t *free = nullptr;

	for (uint32_t i = 0; i < SYSTEM_ADC_COUNT; i++) {
		if (once[i] == base_address) {

			/* This one was done already */

			return OK;
		}

		/* Use first free slot */

		if (free == nullptr && once[i] == 0) {
			free = &once[i];
		}
	}

	if (free == nullptr) {

		/* ADC misconfigured SYSTEM_ADC_COUNT too small */;

		PANIC();
	}

	*free = base_address;

	/* do calibration if supported */

	rCR(base_address)  = ADC_CR_ADVREGEN;

	/* Wait for voltage regulator to power up */

	up_udelay(20);

	/* Enable ADC calibration. ADCALDIF == 0 so this is only for
	 * single-ended conversions, not for differential ones.
	 */

	rCR(base_address)  |= ADC_CR_ADCAL;

	/* Wait for calibration to complete */

	hrt_abstime now = hrt_absolute_time();

	while ((rCR(base_address) & ADC_CR_ADCAL)) {

		/* don't wait for more than 500us, since that means something broke
		 * should reset here if we see this */
		if ((hrt_absolute_time() - now) > 500) {
			return -1;
		}
	}


	/* Enable ADC
	 * Note: ADEN bit cannot be set during ADCAL=1 and 4 ADC clock cycle
	 * after the ADCAL bit is cleared by hardware. If we are using SYSCLK
	 * as ADC clock source, this is the same as time taken to execute 4
	 * ARM instructions.
	 */

	rCR(base_address) |= ADC_CR_ADEN;

	now = hrt_absolute_time();

	/* Wait for hardware to be ready for conversions */

	while ((rISR(base_address) & ADC_INT_ADRDY) == 0) {

		/* don't wait for more than 500us, since that means something broke
		 * should reset here if we see this */
		if ((hrt_absolute_time() - now) > 500) {
			return -2;
		}
	}


	/* arbitrarily configure all channels for 810.5 cycle sample time */

	rSMPR1(base_address) = ADC_SMPR1_DEFAULT;
	rSMPR2(base_address) = ADC_SMPR2_DEFAULT;


	/* Set CFGR configuration
	 * Set the resolution of the conversion.
	 * Disable external trigger for regular channels
	 */

	rCFG(base_address) = (ADC_CFGR_RES_16BIT | ADC_CFGR_EXTEN_NONE);

	/* Set CFGR2 configuration to align right no oversample */

	rCFG2(base_address) = 0;

	/* enable the temperature sensor, VREFINT channel and VBAT */

	rCCR(base_address) = (ADC_CCR_VREFEN | ADC_CCR_VSENSEEN | ADC_CCR_VBATEN |
			      ADC_CCR_PRESC_NOT_DIV | ADC_CCR_CKMODE_ASYCH | ADC_CCR_PRESC_DIV);

	/* configure for a single-channel sequence */

	rSQR1(base_address) = 0;
	rSQR2(base_address) = 0;
	rSQR3(base_address) = 0;
	rSQR4(base_address) = 0;

	/* kick off a sample and wait for it to complete */
	now = hrt_absolute_time();
	rCR(base_address) |= ADC_CR_ADSTART;

	while (!(rISR(base_address) & ADC_INT_EOC)) {

		/* don't wait for more than 500us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 500) {
			return -1;
		}
	}

	return OK;
}

int
ADC::init()
{
	int rv = board_adc_init(_base_address);

	if (rv < 0) {
		PX4_DEBUG("sample timeout");
		return rv;
	}

	/* create the device node */
	return CDev::init();
}

int
ADC::ioctl(file *filp, int cmd, unsigned long arg)
{
	return -ENOTTY;
}

ssize_t
ADC::read(file *filp, char *buffer, size_t len)
{
	const size_t maxsize = sizeof(px4_adc_msg_t) * _channel_count;

	if (len > maxsize) {
		len = maxsize;
	}

	/* block interrupts while copying samples to avoid racing with an update */
	irqstate_t flags = px4_enter_critical_section();
	memcpy(buffer, _samples, len);
	px4_leave_critical_section(flags);

	return len;
}

int
ADC::open_first(struct file *filp)
{
	/* get fresh data */
	Run();

	/* and schedule regular updates */
	ScheduleOnInterval(kINTERVAL, kINTERVAL);

	return 0;
}

int
ADC::close_last(struct file *filp)
{
	ScheduleClear();

	return 0;
}

void
ADC::Run()
{
	hrt_abstime now = hrt_absolute_time();

	/* scan the channel set and sample each */
	for (unsigned i = 0; i < _channel_count; i++) {
		_samples[i].am_data = sample(_samples[i].am_channel);
	}

	update_adc_report(now);
	update_system_power(now);
}

void
ADC::update_adc_report(hrt_abstime now)
{
	adc_report_s adc = {};
	adc.timestamp = now;

	unsigned max_num = _channel_count;

	if (max_num > (sizeof(adc.channel_id) / sizeof(adc.channel_id[0]))) {
		max_num = (sizeof(adc.channel_id) / sizeof(adc.channel_id[0]));
	}

	for (unsigned i = 0; i < max_num; i++) {
		adc.channel_id[i] = _samples[i].am_channel;
		adc.channel_value[i] = _samples[i].am_data * 3.3f / 4096.0f;
	}

	_to_adc_report.publish(adc);
}

void
ADC::update_system_power(hrt_abstime now)
{
#if defined (BOARD_ADC_USB_CONNECTED)
	system_power_s system_power {};
	system_power.timestamp = now;

	/* Assume HW provides only ADC_SCALED_V5_SENSE */
	int cnt = 1;
	/* HW provides both ADC_SCALED_V5_SENSE and ADC_SCALED_V3V3_SENSORS_SENSE */
#  if defined(ADC_SCALED_V5_SENSE) && defined(ADC_SCALED_V3V3_SENSORS_SENSE)
	cnt++;
#  endif

	for (unsigned i = 0; i < _channel_count; i++) {
#  if defined(ADC_SCALED_V5_SENSE)

		if (_samples[i].am_channel == ADC_SCALED_V5_SENSE) {
			// it is 2:1 scaled
			system_power.voltage5v_v = _samples[i].am_data * (ADC_V5_V_FULL_SCALE / 4096.0f);
			cnt--;

		} else
#  endif
#  if defined(ADC_SCALED_V3V3_SENSORS_SENSE)
		{
			if (_samples[i].am_channel == ADC_SCALED_V3V3_SENSORS_SENSE) {
				// it is 2:1 scaled
				system_power.voltage3v3_v = _samples[i].am_data * (ADC_3V3_SCALE * (3.3f / 4096.0f));
				system_power.v3v3_valid = 1;
				cnt--;
			}
		}

#  endif

		if (cnt == 0) {
			break;
		}
	}

	/* Note once the board_config.h provides BOARD_ADC_USB_CONNECTED,
	 * It must provide the true logic GPIO BOARD_ADC_xxxx macros.
	 */
	// these are not ADC related, but it is convenient to
	// publish these to the same topic

	system_power.usb_connected = BOARD_ADC_USB_CONNECTED;
	/* If provided used the Valid signal from HW*/
#if defined(BOARD_ADC_USB_VALID)
	system_power.usb_valid = BOARD_ADC_USB_VALID;
#else
	/* If not provided then use connected */
	system_power.usb_valid  = system_power.usb_connected;
#endif

	/* The valid signals (HW dependent) are associated with each brick */
#if !defined(BOARD_NUMBER_DIGITAL_BRICKS)
	bool  valid_chan[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;
	system_power.brick_valid = 0;

	for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {
		system_power.brick_valid |=  valid_chan[b] ? 1 << b : 0;
	}

#endif

	system_power.servo_valid   = BOARD_ADC_SERVO_VALID;

#ifdef BOARD_ADC_PERIPH_5V_OC
	// OC pins are active low
	system_power.periph_5v_oc  = BOARD_ADC_PERIPH_5V_OC;
#endif

#ifdef BOARD_ADC_HIPOWER_5V_OC
	system_power.hipower_5v_oc = BOARD_ADC_HIPOWER_5V_OC;
#endif

	/* lazily publish */
	_to_system_power.publish(system_power);

#endif // BOARD_ADC_USB_CONNECTED
}

uint16_t board_adc_sample(uint32_t base_address, unsigned channel)
{
	irqstate_t flags = px4_enter_critical_section();

	/* clear any previous EOC */

	if (rISR(base_address) & ADC_INT_EOC) {
		rISR(base_address) &= ~ADC_INT_EOC;
	}

	/* run a single conversion right now - should take about 60 cycles (a few microseconds) max */

	rSQR3(base_address) = channel;
	rCR(base_address) |= ADC_CR_ADSTART;

	/* wait for the conversion to complete */
	const hrt_abstime now = hrt_absolute_time();

	while (!(rISR(base_address) & ADC_INT_EOC)) {

		/* don't wait for more than 50us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 50) {
			px4_leave_critical_section(flags);
			return 0xffff;
		}
	}

	/* read the result and clear EOC */
	uint16_t result = rDR(base_address);

	px4_leave_critical_section(flags);

	return result;
}

uint16_t
ADC::sample(unsigned channel)
{
	perf_begin(_sample_perf);
	uint16_t result = board_adc_sample(_base_address, channel);

	if (result == 0xffff) {
		PX4_ERR("sample timeout");
	}

	perf_end(_sample_perf);
	return result;
}

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int adc_main(int argc, char *argv[]);

namespace
{
ADC	*g_adc{nullptr};

int
test(void)
{

	int fd = open(ADC0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("can't open ADC device %d", errno);
		return 1;
	}

	for (unsigned i = 0; i < 50; i++) {
		px4_adc_msg_t data[ADC_TOTAL_CHANNELS];
		ssize_t count = read(fd, data, sizeof(data));

		if (count < 0) {
			PX4_ERR("read error");
			return 1;
		}

		unsigned channels = count / sizeof(data[0]);

		for (unsigned j = 0; j < channels; j++) {
			printf("%d: %u  ", data[j].am_channel, data[j].am_data);
		}

		printf("\n");
		px4_usleep(500000);
	}

	return 0;
}
}

int
adc_main(int argc, char *argv[])
{
	if (g_adc == nullptr) {
		/* XXX this hardcodes the default channel set for the board in board_config.h - should be configurable */
		g_adc = new ADC(SYSTEM_ADC_BASE, ADC_CHANNELS);

		if (g_adc == nullptr) {
			PX4_ERR("couldn't allocate the ADC driver");
			return 1;
		}

		if (g_adc->init() != OK) {
			delete g_adc;
			PX4_ERR("ADC init failed");
			return 1;
		}
	}

	if (argc > 1) {
		if (!strcmp(argv[1], "test")) {
			return test();
		}
	}

	return 0;
}
#endif
