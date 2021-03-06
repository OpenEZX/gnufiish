#ifndef __S3C2410_PWM_H
#define __S3C2410_PWM_H

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <mach/io.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <plat/regs-timer.h>

enum pwm_timer {
	PWM0,
	PWM1,
	PWM2,
	PWM3,
	PWM4
};

struct s3c2410_pwm {
	enum pwm_timer timerid;
	struct clk *pclk;
	unsigned long pclk_rate;
	unsigned long prescaler;
	unsigned long divider;
	unsigned long counter;
	unsigned long comparer;
};

struct s3c24xx_pwm_platform_data{
        /* callback to attach platform children (to enforce suspend / resume
         * ordering */
        void (*attach_child_devices)(struct device *parent_device);
};

int s3c2410_pwm_init(struct s3c2410_pwm *s3c2410_pwm);
int s3c2410_pwm_enable(struct s3c2410_pwm *s3c2410_pwm);
int s3c2410_pwm_disable(struct s3c2410_pwm *s3c2410_pwm);
int s3c2410_pwm_start(struct s3c2410_pwm *s3c2410_pwm);
int s3c2410_pwm_stop(struct s3c2410_pwm *s3c2410_pwm);
int s3c2410_pwm_duty_cycle(int reg_value, struct s3c2410_pwm *s3c2410_pwm);
int s3c2410_pwm_dumpregs(void);

#endif /* __S3C2410_PWM_H */
