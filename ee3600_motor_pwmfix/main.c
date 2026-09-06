#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

// EE3600 DC motor controller - firmware 2.0-D-PWMFIX
// Raspberry Pi Pico / RP2040 + Kitronik motor driver
// Encoder: GP0=A, GP1=B
// Motor 1: GP3=forward PWM, GP2=reverse PWM
//
// PWMFIX change:
//   In MODE,OPEN, PWM,x is a direct open-loop request and is clamped only to
//   the physical range -100..+100 %. The configurable LIMIT,pct is retained
//   for closed-loop position PID output only. This allows genuine 70/80/90/100
//   step tests even when the dashboard has a lower closed-loop safety LIMIT.

#define ENC_A_PIN 0
#define ENC_B_PIN 1
#define MOTOR_REV_PIN 2
#define MOTOR_FWD_PIN 3

#define PWM_FREQ_HZ 10000u
#define PWM_WRAP 9999u
#define CONTROL_PERIOD_US 10000u
#define TELEMETRY_PERIOD_US 20000u
#define SPEED_PERIOD_US 20000u
#define CMD_BUF_LEN 128

static volatile int32_t encoder_count = 0;
static volatile uint8_t encoder_prev_state = 0;

static int encoder_cpr = 937;
static int encoder_sign = 1;
static float pwm_limit = 95.0f;
static float kp = 0.2000f;
static float ki = 0.0000f;
static float kd = 0.0100f;
static float target_deg = 0.0f;
static float open_pwm = 0.0f;
static float applied_pwm = 0.0f;
static float measured_rpm = 0.0f;
static bool telemetry_enabled = true;

typedef enum { MODE_OPEN = 0, MODE_POS = 1 } control_mode_t;
static control_mode_t mode = MODE_OPEN;
static float pid_integral = 0.0f;
static float pid_prev_error = 0.0f;
static bool pid_prev_valid = false;
static uint motor_slice;
static uint motor_fwd_chan;
static uint motor_rev_chan;

static inline float clampf_local(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static const char *mode_name(void) { return (mode == MODE_POS) ? "POS" : "OPEN"; }

static int32_t read_encoder_count(void) {
    uint32_t irq = save_and_disable_interrupts();
    int32_t c = encoder_count;
    restore_interrupts(irq);
    return c;
}

static void set_encoder_count(int32_t v) {
    uint32_t irq = save_and_disable_interrupts();
    encoder_count = v;
    encoder_prev_state = (uint8_t)((gpio_get(ENC_A_PIN) << 1) | gpio_get(ENC_B_PIN));
    restore_interrupts(irq);
}

static float position_deg_from_count(int32_t count) {
    if (encoder_cpr <= 0) return 0.0f;
    return (float)encoder_sign * (float)count * 360.0f / (float)encoder_cpr;
}

static void encoder_gpio_callback(uint gpio, uint32_t events) {
    (void)gpio; (void)events;
    const uint8_t now = (uint8_t)((gpio_get(ENC_A_PIN) << 1) | gpio_get(ENC_B_PIN));
    const uint8_t transition = (uint8_t)((encoder_prev_state << 2) | now);
    static const int8_t qdec[16] = {
         0, -1, +1,  0,
        +1,  0,  0, -1,
        -1,  0,  0, +1,
         0, +1, -1,  0
    };
    encoder_count += qdec[transition & 0x0F];
    encoder_prev_state = now;
}

static void motor_pwm_init(void) {
    gpio_set_function(MOTOR_FWD_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_REV_PIN, GPIO_FUNC_PWM);
    motor_slice = pwm_gpio_to_slice_num(MOTOR_FWD_PIN);
    motor_fwd_chan = pwm_gpio_to_channel(MOTOR_FWD_PIN);
    motor_rev_chan = pwm_gpio_to_channel(MOTOR_REV_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    const float div = (float)clock_get_hz(clk_sys) / ((float)PWM_FREQ_HZ * (float)(PWM_WRAP + 1u));
    pwm_config_set_clkdiv(&cfg, div);
    pwm_init(motor_slice, &cfg, true);
    pwm_set_chan_level(motor_slice, motor_fwd_chan, 0);
    pwm_set_chan_level(motor_slice, motor_rev_chan, 0);
}

static void motor_set_percent(float percent) {
    percent = clampf_local(percent, -100.0f, 100.0f);
    applied_pwm = percent;
    const float mag = fabsf(percent);
    uint16_t level = (uint16_t)lroundf((mag / 100.0f) * (float)PWM_WRAP);
    if (level > PWM_WRAP) level = PWM_WRAP;
    if (percent > 0.0001f) {
        pwm_set_chan_level(motor_slice, motor_rev_chan, 0);
        pwm_set_chan_level(motor_slice, motor_fwd_chan, level);
    } else if (percent < -0.0001f) {
        pwm_set_chan_level(motor_slice, motor_fwd_chan, 0);
        pwm_set_chan_level(motor_slice, motor_rev_chan, level);
    } else {
        pwm_set_chan_level(motor_slice, motor_fwd_chan, 0);
        pwm_set_chan_level(motor_slice, motor_rev_chan, 0);
    }
}

static void pid_reset(void) {
    pid_integral = 0.0f;
    pid_prev_error = 0.0f;
    pid_prev_valid = false;
}

static void run_position_pid(float dt_s) {
    const int32_t count = read_encoder_count();
    const float pos = position_deg_from_count(count);
    const float error = target_deg - pos;
    pid_integral += error * dt_s;
    pid_integral = clampf_local(pid_integral, -5000.0f, 5000.0f);
    float derivative = 0.0f;
    if (pid_prev_valid && dt_s > 0.0f) derivative = (error - pid_prev_error) / dt_s;
    pid_prev_error = error;
    pid_prev_valid = true;
    float u = kp * error + ki * pid_integral + kd * derivative;
    u = clampf_local(u, -pwm_limit, pwm_limit);
    motor_set_percent(u);
}

static void print_info(void) {
    printf("INFO,EE3600_MOTOR_PID,2.0-D-PWMFIX\n");
    printf("INFO,PINS,MOTOR1=GP3_FWD/GP2_REV,ENCODER=GP0/GP1\n");
    printf("INFO,COMMANDS,HELLO|HELP|STATUS|STOP|ZERO|MODE,OPEN|MODE,POS|PWM,x|TARGET,deg|PID,kp,ki,kd|CPR,n|ENCSIGN,+1|-1|LIMIT,pct\n");
    printf("INFO,PWMFIX,OPEN_MODE_PWM_DIRECT_-100_TO_100;LIMIT_APPLIES_TO_POS_PID\n");
}

static void print_status(void) {
    printf("STATUS,mode=%s,cpr=%d,encsign=%d,limit=%.3f,target_deg=%.3f,kp=%.4f,ki=%.4f,kd=%.4f,count=%ld\n",
           mode_name(), encoder_cpr, encoder_sign, pwm_limit, target_deg,
           kp, ki, kd, (long)read_encoder_count());
}

static void print_help(void) {
    print_info();
    printf("HELP,OPEN PWM is direct and can reach 100 percent; LIMIT is the closed-loop PID safety clamp.\n");
}

static void stop_motor(void) {
    open_pwm = 0.0f;
    pid_reset();
    motor_set_percent(0.0f);
}

static void trim_line(char *s) {
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\r' || s[n - 1] == '\n' || s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = '\0';
    size_t start = 0;
    while (s[start] == ' ' || s[start] == '\t') start++;
    if (start > 0) memmove(s, s + start, strlen(s + start) + 1);
}

static void handle_command(char *line) {
    trim_line(line);
    if (line[0] == '\0') return;
    if (strcmp(line, "HELLO") == 0) { print_info(); print_status(); return; }
    if (strcmp(line, "HELP") == 0) { print_help(); return; }
    if (strcmp(line, "STATUS") == 0) { print_status(); return; }
    if (strcmp(line, "STOP") == 0) { stop_motor(); printf("OK,STOP\n"); return; }
    if (strcmp(line, "ZERO") == 0) { set_encoder_count(0); target_deg = 0.0f; pid_reset(); printf("OK,ZERO\n"); return; }
    if (strcmp(line, "MODE,OPEN") == 0) { mode = MODE_OPEN; pid_reset(); motor_set_percent(open_pwm); printf("OK,MODE,OPEN\n"); return; }
    if (strcmp(line, "MODE,POS") == 0) { mode = MODE_POS; open_pwm = 0.0f; pid_reset(); printf("OK,MODE,POS\n"); return; }
    if (strcmp(line, "TELEM,ON") == 0 || strcmp(line, "TELEMETRY,ON") == 0) { telemetry_enabled = true; printf("OK,TELEM,ON\n"); return; }
    if (strcmp(line, "TELEM,OFF") == 0 || strcmp(line, "TELEMETRY,OFF") == 0) { telemetry_enabled = false; printf("OK,TELEM,OFF\n"); return; }

    if (strncmp(line, "PWM,", 4) == 0) {
        float v = clampf_local(strtof(line + 4, NULL), -100.0f, 100.0f);
        open_pwm = v;
        if (mode == MODE_OPEN) motor_set_percent(open_pwm); // PWMFIX: no LIMIT clamp here
        printf("OK,PWM,%.3f\n", v);
        return;
    }
    if (strncmp(line, "TARGET,", 7) == 0) { target_deg = strtof(line + 7, NULL); printf("OK,TARGET,%.3f\n", target_deg); return; }
    if (strncmp(line, "CPR,", 4) == 0) {
        int v = atoi(line + 4);
        if (v >= 1 && v <= 1000000) { encoder_cpr = v; printf("OK,CPR,%d\n", encoder_cpr); }
        else printf("ERR,CPR\n");
        return;
    }
    if (strncmp(line, "ENCSIGN,", 8) == 0) {
        int v = atoi(line + 8);
        if (v == 1 || v == -1) { encoder_sign = v; printf("OK,ENCSIGN,%d\n", encoder_sign); }
        else printf("ERR,ENCSIGN\n");
        return;
    }
    if (strncmp(line, "LIMIT,", 6) == 0) {
        float v = strtof(line + 6, NULL);
        if (isfinite(v)) { pwm_limit = clampf_local(fabsf(v), 0.0f, 100.0f); printf("OK,LIMIT,%.3f\n", pwm_limit); }
        else printf("ERR,LIMIT\n");
        return;
    }
    if (strncmp(line, "PID,", 4) == 0) {
        float a, b, c;
        if (sscanf(line + 4, "%f,%f,%f", &a, &b, &c) == 3 && isfinite(a) && isfinite(b) && isfinite(c)) {
            kp = a; ki = b; kd = c; pid_reset();
            printf("OK,PID,%.4f,%.4f,%.4f\n", kp, ki, kd);
        } else printf("ERR,PID\n");
        return;
    }
    printf("ERR,UNKNOWN,%s\n", line);
}

int main(void) {
    stdio_init_all();
    gpio_init(ENC_A_PIN); gpio_set_dir(ENC_A_PIN, GPIO_IN); gpio_pull_up(ENC_A_PIN);
    gpio_init(ENC_B_PIN); gpio_set_dir(ENC_B_PIN, GPIO_IN); gpio_pull_up(ENC_B_PIN);
    encoder_prev_state = (uint8_t)((gpio_get(ENC_A_PIN) << 1) | gpio_get(ENC_B_PIN));
    gpio_set_irq_enabled_with_callback(ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_gpio_callback);
    gpio_set_irq_enabled(ENC_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    motor_pwm_init();
    stop_motor();

    absolute_time_t next_control = make_timeout_time_us(CONTROL_PERIOD_US);
    absolute_time_t next_telem = make_timeout_time_us(TELEMETRY_PERIOD_US);
    absolute_time_t next_speed = make_timeout_time_us(SPEED_PERIOD_US);
    int32_t last_speed_count = read_encoder_count();
    absolute_time_t last_speed_time = get_absolute_time();
    char cmd[CMD_BUF_LEN];
    size_t cmd_len = 0;

    while (true) {
        int ch;
        while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            if (ch == '\r' || ch == '\n') {
                if (cmd_len > 0) { cmd[cmd_len] = '\0'; handle_command(cmd); cmd_len = 0; }
            } else if (cmd_len < CMD_BUF_LEN - 1) cmd[cmd_len++] = (char)ch;
            else { cmd_len = 0; printf("ERR,CMD_TOO_LONG\n"); }
        }

        const absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(now, next_control) <= 0) {
            next_control = delayed_by_us(next_control, CONTROL_PERIOD_US);
            if (mode == MODE_POS) run_position_pid((float)CONTROL_PERIOD_US / 1000000.0f);
            else if (fabsf(applied_pwm - open_pwm) > 0.0001f) motor_set_percent(open_pwm);
        }
        if (absolute_time_diff_us(now, next_speed) <= 0) {
            next_speed = delayed_by_us(next_speed, SPEED_PERIOD_US);
            const int32_t c = read_encoder_count();
            const absolute_time_t tnow = get_absolute_time();
            const int64_t dt_us = absolute_time_diff_us(last_speed_time, tnow);
            if (dt_us > 0 && encoder_cpr > 0) {
                const int32_t dc = c - last_speed_count;
                const float raw_rpm = (float)encoder_sign * (float)dc * 60000000.0f / ((float)encoder_cpr * (float)dt_us);
                measured_rpm = 0.65f * measured_rpm + 0.35f * raw_rpm;
            }
            last_speed_count = c;
            last_speed_time = tnow;
        }
        if (absolute_time_diff_us(now, next_telem) <= 0) {
            next_telem = delayed_by_us(next_telem, TELEMETRY_PERIOD_US);
            if (telemetry_enabled) {
                const int32_t count = read_encoder_count();
                const float pos = position_deg_from_count(count);
                const uint32_t ms = to_ms_since_boot(get_absolute_time());
                printf("DATA,%lu,%s,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%ld\n",
                       (unsigned long)ms, mode_name(), target_deg, pos, measured_rpm,
                       applied_pwm, kp, ki, kd, (long)count);
            }
        }
        tight_loop_contents();
    }
}
