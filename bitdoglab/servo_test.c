#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

// FS90R continuo - mesmos pulsos do main.c (sincronizado com P0)
#define SERVO_PIN 2u
#define SERVO_PERIOD_US 20000u
#define PARADO_US 1500u
#define LADO_1_US 1300u
#define LADO_2_US 1700u

static void servo_init_continuo(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    float div = (float)clock_get_hz(clk_sys) / 1000000.0f;
    pwm_set_clkdiv(slice, div);
    pwm_set_wrap(slice, SERVO_PERIOD_US - 1);
    pwm_set_gpio_level(SERVO_PIN, PARADO_US);
    pwm_set_enabled(slice, true);
    printf("Servo FS90R GP%u: parado %u us (50Hz)\n", SERVO_PIN, PARADO_US);
    printf("Ajuste fino: PARADO_US 1480/1520 se ficar girando parado\n");
}

static void gira(uint us, uint ms) {
    printf(" -> %u us por %u ms\n", us, ms);
    pwm_set_gpio_level(SERVO_PIN, us);
    sleep_ms(ms);
    pwm_set_gpio_level(SERVO_PIN, PARADO_US);
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    servo_init_continuo();
    printf("Teste FS90R: LADO1 %u / LADO2 %u / PARADO %u\n", LADO_1_US, LADO_2_US, PARADO_US);

    while (true) {
        printf("[TESTE] LADO1 -> espera -> LADO2\n");
        gira(LADO_1_US, 500);
        sleep_ms(2000);
        gira(LADO_2_US, 500);
        sleep_ms(1000);
        printf("[TESTE] LADO2 -> espera -> LADO1\n");
        gira(LADO_2_US, 500);
        sleep_ms(2000);
        gira(LADO_1_US, 500);
        sleep_ms(2000);
    }
}
