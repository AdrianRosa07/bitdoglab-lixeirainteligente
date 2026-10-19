#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/clocks.h"

/*
 * BitDogLab - Lixeira com separacao - SERVO CONTINUO (FS90R)
 * Servo GP2: gira por tempo (nao tem posicao)
 *   1500 = parado | 1300 = LADO1 | 1700 = LADO2
 * RECICLAVEL     : LADO1 -> espera -> LADO2
 * NAO_RECICLAVEL : LADO2 -> espera -> LADO1
 * UART: GP16 TX -> ESP32 GPIO14 RX | GP17 RX <- ESP32 GPIO13 TX | 9600 8N1
 * Sem botoes - apenas ESP32-CAM controla via UART
 */

#define SERVO_PIN     2u

#define LINK_UART     uart0
#define LINK_BAUD     9600u
#define LINK_TX_PIN   16u
#define LINK_RX_PIN   17u

#define PERIODO_US    20000u
#define PARADO_US     1500u
#define LADO_1_US     1300u
#define LADO_2_US     1700u
#define TEMPO_GIRO_MS 500u
#define ESPERA_MS     2000u

#define LINE_MAX      48u

// ---------- Servo continuo - NAO BLOQUEANTE ----------
typedef enum { SERVO_PARADO = 0, SERVO_IDA, SERVO_ESPERA, SERVO_VOLTA } servo_estado_t;
static servo_estado_t servo_estado = SERVO_PARADO;
static absolute_time_t servo_deadline;
static uint16_t servo_ida_us = 0;
static uint16_t servo_volta_us = 0;
static char servo_nome[24] = "";

static void servo_init(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    float div = (float)clock_get_hz(clk_sys) / 1000000.0f;   // 1 tick = 1 us
    pwm_set_clkdiv(slice, div);
    pwm_set_wrap(slice, PERIODO_US - 1);
    pwm_set_gpio_level(SERVO_PIN, PARADO_US);
    pwm_set_enabled(slice, true);
    servo_estado = SERVO_PARADO;
    printf("Servo continuo GP%u: parado %u us (50Hz)\n", SERVO_PIN, PARADO_US);
    printf("Calibracao: se parado andar devagar, ajuste PARADO_US +-10 (ex: 1480/1520)\n");
}

static void servo_parar(void) { pwm_set_gpio_level(SERVO_PIN, PARADO_US); }
static void servo_girar(uint16_t pulso_us) { pwm_set_gpio_level(SERVO_PIN, pulso_us); }
static bool servo_ocupado(void) { return servo_estado != SERVO_PARADO; }

// Necessario antes de servo_poll usar uart_send
static void uart_send(const char *s) {
    uart_puts(LINK_UART, s);
    uart_puts(LINK_UART, "\n");
}

static void servo_iniciar(uint16_t ida, uint16_t volta, const char *nome) {
    servo_ida_us = ida;
    servo_volta_us = volta;
    strncpy(servo_nome, nome, sizeof(servo_nome)-1);
    servo_nome[sizeof(servo_nome)-1] = '\0';
    printf("[SERVO] %s: ida %u (%u ms) -> espera %u ms -> volta %u (%u ms)\n",
           nome, ida, (unsigned)TEMPO_GIRO_MS, (unsigned)ESPERA_MS, volta, (unsigned)TEMPO_GIRO_MS);
    servo_girar(ida);
    servo_estado = SERVO_IDA;
    servo_deadline = make_timeout_time_ms(TEMPO_GIRO_MS);
}

static void servo_poll(void) {
    if (servo_estado == SERVO_PARADO) return;
    if (!time_reached(servo_deadline)) return;
    if (servo_estado == SERVO_IDA) {
        servo_parar();
        printf("[SERVO] %s: ida OK -> esperando %u ms parado\n", servo_nome, (unsigned)ESPERA_MS);
        servo_estado = SERVO_ESPERA;
        servo_deadline = make_timeout_time_ms(ESPERA_MS);
    } else if (servo_estado == SERVO_ESPERA) {
        printf("[SERVO] %s: iniciando volta %u (%u ms)\n", servo_nome, servo_volta_us, (unsigned)TEMPO_GIRO_MS);
        servo_girar(servo_volta_us);
        servo_estado = SERVO_VOLTA;
        servo_deadline = make_timeout_time_ms(TEMPO_GIRO_MS);
    } else if (servo_estado == SERVO_VOLTA) {
        servo_parar();
        printf("[SERVO] %s: ciclo completo -> parado %u us\n", servo_nome, PARADO_US);
        servo_estado = SERVO_PARADO;
        uart_send("READY");
        printf("[LINK] >>> READY (servo parado, pronto)\n");
        servo_nome[0] = '\0';
        while (uart_is_readable(LINK_UART)) (void)uart_getc(LINK_UART);
    }
}

static void gira(uint16_t pulso_us, uint32_t ms) {
    pwm_set_gpio_level(SERVO_PIN, pulso_us);
    sleep_ms(ms);
    pwm_set_gpio_level(SERVO_PIN, PARADO_US);
}

static void abre_e_fecha(uint16_t ida, uint16_t volta, const char *nome) {
    // usado apenas no teste inicial (antes do loop nao-bloqueante)
    printf("[%s] ida %u (%u ms) -> espera %u ms -> volta %u (%u ms)\n",
           nome, ida, (unsigned)TEMPO_GIRO_MS, (unsigned)ESPERA_MS, volta, (unsigned)TEMPO_GIRO_MS);
    gira(ida, TEMPO_GIRO_MS);
    sleep_ms(ESPERA_MS);
    gira(volta, TEMPO_GIRO_MS);
    printf("[%s] OK parado\n", nome);
}

// ---------- Helpers UART ja declarada acima ----------

static void to_upper_trim(char *dst, const char *src, size_t n) {
    size_t j = 0;
    while (*src == ' ' || *src == '\t') src++;
    size_t len = strlen(src);
    while (len > 0 && (src[len-1] == ' ' || src[len-1] == '\t')) len--;
    for (size_t i = 0; i < len && j + 1 < n; i++) dst[j++] = toupper((unsigned char)src[i]);
    dst[j] = '\0';
}

static void acao_reciclavel(const char *nome) {
    if (servo_ocupado()) {
        printf("[SERVO] RECICLAVEL ignorado: ocupado -> ERR:BUSY\n");
        uart_send("ERR:BUSY");
        return;
    }
    uart_send("ACK:RECICLAVEL");
    servo_iniciar(LADO_1_US, LADO_2_US, nome);
}

static void acao_nao_reciclavel(const char *nome) {
    if (servo_ocupado()) {
        printf("[SERVO] NAO_RECICLAVEL ignorado: ocupado -> ERR:BUSY\n");
        uart_send("ERR:BUSY");
        return;
    }
    uart_send("ACK:NAO_RECICLAVEL");
    servo_iniciar(LADO_2_US, LADO_1_US, nome);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("\n========================================\n");
    printf(" BitDogLab Lixeira - Servo CONTINUO\n");
    printf("========================================\n");

    servo_init();

    uart_init(LINK_UART, LINK_BAUD);
    gpio_set_function(LINK_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LINK_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(LINK_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LINK_UART, true);

    // Teste inicial
    printf("--- TESTE INICIAL ---\n");
    abre_e_fecha(LADO_1_US, LADO_2_US, "TESTE-RECICLAVEL");
    sleep_ms(1000);
    abre_e_fecha(LADO_2_US, LADO_1_US, "TESTE-NAO");
    printf("--- TESTE OK -> aguardando ESP32 (UART) ---\n");
    uart_send("READY");

    char line[LINE_MAX];
    size_t len = 0;
    bool discard = false;

    while (true) {
        servo_poll();

        // SEM READY periodico: fonte da verdade eh o READY enviado ao fim do ciclo (servo_poll)
        // e o READY inicial. O ESP considera READY = servo parado e pronto.

        // UART (unico controle - sem botoes) - continua lendo mesmo durante IDA/ESPERA/VOLTA
        while (uart_is_readable(LINK_UART)) {
            int c = uart_getc(LINK_UART);
            if (c == '\r') continue;
            if (c == '\n') {
                if (!discard && len > 0) {
                    line[len] = '\0';
                    printf("UART RX: %s\n", line);
                    char up[LINE_MAX];
                    to_upper_trim(up, line, sizeof(up));

                    // ignora CSV/debug
                    if (strncmp(up, "CSV,", 4) == 0 || strncmp(up, ">>>", 3) == 0 || up[0] == '[') {
                        // ignora
                    } else {
                        const char *p = up;
                        if (strncmp(up, "RESULTADO:", 10) == 0) {
                            p = up + 10;
                            while (*p == ' ' || *p == '\t') p++;
                        }
                        if (strcmp(p, "RECICLAVEL") == 0) {
                            acao_reciclavel("RECICLAVEL");
                        } else if (strcmp(p, "NAO_RECICLAVEL") == 0 ||
                                   strcmp(p, "NAO-RECICLAVEL") == 0 ||
                                   strcmp(p, "NAORECICLAVEL") == 0) {
                            acao_nao_reciclavel("NAO_RECICLAVEL");
                        } else if (strcmp(p, "DESCONHECIDO") == 0 || strcmp(p, "UNKNOWN") == 0) {
                            if (servo_ocupado()) {
                                // Se esta em movimento, apenas confirma sem interromper ciclo atual
                                printf("DESCONHECIDO recebido durante movimento - mantem ciclo atual\n");
                                uart_send("ACK:DESCONHECIDO");
                            } else {
                                printf("DESCONHECIDO - servo parado\n");
                                pwm_set_gpio_level(SERVO_PIN, PARADO_US);
                                uart_send("ACK:DESCONHECIDO");
                                uart_send("READY");
                            }
                        } else if (strcmp(p, "PING") == 0) {
                            uart_send("PONG");
                        } else if (strcmp(p, "CAPTURE") == 0) {
                            uart_send("CAPTURE");
                        }
                    }
                }
                len = 0;
                discard = false;
                continue;
            }
            if (discard) continue;
            if (c < 32 || c > 126 || len + 1 >= LINE_MAX) { discard = true; continue; }
            line[len++] = (char)c;
        }
        sleep_ms(5);
    }
}