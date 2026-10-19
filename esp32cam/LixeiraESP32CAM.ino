/*
  Lixeira Inteligente - ESP32-CAM + Edge Impulse + BitDogLab
  Autor: Adrian Rosa - Projeto Lixeira Inteligente
  Modelo: adrianrosa07-project-1 (v1.0.2)
  Classes: reciclavel / nao_reciclavel
  Input do modelo: 96x96 RGB

  O que faz:
    - Inicializa a cÃ¢mera da ESP32-CAM (AI-Thinker)
    - Classifica imagem com Edge Impulse localmente
    - Imprime na Serial (115200) para debug
    - Envia o resultado via UART para a BitDogLab mover o servo

  FiaÃ§Ã£o com BitDogLab (FECHADO - sem LED aceso):
    BitDogLab GP16 (TX) â”€â”€â–º ESP32 GPIO14 (RX)  [header IO14, HS2_CLK - SEM LED, sem strapping]
    BitDogLab GP17 (RX) â—„â”€â”€ ESP32 GPIO13 (TX)  [header IO13, HS2_DATA3 - SEM LED]
    BitDogLab GND       â”€â”€â”€ ESP32 GND          (sem isso NADA funciona!)
    Por que 13/14: GPIO4 tem o LED flash (fica aceso em idle HIGH), 15 Ã© strapping MTDO.
    13/14 nÃ£o tÃªm LED e nÃ£o afetam boot. Se usar cartÃ£o SD, nÃ£o use 12/13/14/15/2/4.

  Protocolo UART (9600 8N1, linhas ASCII + \n):
    ESP32 -> BitDogLab: "RECICLAVEL" | "NAO_RECICLAVEL" | "DESCONHECIDO"
                        + "RESULTADO:reciclavel" (compatibilidade)
                        + "READY" | "PONG" | "CSV,..."
    BitDogLab -> ESP32: "CAPTURE" (classifica agora) | "READY" | "PING"
                        "ACK:RECICLAVEL" etc | "ERR:BUSY"

  Modos:
    - AutomÃ¡tico: classifica a cada INTERVALO_MS e envia se BitDogLab estiver READY
    - Sob demanda: BitDogLab envia CAPTURE ou BotÃ£o A da BitDogLab

  InstalaÃ§Ã£o:
    1. Biblioteca: Add .ZIP Library -> ei-adrianrosa07-project-1-arduino-1.0.2-impulse-#1.zip
       (apague a antiga v1.0.1 em Documents/Arduino/libraries/ se existir)
    2. Placa: AI Thinker ESP32-CAM | Huge APP | PSRAM Enabled
    3. Grave com FTDI/USB-CAM-MB (IO0->GND), depois solte e RST
    4. BitDogLab: compile C:\Users\Adr1an\Desktop\bitdoglab\ e copie .uf2 em BOOTSEL
*/

#include <adrianrosa07-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include "esp_heap_caps.h"

// ===================== Pinos AI-Thinker =====================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ===================== UART para BitDogLab =====================
// DATASHEET AI-Thinker ESP32-CAM:
//   Camera OV2640: 32(PWDN) 0(XCLK) 26(SDA) 27(SCL) 35(Y9) 34(Y8) 39(Y7) 36(Y6)
//                  21(Y5) 19(Y4) 18(Y3) 5(Y2) 25(VSYNC) 23(HREF) 22(PCLK)
//   PSRAM/flash: 6-11,16,17 interno | USB: 1/3 | Strapping: 0,2,12,15
//   SD_MMC HS2: 2,4,12,13,14,15 - livres SE nao usar cartao SD
// ESCOLHA FECHADA: 13/14 nÃ£o tÃªm LED e nÃ£o sÃ£o strapping crÃ­tico.
// GPIO4 tem LED flash (fica aceso em idle HIGH), GPIO15 Ã© strapping MTDO.
#define LINK_BAUD       9600
#define LINK_RX_PIN     14  // ESP32 RX <- BitDogLab GP16 TX  (HS2_CLK - SEM LED)
#define LINK_TX_PIN     13  // ESP32 TX -> BitDogLab GP17 RX  (HS2_DATA3 - SEM LED)
#define LINK_LINE_CAPACITY 48

// ===================== Configuracao =====================
#define SERIAL_BAUD         115200
#define INTERVALO_MS        4000
#define COOLDOWN_TIMEOUT_MS 8000        // seguranca: sem READY em 8s = link caido
#define REPETICAO_BLOQUEIO_MS 10000
#define LIMIAR_CONFIANCA    0.60f
#define LIMIAR_MARGEM       0.10f
// Cooldown agora eh timeout de seguranca: READY da BitDogLab libera proxima classificacao
#define PING_INTERVALO_MS   10000
#define LINK_TIMEOUT_MS     15000

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS   320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS   240
#define EI_CAMERA_FRAME_BYTE_SIZE          3

static const char LABEL_RECICLAVEL[]     = "reciclavel";
static const char LABEL_NAO_RECICLAVEL[] = "nao_reciclavel";
static const char LABEL_DESCONHECIDO[]   = "desconhecido";

// ===================== VariÃ¡veis globais =====================
static bool is_initialised = false;
static bool camera_ok = false;
uint8_t *snapshot_buf = nullptr;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// Link com BitDogLab - READY eh fonte da verdade (servo parado)
static char link_incoming[LINK_LINE_CAPACITY];
static size_t link_incoming_len = 0;
static bool link_incoming_discard = false;
static bool controller_ready = false;   // BitDogLab enviou READY (servo parado) - libera proxima classificacao
static bool waiting_ack = false;        // aguardando ACK da ultima classificacao
static unsigned long waiting_since_ms = 0; // timeout do ACK (2.5s)
static char last_sent_label[24] = "desconhecido";
static unsigned long last_sent_ms = 0;
static char pending_label[24] = "";
static bool pending_to_send = false;
static unsigned long last_ready_ms = 0; // legado (mantido por compatibilidade, nao usado como dead block)
static unsigned long cooldownUntil = 0; // agora so timeout de seguranca (8s sem READY = link caido)
static unsigned long lastServoMoveMs = 0;
static bool link_conectado = false;
static unsigned long ultimo_ready_ou_pong_ms = 0;
static unsigned long ultimo_ping_ms = 0;

// ProtÃ³tipos
bool ei_camera_init(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
void run_classification(bool from_capture);
void send_to_bitdog(const char *label);
void poll_bitdog_link(void);
void handle_bitdog_line(const char *line);

// ===================== SETUP =====================
#line 149 "C:\\Users\\Adr1an\\Desktop\\separaÃ§Ã£o\\LixeiraESP32CAM\\LixeiraESP32CAM.ino"
void setup();
#line 201 "C:\\Users\\Adr1an\\Desktop\\separaÃ§Ã£o\\LixeiraESP32CAM\\LixeiraESP32CAM.ino"
void loop();
#line 149 "C:\\Users\\Adr1an\\Desktop\\separaÃ§Ã£o\\LixeiraESP32CAM\\LixeiraESP32CAM.ino"
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);
    Serial.println();
    Serial.println("===========================================");
    Serial.println("  LIXEIRA INTELIGENTE - ESP32-CAM");
    Serial.println("  Modelo: adrianrosa07-project-1 (96x96)");
    Serial.println("  Classes: reciclavel / nao_reciclavel");
    Serial.printf("  Link BitDogLab: %d baud RX=GPIO%d TX=GPIO%d\n", LINK_BAUD, LINK_RX_PIN, LINK_TX_PIN);
    Serial.println("===========================================");

    Serial.printf("PSRAM: %s\n", psramFound() ? "SIM" : "NAO (vai falhar!)");
    if (psramFound()) {
        Serial.printf("PSRAM livre: %u bytes | Heap livre: %u\n", ESP.getFreePsram(), ESP.getFreeHeap());
    }

    // UART para BitDogLab (Serial2)
    Serial2.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
    Serial.printf("[LINK] UART2 iniciada: %d baud (RX GPIO%d <- GP16 | TX GPIO%d -> GP17)\n",
                  LINK_BAUD, LINK_RX_PIN, LINK_TX_PIN);

    // Aloca buffer UMA vez em PSRAM no setup (em vez de malloc/free a cada classificacao - item 3)
    size_t _buf_size = EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE;
    if (psramFound()) {
        snapshot_buf = (uint8_t*)heap_caps_malloc(_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!snapshot_buf) snapshot_buf = (uint8_t*)heap_caps_malloc(_buf_size, MALLOC_CAP_8BIT);
    if (!snapshot_buf) snapshot_buf = (uint8_t*)malloc(_buf_size);
    if (snapshot_buf) Serial.printf("[BUF] Snapshot %u bytes em %s (alocado 1x no setup)\n", (unsigned)_buf_size, psramFound() ? "PSRAM" : "HEAP");
    else Serial.println("[ERRO] Falha ao alocar snapshot_buf! Classificacao vai falhar.");

    ultimo_ready_ou_pong_ms = millis();
    ultimo_ping_ms = millis();
    link_conectado = false;
    controller_ready = false; // READY da BitDogLab libera - nao assume pronta no boot

    Serial.println("\n[INFO] Inicializando camera...");
    if (ei_camera_init()) {
        Serial.println("[OK] Camera inicializada!");
        camera_ok = true;
    } else {
        Serial.println("[ERRO] Falha ao inicializar camera! Verifique alimentaÃ§Ã£o 5V 1A+");
        camera_ok = false;
    }

    Serial.printf("\n[INFO] Modelo: %d categorias\n", EI_CLASSIFIER_LABEL_COUNT);
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.printf("  [%d] %s\n", (int)i, ei_classifier_inferencing_categories[i]);
    }
    Serial.printf("  Limiar: %.0f%% | Margem: %.0f%% | Intervalo: %d ms\n",
                  LIMIAR_CONFIANCA*100, LIMIAR_MARGEM*100, INTERVALO_MS);
    Serial.println();
    Serial.println("[INFO] Comandos Serial (USB):");
    Serial.println("  c / ENTER = classifica agora | h = ajuda | p = envia PING para BitDogLab");
    Serial.println("[INFO] Comandos da BitDogLab via UART:");
    Serial.println("  CAPTURE = classifica agora | READY = pronta | PING -> PONG");
    Serial.println();
    Serial.println("[INFO] FiaÃ§Ã£o: GP16->GPIO14 | GP17<-GPIO13 | GND-GND | Servo GP2 BitDogLab");
    Serial.println("-----------------------------------------------------------");
    delay(1000);
}

// ===================== LOOP =====================
void loop() {
    poll_bitdog_link();

    // Timeout do ACK (2.5s): se BitDogLab nÃ£o enviou ACK, assume perdida e libera
    if (waiting_ack && millis() - waiting_since_ms > 2500) {
        Serial.println("[LINK] Timeout ACK (2.5s) - liberando");
        waiting_ack = false;
        controller_ready = true;
        cooldownUntil = 0;
    }

    // Timeout de seguranca (COOLDOWN_TIMEOUT_MS): se enviou e nao recebeu READY, considera link caido
    if ((waiting_ack || !controller_ready) && millis() - lastServoMoveMs > COOLDOWN_TIMEOUT_MS) {
        Serial.printf("[TIMER] Timeout seguranca %d ms sem READY (servo deveria ter voltado) - liberando\n", COOLDOWN_TIMEOUT_MS);
        waiting_ack = false;
        controller_ready = true;
        cooldownUntil = 0;
    }

    // PING periodico + deteccao link caido
    if (millis() - ultimo_ping_ms > PING_INTERVALO_MS) {
        ultimo_ping_ms = millis();
        if (!waiting_ack) {
            Serial.println("[LINK] PING -> BitDogLab");
            Serial2.print("PING\n");
        }
        if (millis() - ultimo_ready_ou_pong_ms > LINK_TIMEOUT_MS) {
            if (link_conectado) {
                Serial.println("[LINK] AVISO: sem READY/PONG ha 15s - link DESCONECTADO");
                link_conectado = false;
                controller_ready = false;
            }
        }
    }

    // Se ha resultado pendente e BitDogLab ficou pronta (READY), envia agora
    // READY eh a fonte da verdade - nao usa cooldownAtivo
    if (pending_to_send && controller_ready && !waiting_ack) {
        Serial.printf("[LINK] BitDogLab READY -> enviando pendente: %s\n", pending_label);
        send_to_bitdog(pending_label);
        pending_to_send = false;
    }

    // Comandos manuais Serial USB (switch simplificado, ignora \r)
    if (Serial.available()) {
        char c = Serial.read();
        while (Serial.available()) {
            char peek = Serial.peek();
            if (peek == '\n' || peek == '\r') { Serial.read(); break; }
            Serial.read();
        }
        if (c == '\r' || c == '\n') {
            Serial.println("\n[CMD] Classificacao manual (ENTER) solicitada...");
            run_classification(false);
        } else {
            switch (tolower(c)) {
                case 'c': Serial.println("\n[CMD] Classificacao manual (c) solicitada..."); run_classification(false); break;
                case 'h': Serial.println("\n--- Comandos: c/ENTER=classificar | h=ajuda | p=ping BitDogLab ---"); break;
                case 'p': Serial.println("[LINK] Enviando PING -> BitDogLab"); Serial2.print("PING\n"); break;
                default: break;
            }
        }
    }

    // Classificacao automatica: so se IDLE (READY true, sem ACK pendente, sem pendente, link OK)
    static unsigned long ultima_ms = 0;
    bool pode_classificar = controller_ready && !waiting_ack && !pending_to_send && link_conectado;
    // Modo standalone (link caiu): classifica mas nao envia ao servo
    bool modo_standalone = !link_conectado && controller_ready && !waiting_ack;
    if (modo_standalone && millis() - ultima_ms >= INTERVALO_MS) {
        ultima_ms = millis();
        Serial.println("[MODO] Standalone (sem BitDogLab) - classificando apenas para log");
        run_classification(false);
    } else if (pode_classificar && millis() - ultima_ms >= INTERVALO_MS) {
        ultima_ms = millis();
        run_classification(false);
    }

    delay(5);
}

// ===================== Envio para BitDogLab =====================
void send_to_bitdog(const char *label) {
    // REPETICAO_BLOQUEIO_MS mantido como seguranca secundaria (redundante com READY-based)
    if (strcmp(label, LABEL_DESCONHECIDO) != 0 && strcmp(label, last_sent_label) == 0 && (millis() - last_sent_ms) < REPETICAO_BLOQUEIO_MS) {
        long falta = (long)REPETICAO_BLOQUEIO_MS - (long)(millis() - last_sent_ms);
        Serial.printf("[BLOQUEIO] Mesmo comando '%s' ha %lu ms - ignora por %ld ms\n", label, millis() - last_sent_ms, falta);
        Serial.printf("[LINK] >>> BLOQUEADO (repeticao): %s - aguardando %ld ms\n", label, falta);
        return;
    }

    strncpy(last_sent_label, label, sizeof(last_sent_label)-1);
    last_sent_label[sizeof(last_sent_label)-1] = '\0';
    last_sent_ms = millis();

    bool isUnknown = (strcmp(label, LABEL_DESCONHECIDO) == 0);
    const char *upper_label;
    if (strcmp(label, LABEL_RECICLAVEL) == 0) upper_label = "RECICLAVEL";
    else if (strcmp(label, LABEL_NAO_RECICLAVEL) == 0) upper_label = "NAO_RECICLAVEL";
    else upper_label = "DESCONHECIDO";

    Serial.printf("[LINK] >>> Enviando: %s (UART2 TX GPIO%d -> GP%d)%s\n",
                  upper_label, LINK_TX_PIN, 17, isUnknown ? " [SEM TRAVA]" : "");

    if (isUnknown) {
        Serial2.print(upper_label); Serial2.write('\n'); Serial2.flush();
        Serial.println("[LINK] DESCONHECIDO enviado sem trava (servo parado)");
        return;
    }

    Serial2.print(upper_label); Serial2.write('\n'); Serial2.flush();

    controller_ready = false;
    waiting_ack = true;
    waiting_since_ms = millis();
    lastServoMoveMs = millis();
    // COOLDOWN_TIMEOUT_MS agora eh APENAS timeout de seguranca (8s sem READY = link caido)
    // READY da BitDogLab eh que libera de verdade
    cooldownUntil = millis() + COOLDOWN_TIMEOUT_MS;
    Serial.printf("[TIMER] Timeout seguranca %d ms iniciado - aguardando READY da BitDogLab\n", COOLDOWN_TIMEOUT_MS);
}

// ===================== Recebe da BitDogLab =====================
void poll_bitdog_link(void) {
    while (Serial2.available()) {
        int c = Serial2.read();
        if (c == '\r') continue;
        if (c == '\n') {
            if (!link_incoming_discard && link_incoming_len > 0) {
                link_incoming[link_incoming_len] = '\0';
                handle_bitdog_line(link_incoming);
            }
            link_incoming_len = 0;
            link_incoming_discard = false;
            continue;
        }
        if (link_incoming_discard) continue;
        if (c < 32 || c > 126 || link_incoming_len + 1 >= sizeof(link_incoming)) {
            link_incoming_discard = true;
            continue;
        }
        link_incoming[link_incoming_len++] = (char)c;
    }
}

void handle_bitdog_line(const char *line) {
    Serial.printf("[LINK] <<< BitDogLab: %s (RX GPIO%d <- GP%d)\n", line, LINK_RX_PIN, 16);

    char upper[LINK_LINE_CAPACITY];
    strncpy(upper, line, sizeof(upper)-1);
    upper[sizeof(upper)-1] = '\0';
    for (char *p = upper; *p; p++) *p = toupper((unsigned char)*p);
    char *u = upper;
    while (*u == ' ' || *u == '\t') u++;

    // READY = servo voltou ao centro e esta PARADO (fonte da verdade)
    if (strcmp(u, "READY") == 0) {
        unsigned long tempoServo = millis() - lastServoMoveMs;
        Serial.printf("[LINK] BitDogLab READY - servo parado em %lu ms\n", tempoServo);
        ultimo_ready_ou_pong_ms = millis();
        link_conectado = true;
        waiting_ack = false;
        controller_ready = true;
        cooldownUntil = 0; // READY sempre libera completamente
        Serial.println("[TIMER] READY recebido - ESP liberada para proxima classificacao");
        return;
    }
    if (strcmp(u, "CAPTURE") == 0) {
        if (millis() < cooldownUntil || !controller_ready) {
            Serial.printf("[LINK] CAPTURE ignorado: servo ainda voltando ao 0 (%lu ms) ou nÃ£o READY\n",
                          cooldownUntil > millis() ? cooldownUntil - millis() : 0);
            Serial2.print("ERR:BUSY\n");
            return;
        }
        Serial.println("[LINK] CAPTURE recebido -> classificando agora");
        run_classification(true);
        return;
    }
    if (strcmp(u, "PING") == 0) {
        Serial.println("[LINK] PING -> PONG");
        Serial2.print("PONG\n");
        return;
    }
    if (strcmp(u, "PONG") == 0) {
        Serial.println("[LINK] PONG recebido (link OK)");
        ultimo_ready_ou_pong_ms = millis();
        link_conectado = true;
        return;
    }
    if (strncmp(u, "ACK:", 4) == 0) {
        Serial.printf("[LINK] ACK recebido: %s -> servo girando, aguardando READY (volta ao 0)\n", u);
        waiting_ack = false;
        controller_ready = false; // espera READY (servo ainda fora do 0)
        // MantÃ©m cooldown atÃ© READY
        return;
    }
    if (strcmp(u, "ERR:BUSY") == 0) {
        Serial.println("[LINK] BitDogLab ocupada (ERR:BUSY) -> vai reenviar quando READY");
        waiting_ack = false;
        controller_ready = false;
        // Reenfileira o Ãºltimo label
        strncpy(pending_label, last_sent_label, sizeof(pending_label)-1);
        pending_label[sizeof(pending_label)-1] = '\0';
        pending_to_send = true;
        return;
    }
    if (strncmp(u, "ERR:", 4) == 0) {
        Serial.printf("[LINK] Erro da BitDogLab: %s -> reenviando quando READY\n", u);
        waiting_ack = false;
        controller_ready = false;
        strncpy(pending_label, last_sent_label, sizeof(pending_label)-1);
        pending_label[sizeof(pending_label)-1] = '\0';
        pending_to_send = true;
        return;
    }

    Serial.printf("[LINK] Linha nÃ£o reconhecida: %s\n", line);
}

// ===================== ClassificaÃ§Ã£o =====================
void run_classification(bool from_capture) {
    if (!camera_ok) {
        Serial.println("[ERRO] Camera nao inicializada");
        Serial.println("RESULTADO:erro_camera");
        if (from_capture || controller_ready) send_to_bitdog(LABEL_DESCONHECIDO);
        return;
    }

    if (!snapshot_buf) {
        Serial.println("[ERRO] snapshot_buf nao alocado (falha no setup)");
        Serial.println("RESULTADO:erro_memoria");
        if (from_capture || controller_ready) send_to_bitdog(LABEL_DESCONHECIDO);
        return;
    }

    Serial.println(from_capture ? "\n[CAPTURA] CAPTURE -> capturando..." : "\n[CAPTURA] Capturando imagem...");
    bool ok = ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf);
    if (!ok) {
        Serial.println("[ERRO] Falha ao capturar imagem");
        Serial.println("RESULTADO:erro_captura");
        if (from_capture || controller_ready) send_to_bitdog(LABEL_DESCONHECIDO);
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    if (err != EI_IMPULSE_OK) {
        Serial.printf("[ERRO] run_classifier falhou (%d)\n", err);
        Serial.println("RESULTADO:erro_modelo");
        if (from_capture || controller_ready) send_to_bitdog(LABEL_DESCONHECIDO);
        return;
    }

    float score_rec = 0, score_nao = 0;
    float best = -1, second = -1;
    size_t best_idx = 0;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        float v = result.classification[i].value;
        const char* l = result.classification[i].label;
        if (strcmp(l, LABEL_RECICLAVEL) == 0) score_rec = v;
        else if (strcmp(l, LABEL_NAO_RECICLAVEL) == 0) score_nao = v;
        if (v > best) { second = best; best = v; best_idx = i; }
        else if (v > second) second = v;
    }
    const char* best_label = result.classification[best_idx].label;

    Serial.printf("Predictions (DSP:%d ms Class:%d ms):\n", result.timing.dsp, result.timing.classification);
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.printf("  %s: %.5f (%.1f%%)\n", result.classification[i].label,
                      result.classification[i].value, result.classification[i].value*100);
    }

    const char* final_label;
    bool ok_conf = true;
    if (best < LIMIAR_CONFIANCA) {
        ok_conf = false;
        Serial.printf("[AVISO] Confianca %.1f%% < %.0f%% -> DESCONHECIDO\n", best*100, LIMIAR_CONFIANCA*100);
    }
    if (second >= 0 && (best - second) < LIMIAR_MARGEM) {
        ok_conf = false;
        Serial.printf("[AVISO] Margem %.1f%% < %.0f%% -> DESCONHECIDO\n", (best-second)*100, LIMIAR_MARGEM*100);
    }
    if (!ok_conf) final_label = LABEL_DESCONHECIDO;
    else {
        if (strcmp(best_label, LABEL_RECICLAVEL) == 0) final_label = LABEL_RECICLAVEL;
        else if (strcmp(best_label, LABEL_NAO_RECICLAVEL) == 0) final_label = LABEL_NAO_RECICLAVEL;
        else final_label = LABEL_DESCONHECIDO;
    }

    // SaÃ­da Serial USB (debug)
    if (strcmp(final_label, LABEL_RECICLAVEL) == 0)
        Serial.printf(">>> RESULTADO: RECICLAVEL (%.1f%%)\n", best*100);
    else if (strcmp(final_label, LABEL_NAO_RECICLAVEL) == 0)
        Serial.printf(">>> RESULTADO: NAO_RECICLAVEL (%.1f%%)\n", best*100);
    else
        Serial.printf(">>> RESULTADO: DESCONHECIDO (melhor %s %.1f%%)\n", best_label, best*100);

    Serial.print("RESULTADO:"); Serial.println(final_label);
    Serial.printf("CSV,%.4f,%.4f,%s,%d\n", score_rec, score_nao, final_label,
                  result.timing.dsp + result.timing.classification);
    Serial.println("-----------------------------------------------------------");

    // snapshot_buf alocado 1x no setup em PSRAM - NAO dar free aqui

    // Envia para BitDogLab
    // Se veio de CAPTURE, envia imediatamente mesmo se nÃ£o estava READY (resposta direta)
    // Se Ã© automÃ¡tico, sÃ³ envia se BitDogLab estiver pronta, senÃ£o enfileira
    if (from_capture) {
        send_to_bitdog(final_label);
    } else {
        // AutomÃ¡tico
        bool standalone = !controller_ready && !waiting_ack && !pending_to_send;
        if (standalone) {
            // Sem BitDogLab conectada, sÃ³ loga na Serial (jÃ¡ feito)
            Serial.println("[LINK] BitDogLab nÃ£o conectada (standalone) - resultado sÃ³ na Serial");
        } else if (controller_ready && !waiting_ack) {
            send_to_bitdog(final_label);
        } else {
            // Enfileira para quando ficar READY
            Serial.printf("[LINK] BitDogLab ocupada -> enfileirando: %s\n", final_label);
            strncpy(pending_label, final_label, sizeof(pending_label)-1);
            pending_label[sizeof(pending_label)-1] = '\0';
            pending_to_send = true;
        }
    }
}

// ===================== CÃ¢mera =====================
bool ei_camera_init(void) {
    if (is_initialised) return true;
    // FIX 2026-10-17: NÃƒO configure GPIO13/14 como INPUT se estÃ£o em uso para UART com BitDogLab
    // Se LINK usa 13/14, pula esses pinMode senÃ£o quebra Serial2 (TX nÃ£o chega na BitDogLab -> ACK nunca vem)
    if (LINK_RX_PIN != 13 && LINK_RX_PIN != 14 && LINK_TX_PIN != 13 && LINK_TX_PIN != 14) {
        pinMode(13, INPUT_PULLUP);
        pinMode(14, INPUT_PULLUP);
    }
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init falhou 0x%x\n", err);
        return false;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        if (s->id.PID == OV3660_PID) { s->set_vflip(s, 1); s->set_brightness(s, 1); s->set_saturation(s, -1); }
        s->set_gain_ctrl(s, 1); s->set_exposure_ctrl(s, 1); s->set_awb_gain(s, 1); s->set_whitebal(s, 1);
    }
    is_initialised = true;
    return true;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) return false;
    // Descarta primeiro frame (pode ser antigo no buffer da camera)
    camera_fb_t *fb_antigo = esp_camera_fb_get();
    if (fb_antigo) {
        esp_camera_fb_return(fb_antigo);
        delay(30);
    }
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);
    if (!converted) return false;
    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf, img_width, img_height);
    }
    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pix = offset * 3;
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (snapshot_buf[pix+2] << 16) + (snapshot_buf[pix+1] << 8) + snapshot_buf[pix];
        pix += 3;
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Modelo invÃ¡lido: precisa ser modelo de CÃ‚MERA do Edge Impulse"
#endif

