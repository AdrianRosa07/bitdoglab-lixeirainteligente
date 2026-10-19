# Lixeira Inteligente - ESP32-CAM + Edge Impulse + BitDogLab

**Modelo:** `adrianrosa07-project-1` v1.0.2 | **Classes:** `reciclavel` / `nao_reciclavel` | **Input:** 96x96 RGB

## O que faz
1. ESP32-CAM captura imagem (320x240 → 96x96) e classifica com Edge Impulse localmente
2. Imprime na **Serial USB 115200** para debug
3. **Envia o resultado via UART 9600 para a BitDogLab** que move o servo

```
ESP32-CAM (ML) ──UART 9600──► BitDogLab (GP16/GP17) ──PWM──► Servo GP2
```

## Fiação BitDogLab ↔ ESP32-CAM

```
BitDogLab GP16 (TX) ──► ESP32 GPIO14 (RX)
BitDogLab GP17 (RX) ◄── ESP32 GPIO13 (TX)
BitDogLab GND       ─── ESP32 GND          ← OBRIGATÓRIO!
Servo sinal         ─── BitDogLab GP2
Servo VCC           ─── 5V externo (1-2A, NÃO use USB do Pico!)
Servo GND           ─── GND comum (Pico + ESP32 + fonte 5V)
Botão A             ─── GP5 (teste manual)
```

## Protocolo UART (9600 8N1, linhas com \n)

**ESP32 → BitDogLab:**
- `RECICLAVEL` → servo 55° (reciclável)
- `NAO_RECICLAVEL` → servo 125° (não reciclável)
- `DESCONHECIDO` → mantém no centro 90°

**BitDogLab → ESP32:**
- `READY` → pronta para próximo item
- `CAPTURE` → classifica agora (botão A)
- `ACK:RECICLAVEL` etc → confirma recebimento
- `ERR:BUSY` → ESP32 reenfileira e reenvia quando READY

## Saída na Serial (USB 115200)

```
Predictions (DSP:12 ms Class:84 ms):
  nao_reciclavel: 0.07659 (7.7%)
  reciclavel: 0.92341 (92.3%)
>>> RESULTADO: RECICLAVEL (92.3%)
RESULTADO:reciclavel
CSV,0.9234,0.0766,reciclavel,96
[LINK] >>> Enviando para BitDogLab: RECICLAVEL
[LINK] <<< BitDogLab: ACK:RECICLAVEL
[LINK] <<< BitDogLab: READY
```

## Instalação ESP32-CAM

1. Biblioteca: `Sketch > Include Library > Add .ZIP Library` → selecione `ei-adrianrosa07-project-1-arduino-1.0.2-impulse-#1.zip`
   - Se já tem `adrianrosa07-project-1_inferencing` v1.0.1 em `Documents/Arduino/libraries/`, apague antes
2. Placa: `AI Thinker ESP32-CAM` | `Partition: Huge APP` | `PSRAM: Enabled`
3. Grave com FTDI/USB-CAM-MB (IO0→GND, RST)
4. Abra Serial Monitor 115200

## Instalação BitDogLab

Pasta: `C:\Users\Adr1an\Desktop\bitdoglab\` (projeto Pico SDK)

**VS Code + Pico Extension (recomendado):**
1. Abra a pasta `bitdoglab` no VS Code
2. `Pico: Compile Project` → gera `build/bitdoglab_lixeira.uf2`
3. Pico em BOOTSEL (segure BOOTSEL, plugue USB) → copie o `.uf2` para `RPI-RP2`

**Teste só do servo:** compile `bitdoglab_servo_test.uf2` (mesma pasta, servo em GP2)

Serial BitDogLab (115200):
```
Lixeira Inteligente - BitDogLab
UART0: 9600 baud | TX=GP16 -> ESP32 GPIO14 | RX=GP17 <- ESP32 GPIO13
Servo SG90: GP2 | centro=90 | reciclavel=55 | nao_reciclavel=125
Aguardando classificacao...
UART RX: RECICLAVEL
Classificacao: RECICLAVEL -> SG90 para 55 graus
Divisoria voltou ao centro (90 graus) -> pronta
```

## Modos de operação

- **Automático:** ESP32 classifica a cada 2s e envia se BitDogLab estiver READY (senão enfileira)
- **Sob demanda:** BitDogLab envia `CAPTURE` (botão A) → ESP32 classifica na hora
- **Standalone:** Sem BitDogLab, ESP32 só imprime na Serial (funciona igual)

## Calibração

Em `LixeiraESP32CAM.ino`:
```cpp
#define INTERVALO_MS     2000   // intervalo auto
#define LIMIAR_CONFIANCA 0.60f  // <60% = desconhecido
#define LIMIAR_MARGEM    0.10f
```

Em `bitdoglab/main.c`:
```c
#define SERVO_CENTER_DEG 90u      // centro
#define SERVO_RECYCLE_DEG 55u     // reciclável
#define SERVO_NONRECYCLE_DEG 125u // não reciclável
#define SERVO_HOLD_MS 900u
```
Ajuste os ângulos até a divisória abrir certo. Se inverter, troque 55↔125.

## Troubleshooting

- **Nada acontece:** Confira GND comum (Pico+ESP32+fonte 5V)
- **Servo treme:** Fonte 5V fraca, use fonte externa
- **BitDogLab não recebe:** TX/RX cruzados? GP16→GPIO14, GP17←GPIO13, 9600 baud
- **ESP32 não responde CAPTURE:** Atualize o sketch (este já tem UART)
- **Servo ao contrário:** Troque os ângulos no `main.c`
- `PSRAM nao detectada` -> ative PSRAM no Tools ou troque a ESP32-CAM
- `Falha ao iniciar camera (0x105)` -> alimentação fraca, use 5V 2A
