# BitDogLab - Lixeira Inteligente (UART + Servo HB1750/SG90)

Recebe a classificação da **ESP32-CAM** via UART e move o **servo motor** para separar o lixo.

## Fiação

```
BitDogLab GP16 (TX) ──► ESP32 GPIO14 (RX)
BitDogLab GP17 (RX) ◄── ESP32 GPIO13 (TX)
BitDogLab GND       ─── ESP32 GND          ← OBRIGATÓRIO (sem isso NADA funciona!)
Servo sinal (amarelo) ─── BitDogLab GP2
Servo VCC (vermelho)  ─── 5V externo (fonte 5V 1-2A, NÃO use a USB do Pico!)
Servo GND (marrom)    ─── GND comum (Pico + ESP32 + fonte 5V)
Botão A               ─── GP5 (já na placa BitDogLab)
```

> **Atenção:** Alimente o servo com 5V externo. Se alimentar só pela USB do Pico, a tensão cai e o Pico reinicia.

## Protocolo UART

- **Baud:** 9600 8N1
- **ESP32 → BitDogLab:**
  - `RECICLAVEL` → servo para 55°
  - `NAO_RECICLAVEL` → servo para 125°
  - `DESCONHECIDO` → mantém no centro (90°)
  - Compatível também com `RESULTADO:reciclavel` (do sketch simples da pasta `separação`)
  - Linhas `CSV,...` e `>>> RESULTADO:` são ignoradas

- **BitDogLab → ESP32:**
  - `READY` → pronta para próximo item
  - `ACK:RECICLAVEL` / `ACK:NAO_RECICLAVEL` / `ACK:DESCONHECIDO`
  - `CAPTURE` → solicita classificação (botão A)
  - `ERR:BUSY` → servo ocupado

## Fluxo

1. ESP32-CAM classifica a imagem (Edge Impulse 96x96)
2. ESP32 envia `RECICLAVEL` ou `NAO_RECICLAVEL` via UART (9600 baud)
3. BitDogLab recebe, move o servo para o ângulo correspondente
4. Após 900ms, volta ao centro (90°) e envia `READY`
5. Pronta para o próximo item

**Teste manual:** Pressione **Botão A (GP5)** na BitDogLab → envia `CAPTURE` para a ESP32-CAM classificar na hora.

## Compilação

### Opção 1: VS Code + Pico Extension (recomendado)
1. Instale a extensão **Raspberry Pi Pico** no VS Code
2. Abra a pasta `C:\Users\Adr1an\Desktop\bitdoglab`
3. `Pico: Compile Project`
4. Coloque o Pico em **BOOTSEL** (segure BOOTSEL, plugue USB, solte)
5. Copie `build/bitdoglab_lixeira.uf2` para o drive `RPI-RP2`

### Opção 2: Linha de comando
```bash
cd C:\Users\Adr1an\Desktop\bitdoglab
mkdir build && cd build
cmake -G "Ninja" ..
ninja
# Copie build/bitdoglab_lixeira.uf2 para o Pico em BOOTSEL
```

### Teste só do servo
Compile `bitdoglab_servo_test.uf2` (mesma pasta) para testar só o servo em GP2 sem precisar da ESP32.

## Serial Monitor

Após gravar, abra o Serial Monitor (115200 baud) na porta do Pico:

```
Lixeira Inteligente - BitDogLab
UART0: 9600 baud | TX=GP16 -> ESP32 GPIO14 | RX=GP17 <- ESP32 GPIO13
Servo SG90: GP2 | centro=90 | reciclavel=55 | nao_reciclavel=125 | hold=900ms
Aguardando classificacao da ESP32-CAM...

UART RX: RECICLAVEL
Classificacao: RECICLAVEL -> SG90 para 55 graus
Divisoria voltou ao centro (90 graus) -> pronta para proximo item
```

## Calibração do servo

No topo de `main.c`:

```c
#define SERVO_CENTER_DEG 90u      // centro / fechada
#define SERVO_RECYCLE_DEG 55u     // reciclável
#define SERVO_NONRECYCLE_DEG 125u // não reciclável
#define SERVO_HOLD_MS 900u        // tempo que fica na posição
```

Ajuste os ângulos até a divisória abrir corretamente. Se girar ao contrário, troque 55 ↔ 125.

## Troubleshooting

- **Nada acontece:** Confira GND comum entre Pico, ESP32 e fonte 5V
- **Servo treme/não move:** Fonte 5V fraca, use fonte externa 1-2A
- **BitDogLab não recebe:** Verifique TX/RX cruzados (GP16→GPIO14, GP17←GPIO13)
- **ESP32 não responde ao CAPTURE:** Atualize o sketch da ESP32 em `separação/LixeiraESP32CAM/` para a versão com UART
