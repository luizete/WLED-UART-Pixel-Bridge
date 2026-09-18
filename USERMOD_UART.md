# Mudanças no usermod WLED-UART-Pixel-Bridge

O que mudar no [WLED-UART-Pixel-Bridge](https://github.com/luizete/WLED-UART-Pixel-Bridge)
para a Pi ler, pela UART, o estado de todos os LEDs de cada par
master → receptor. A ponte do lado da Pi já existe (`ponte/`) e segue este
documento. Arquitetura e motivo: [PLANO.md](PLANO.md), *Transporte por UART*.

**Base lida:** `usermods/uart_pixel_bridge/uart_pixel_bridge.cpp`, versão
`1.4.0` (`bridge-version`), branch `main` em 14/09/2026. Os números de linha
abaixo são dessa versão.

**Decidido pelo usuário:** o master manda todos os LEDs; o receptor descarta
o que não é dele; a tabela de segmentos vai na mesma UART, num quadro à parte;
nenhum fio novo.

---

## O que o código faz hoje (conferido)

| ponto | como é | linha |
|---|---|---|
| CRC | CRC-16/CCITT-FALSE do **byte 4 até o fim do payload — o magic fica de fora** —, gravado little-endian | 802–803 |
| reverso | o master grava os pixels em ordem invertida e liga o bit 0 das flags; o receptor **não** desfaz, aplica como chegou | 779, 789–791 |
| canal W | somado ao R, G e B, saturando em 255 | 796–799 |
| brilho | o master manda o `bri` global; o receptor o aplica com `strip.setBrightness` | 782, 1007–1010 |
| trecho | o master copia `led-count` pixels a partir de `source-start`; o receptor escreve a partir de `target-start` | 788–795, 1013–1028 |
| **quadro maior que o receptor** | `N > led-count` do receptor é **erro de framing**: o quadro é jogado fora | 868 |
| magic | o parser do receptor só reconhece `WPB1` — outro magic é ignorado sem contar erro | 845–857 |
| sequência | o receptor conta buraco, duplicado e ressincronia pela sequência de 32 bits | 289–316 |
| limites | até 2048 LEDs, 115200–5.000.000 baud, até 120 quadros/s | 71–74, 1056–1064 |
| elétrica | o comentário do topo recomenda **transceptor diferencial** (MAX3485) nas duas pontas | 19–21 |

A linha 868 é o motivo de a mudança 3 ter de vir **antes** da 1: um receptor
de hoje recusa qualquer quadro com mais LEDs do que ele controla.

---

## Resumo

| # | onde | mudança |
|---|---|---|
| 1 | master | mandar **todos** os LEDs (`led-count` = total da fita) |
| 2 | master | mandar o quadro de segmentos **`WPS1`** a cada mudança e a cada 1 s |
| 3 | receptor | aceitar quadro maior e **recortar o seu trecho** (`source-start` + `led-count`) |
| 4 | receptor | **reverso** passa a ser do receptor, por trecho |
| 5 | os dois | `bridge-version` 1.5.0; ordem de atualização |

O formato do `WPB1` **não muda** (versão continua 1).

---

## 1. Master: mandar todos os LEDs

- **Nova opção `send-all`** (bool, padrão falso para não mudar quem já usa).
  Ligada, o master ignora `source-start`/`led-count` e usa
  `source-start = 0` e `led-count = strip.getLengthTotal()` em `startBridge()`
  — os buffers (`allocateBuffers`) já saem do tamanho certo.
- **O total pode mudar** (salvar a configuração de LEDs do WLED): em `loop()`,
  comparar `strip.getLengthTotal()` com o valor usado e, se mudou, pôr
  `_restartRequested = true`. Conferir se o WLED já chama `readFromConfig`
  do usermod nesse caso — se chamar, isto é redundante.
- **Limite:** o total tem de caber em `MAX_LED_COUNT` (2048). Se passar,
  `_lastError` e não sobe (como hoje).
- **`reverse` no master fica desligado** com `send-all` (ver a mudança 4). A
  ponte da Pi desfaz a flag se vier ligada, mas o receptor ficaria com o
  trecho errado.

**Banda.** Cada byte custa 10 bits na UART. Quadro = `18 + 3·N` bytes.

| LEDs | bytes | a 60 q/s | máximo a 2 Mbps |
|---|---|---|---|
| 600 (externo) | 1818 | 1,09 Mbps | 110 q/s |
| 1000 | 3018 | 1,81 Mbps | 66 q/s |
| 2048 | 6162 | 3,70 Mbps | 32 q/s |

`max-fps` ≤ `baud / (10 · (18 + 3·N))`. Acima disso o `txTaskLoop` não dá
conta e o master descarta quadros (`dropped` sobe) — funciona, mas é
desperdício. O número de LEDs do par interno ainda não foi informado.

---

## 2. Master: quadro de segmentos `WPS1`

Mesmo cabeçalho de 16 bytes e mesmo CRC do `WPB1` (sem o magic), com outro
magic:

| offset | tam. | campo |
|---|---|---|
| 0 | 4 | `"WPS1"` |
| 4 | 1 | versão (1) |
| 5 | 1 | flags: **bit 0 = WLED ligado** |
| 6 | 4 | sequência — **contador próprio**, separado do `WPB1` |
| 10 | 2 | **número de segmentos** (S) |
| 12 | 1 | `bri` global |
| 13 | 1 | reservado (0) |
| 14 | 2 | tamanho do payload = 7·S |
| 16 | 7·S | por segmento, abaixo |
| 16+7·S | 2 | CRC-16/CCITT-FALSE dos bytes 4 até aqui, little-endian |

Por segmento (7 bytes):

| tam. | campo | origem no WLED |
|---|---|---|
| 1 | id | índice do segmento |
| 2 | start | `seg.start` |
| 2 | stop | `seg.stop` (exclusivo) |
| 1 | on | `seg.on` |
| 1 | bri | `seg.opacity` |

`start`/`stop` são índices **lógicos**, no mesmo espaço do
`strip.getPixelColor(i)` que o `WPB1` usa — é o que permite à Pi recortar o
quadro de pixels por segmento.

**Regras:**

- **Só segmentos ativos** (`seg.isActive()`), com o id = índice no vetor, para
  bater com o `id` do `/json/state` do WLED — é por esse id que o app mapeia
  segmento → zona.
- **Quando:** ao mudar (comparar o payload montado com o último enviado) **e**
  a cada 1000 ms, para quem começar a ouvir no meio pegar a tabela. A ponte
  considera o controlador morto só por falta de `WPB1`; o `WPS1` periódico é
  para a tabela, não para o "vivo".
- **Sequência própria.** Se usar o contador do `WPB1`, cada `WPS1` abre um
  buraco de 1 na sequência vista pelo receptor: `missing-sequences` sobe à toa
  e `_consecutiveValidFrames` volta a 1 (linhas 895–899), o que atrasa a
  retomada com o `reacquire-guard`.
- **Um só escritor na UART.** Montar o `WPS1` no `handleOverlayDraw()` (mesma
  tarefa que monta o `WPB1`, onde ler os segmentos é seguro), num buffer
  próprio pequeno (16 + 7·32 + 2 = 242 bytes cobre 32 segmentos), marcar
  `_segPendente` sob o `_bufferMutex` e acordar a tarefa (`xTaskNotifyGive`).
  O `txTaskLoop` escreve o `WPS1` pendente **antes** do próximo `WPB1`. Nunca
  chamar `_uart->write` da tarefa principal — dois escritores intercalam bytes
  e corrompem os dois quadros.
- **"Ligado"**: conferir de onde tirar. No WLED o `bri` global costuma ir a 0
  quando desliga (e o anterior fica em `briLast`); se for assim, bit 0 =
  `bri > 0`.

Esboço (nomes da API do WLED 0.15 a conferir na compilação):

```cpp
static constexpr uint8_t SEG_MAGIC[4] = {'W', 'P', 'S', '1'};
static constexpr size_t SEG_MAX = 32;
uint8_t  _segFrame[HEADER_SIZE + 7 * SEG_MAX + CRC_SIZE];
size_t   _segFrameSize = 0;
bool     _segPendente = false;          // protegido pelo _bufferMutex
uint32_t _segSequence = 0;
uint16_t _segUltimoCrc = 0;
uint32_t _segUltimoEnvioMs = 0;

void buildSegmentsFrame() {             // chamado no handleOverlayDraw, no master
  uint8_t* p = _segFrame + HEADER_SIZE;
  uint16_t n = 0;
  for (uint8_t i = 0; i < strip.getSegmentsNum() && n < SEG_MAX; i++) {
    Segment& s = strip.getSegment(i);
    if (!s.isActive()) continue;
    p[0] = i;
    writeLe16(p + 1, s.start);
    writeLe16(p + 3, s.stop);
    p[5] = s.on ? 1 : 0;
    p[6] = s.opacity;
    p += 7; n++;
  }
  memcpy(_segFrame, SEG_MAGIC, 4);
  _segFrame[4] = PROTOCOL_VERSION;
  _segFrame[5] = bri > 0 ? 0x01 : 0x00;
  writeLe16(_segFrame + 10, n);
  _segFrame[12] = bri;
  _segFrame[13] = 0;
  writeLe16(_segFrame + 14, n * 7);
  // CRC do conteúdo SEM a sequência, só para detectar mudança
  const uint16_t conteudo = crc16(_segFrame + 10, 6 + n * 7, crc16(_segFrame + 5, 1));
  const uint32_t agora = millis();
  if (conteudo == _segUltimoCrc && agora - _segUltimoEnvioMs < 1000U) return;

  writeLe32(_segFrame + 6, _segSequence++);
  const size_t len = HEADER_SIZE + n * 7;
  writeLe16(_segFrame + len, crc16(_segFrame + MAGIC_SIZE, len - MAGIC_SIZE));
  if (!takeMutex(pdMS_TO_TICKS(2))) return;          // tenta de novo no próximo quadro
  _segFrameSize = len + CRC_SIZE;
  _segPendente = true;
  giveMutex();
  _segUltimoCrc = conteudo;
  _segUltimoEnvioMs = agora;
  if (_ioTaskHandle) xTaskNotifyGive(_ioTaskHandle);
}
```

No `txTaskLoop`, antes do laço dos buffers de pixels: copiar o `_segFrame`
para um buffer local sob o mutex, limpar `_segPendente`, soltar o mutex e
escrever a cópia. (Copiar evita que o `handleOverlayDraw` sobrescreva o
buffer durante o `write`.)

Custo: ~50 bytes a cada segundo com 4 segmentos — desprezível perto do `WPB1`.

**O receptor não precisa mudar para isto:** o parser dele só aceita `WPB1`
(linhas 845–857) e descarta o `WPS1` byte a byte, sem contar erro.

---

## 3. Receptor: recortar o seu trecho

Hoje o receptor exige `N ≤ led-count` e guarda o payload inteiro. Passa a
aceitar o quadro do tamanho que vier e guardar só a janela dele.

- **`source-start` passa a valer no receptor** (o campo já existe e já é
  salvo — hoje é "usado só pelo master", linha 1182). Texto da ajuda:
  "Receiver: primeiro LED do quadro recebido usado aqui".
- **`parseHeader` (linha 859):**
  - trocar `_rxLedCount > _ledCount` por
    `_rxLedCount > MAX_STREAM_LEDS` (ex. 8192; o limite real é o `uint16` do
    tamanho: 21.845 LEDs);
  - **remover** `_rxPayloadLength > _payloadSize` — o payload agora pode ser
    maior que o buffer;
  - manter `_rxPayloadLength == _rxLedCount * 3`;
  - novo: se `source-start + led-count > _rxLedCount` → **erro de framing**
    (quadro não contém o trecho: configuração errada, tem de aparecer nos
    contadores). Não aplicar pela metade.
- **`consumeRxByte`, `ReadPayload` (linha 936):** o CRC continua sobre
  **todos** os bytes; só copia para o buffer quem estiver na janela:

  ```cpp
  case RxState::ReadPayload:
    if (_rxFillBuffer >= 0 && _rxPayloadPos >= _winStart && _rxPayloadPos < _winEnd)
      _buffers[_rxFillBuffer][_rxPayloadPos - _winStart] = value;
    _rxPayloadPos++;
    _rxCrc = crc16Update(_rxCrc, value);
    ...
  ```

  com a janela, em bytes, calculada no `parseHeader` (ver o reverso abaixo):
  `_winStart = 3 · início`, `_winEnd = _winStart + 3 · led-count`.
- **`completeReceivedFrame` (linha 902):** `_rxMeta[...].ledCount = _ledCount`
  (o trecho), não o N do quadro. O `applyReceiverFrame` já escreve
  `meta.ledCount` pixels a partir de `target-start` — não muda.
- **Buffers** continuam do tamanho do trecho (`allocateBuffers`, linha 622):
  o receptor não gasta memória com os LEDs dos outros.
- **Buffer do driver RX (linha 694):** hoje sai do tamanho do quadro do
  receptor, que agora é menor que o quadro na linha. Calcular pelo quadro
  maior esperado — o mais simples é fixar no teto que já existe, 24.576
  bytes (4 quadros de 2048 LEDs).

---

## 4. Receptor: reverso por trecho

Com o master mandando tudo, o reverso do master inverteria **a fita inteira**
— o trecho de cada receptor mudaria de lugar. O reverso passa para o
receptor:

- **Nova opção `reverse` no receptor** (o campo já existe; hoje é só do
  master): inverte a ordem dos `led-count` pixels do trecho ao aplicar —
  `target-start + i` recebe o pixel `led-count − 1 − i` do trecho.
- **Se o quadro vier com a flag de reverso** (master antigo, ou `send-all`
  desligado), o receptor desfaz antes de recortar. O pixel lógico `j` está na
  posição `N − 1 − j` do fio, então a janela vira
  `início = N − source-start − led-count`, e o trecho sai invertido (combinar
  com o `reverse` do receptor: aplicar invertido se exatamente um dos dois
  estiver ligado).

A ponte da Pi já desfaz a flag do quadro inteiro: para ela o índice 0 é sempre
o LED lógico 0 do WLED.

---

## 5. Versão e ordem de atualização

- **`bridge-version` → `1.5.0`** (linha 1322). O `WPB1` continua versão 1.
- **Ordem:**
  1. **Receptores primeiro.** Com `source-start = 0` e o master antigo
     (N = `led-count`), o receptor novo se comporta como hoje.
  2. **Depois o master**, com `send-all` ligado e `reverse` desligado,
     acertando `source-start`/`reverse` de cada receptor.

  Na ordem inversa, o receptor antigo recusa todos os quadros (linha 868) e
  as fitas dele entram no failsafe.

---

## Elétrica: como a Pi entra na linha

O comentário do código recomenda **MAX3485 nas duas pontas**. O que fazer
depende de como está no carro — **a confirmar com o usuário**:

- **Linha RS-485 (A/B, com MAX3485):** a Pi **não** pode ligar no TX de 3,3 V
  do ESP32 depois do transceptor — na linha só há o par diferencial. A Pi
  ganha **o próprio MAX3485 só recebendo** (`DE` e `/RE` em nível baixo), com
  A e B em paralelo no par e o RO no RX da UART da Pi. RS-485 aceita vários
  receptores no mesmo par. **Terminação de 120 Ω só nas duas pontas da
  linha**: se a Pi ficar no meio, o módulo dela vai sem terminação (muitos
  módulos vêm com o resistor soldado — tirar).
- **UART direta de 3,3 V (TX → RX):** a derivação do PLANO.md — RX da Pi em
  paralelo com o RX do receptor, mais o terra comum. A 2 Mbps, só com fio
  curto.

---

## Como testar

- **Receptor, sem regressão:** no `/json/state` do receptor, bloco
  `"UART Pixel Bridge"` — `crc-errors` e `framing-errors` em 0,
  `missing-sequences` sem subir com o `WPS1` ligado (se subir, o `WPS1` está
  usando a sequência do `WPB1`), `receiver-state` `active`, e as fitas iguais
  a antes.
- **Recorte:** um efeito com gradiente na fita inteira do master; cada
  receptor tem de mostrar o seu pedaço do gradiente, no sentido certo.
- **Pi:** `python ponte/ponte.py` com a entrada
  `serial:/dev/ttyAMA2@2000000`. No `/json/info` da ponte, `validos` subindo e
  `crc_errado` parado; no `/json/state` da ponte, a tabela de segmentos igual
  à do `/json/state` do master.
- **Sem a Pi:** a mesma ponte roda no computador com um adaptador USB-serial
  que aguente 2 Mbps (ex. FT232R ou CP2102N) — `serial:COM5@2000000` — e o
  app com `?wled=externo@127.0.0.1:<porta>`.

## A conferir no WLED

- **Brilho no `getPixelColor`.** O app assume que o RGB vem **sem** o brilho
  mestre (como no preview por wifi) e multiplica pelo `bri`. O receptor faz o
  mesmo (aplica `bri` de novo, linha 1009), o que sugere que é assim — mas com
  `bri` baixo o WLED recupera a cor dividindo pelo brilho, com perda.
  Conferir pela ponte mudando o brilho, como foi feito no wifi.
- **Gama:** comparar o mesmo LED pelo preview do wifi e pela UART (PLANO.md,
  *A medir quando existir*).
