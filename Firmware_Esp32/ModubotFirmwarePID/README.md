# ModubotFirmwarePID — firmware de locomoção v2 (malha fechada)

Evolução do firmware anterior (`../Firmwere.ino`), incorporando a ideia de
controle em malha fechada do `ESP32.ino` (RoboDC), adaptada à arquitetura real do ModuBot:
driver ZS-X11H via DAC, protocolo ASCII em linha, e a odometria em ticks crus que o
`serial_odom_node` já consome.

## Princípio da arquitetura: cada constante existe em UM lugar só

| Constante | Natureza | Dono |
|---|---|---|
| `wheel_separation` | geometria (URDF, TF, AMCL) | **ROS** |
| `wheel_radius` | contínua, recalibrável (raio carregado) | **ROS** |
| `max_wheel_speed` | política de navegação | **ROS** |
| `ticks_per_rev`, PI e feedforward | identificação experimental | **YAML ROS, reaplicado no firmware** |
| rampa e watchdog | proteção do atuador | **firmware** |

O comando de movimento é explícito em unidade (`W 4.5 4.5` = rad/s). Após um reset da
ESP32, o bridge reaplica `C`, `K`, `F` e `M` antes de retomar o movimento.

O raio da roda não aparece no firmware: o feedback vai de `ticks → rev/s → rad/s`
usando só `ticks_per_rev`. Recalibrar o raio é mexer em um parâmetro do ROS, sem reflash.

## Arquivos

| Arquivo | Função |
|---|---|
| `ModubotFirmwarePID.ino` | Firmware (Arduino IDE, placa "ESP32 Dev Module") |
| `count_ticks.py` | Calibra `ticks_per_rev` contando pulsos de N voltas |
| `estima_tau.py` | Extrai `a`, `v_f` e `τ` dos brutos de uma campanha de calibração |
| `plot_pid.py` | Plot em tempo real + envio de comandos para sintonia |
| `analisa_degrau.py` | Extrai overshoot, t_subida, t_acomodação e e_regime do CSV |
| `../../modubot_ws/src/modubot_serial_bridge/modubot_serial_bridge/cmdvel_to_serial.py` | Ponte ROS 2 e proprietária da serial |
| `../../modubot_ws/src/modubot_serial_bridge/config/modubot_params.yaml` | Parâmetros compartilhados |

## Protocolo serial (115200, linhas ASCII)

### Entrada

| Comando | Efeito |
|---|---|
| `W <wL> <wR>` | **Comando principal.** Setpoint em **rad/s por roda** (malha fechada). |
| `V <nL> <nR>` | Fração do atuador [-1..1] → DAC direto. **Só em malha aberta**; em malha fechada é recusado com aviso. |
| `S` | Freio imediato + zera PI. |
| `M <0\|1>` | 0 = malha aberta, 1 = malha fechada (**padrão: 1**). |
| `K <kp> <ki> <kd>` | Ganhos PI — unidade: **DAC por rad/s**. |
| `F <kff> <dac_min>` | Feedforward: `u_ff = sign(sp)·(dac_min + kff·|sp|)`. |
| `C <ticksL> <ticksR>` | Bordas por volta usadas no feedback. |
| `P <0\|1> [hz]` | Liga/desliga telemetria (padrão 25 Hz). |
| `G` | Imprime a configuração atual. |

### Saída

| Linha | Conteúdo |
|---|---|
| `O <dL> <dR> <dt_ms>` | Odometria em **ticks crus**, publicada pelo bridge para o nó de odometria. |
| `T <ms> <spL> <wL> <uL> <dacL> <spR> <wR> <uR> <dacR>` | Telemetria: setpoint e medida em **rad/s**, `u`/`dac` em contagens com sinal. |
| `# ...` | Mensagens humanas — os nós ROS ignoram (só fazem parse de `O`). |

Por que `V` é recusado em malha fechada: se um bridge antigo mandar `V 0.100` achando
que é "10% de 0,6 m/s", a interpretação certa seria ambígua. Recusar faz o robô **não se
mexer** e imprimir o motivo — falha alta e segura, em vez de andar na velocidade errada.

## Odometria e `dt_ms`

O firmware emite **ticks crus**, não velocidade — assim o dado bruto fica preservado no
rosbag e o raio pode ser recalibrado offline. O `dt_ms` agora é o tempo real decorrido
(antes era a constante 50, o que inflava a velocidade calculada no ROS quando o loop
atrasava por causa do `delay(100)` da inversão de sentido).

## Correções em relação ao firmware antigo

- `dt_ms` real na odometria (era fixo em 50 ms).
- Inversão de sentido **não bloqueante** (era `delay(100)`, congelava serial e odometria).
- Parser com buffer `char` fixo + `strtof` (era `String` + `sscanf`).
- Watchdog freia e zera o PI uma única vez, com aviso `# WATCHDOG`.
- Filtro de glitch de 100 µs nas ISRs (bordas lentas do conversor 5 V→3,3 V geravam
  contagens fantasmas).
- Estimador de velocidade por **período entre bordas** em vez de contagem por janela —
  é o que torna o PI viável a 50 Hz (contar pulsos em 20 ms dá degraus de dezenas de RPM).

**Limitação mantida (hardware):** o pino S não informa sentido de giro; o sinal da
velocidade e da odometria vem da direção comandada, como no firmware antigo.

## Frequências

| Elo | Taxa |
|---|---|
| `/cmd_vel` | ~20 Hz (Nav2 / teleop) |
| Linha `W` na serial | 20 Hz (timer do bridge, também serve de keepalive) |
| **Malha PI na ESP32** | **50 Hz** |
| Odometria `O` | 20 Hz |
| Watchdog do firmware | 600 ms sem linha → freio |

Malha interna 2,5× mais rápida que o comando: cada setpoint é trabalhado por ~2–3 ciclos
do PI antes do próximo chegar.

## Identificação

As campanhas de 05–06/08/2026 (90 runs no chão com roda traseira fixa + 150 suspensos)
fornecem os valores iniciais abaixo. A contagem de bordas por volta deve ser confirmada
manualmente antes da validação final.

| Grandeza | Valor | Origem |
|---|---|---|
| `ticks_per_rev` | **91** | valor usado nas campanhas; confirmar em 10 voltas |
| `a` — ganho | **0,171 rad/s por DAC** | ajuste linear, R² = 0,9998 |
| `1/a` — `kff` | **5,9** | idem |
| `d` — zona morta | **7,4** (6,9 avanço / 7,9 ré) | intercepto do ajuste |
| `τ` — const. de tempo | **0,196 s** (0,207 avanço / 0,185 ré) | `estima_tau.py` sobre 120 transitórios |

```
F 5.9 7
K 6 30 0      # lambda = tau  (conservador — default do .ino)
K 12 60 0     # lambda = tau/2 (recomendado após validar)
```

O `KFF` fornece o comando inicial previsto pelo mapa; o PI corrige somente o que faltar.
Por exemplo, para `5 rad/s`, `F 5.9 7` inicia em aproximadamente
`7 + 5,9 × 5 = 36,5 DAC`. Sem feedforward, o PI teria de construir esse valor aos poucos
a partir do erro.

O `τ` foi extraído com `estima_tau.py`, que ajusta uma reta ao trecho de regime da
**distância acumulada** e lê o τ no cruzamento com zero. Isso contorna a quantização da
velocidade (1 tick / 50 ms = 0,108 m/s), que inviabiliza ajustar exponencial na
velocidade instantânea.

Dois fatos úteis desses dados:
- **τ no chão é ~3× o τ suspenso** (0,196 s contra 0,074 s). Ganhos derivados de ensaio
  suspenso ficariam agressivos demais no solo — mais uma razão para não usar bancada
  como referência de desempenho.
- A rampa `DAC_SLEW_PER_CYCLE = 12` leva 87 ms para chegar ao DAC 52 (regime de
  0,6 m/s), **abaixo do τ de 196 ms** — ou seja, a rampa não limita a resposta.

Para refazer a identificação em outra configuração (outra roda, outro piso):
```
python count_ticks.py COM5 --turns 10                       # ticks_per_rev
python estima_tau.py <pasta_da_campanha> --radius 0.078     # a, v_f e tau
```

## Roteiro de sintonia

**Fase 4 — validar o feedforward.** Com `K 0 0 0` e `M 1`, um `W 7 7` deve estabilizar
perto de 7 rad/s só com o FF. Se parar longe, a identificação não vale para a
configuração atual.

**Fase 5 — subir os ganhos.** Comece nos defaults (`K 6 30 0`), confirme estabilidade,
e vá para `K 12 60 0`. Ajuste fino lendo o gráfico (seção abaixo).

**Fase 6 — gravar** os ganhos bons no YAML e como defaults do `.ino`.

## Como ler os gráficos

O `plot_pid.py` mostra três painéis: velocidade L, velocidade R e a saída do controlador.
No terceiro painel há **duas curvas por roda**: `u` (o que o PI pediu) e `DAC` (o que foi
realmente aplicado, após a rampa). Elas se separarem significa que a **rampa**
`DAC_SLEW_PER_CYCLE` está limitando — não adianta subir `kp`.

Para os números, use o CSV:
```
python analisa_degrau.py ensaio1.csv --radius 0.078
```

| Sintoma no gráfico | Causa provável | Ação |
|---|---|---|
| Sobe devagar, erro de regime persiste | `kp` e `ki` baixos | subir ambos |
| Overshoot com oscilação **rápida** decrescente | `kp` alto | baixar `kp` ~30% |
| Oscilação **sustentada** rápida | `kp` muito alto ou ruído amplificado | baixar `kp`; subir filtro (`SPEED_ALPHA` menor) |
| Overshoot grande, oscilação **lenta** | `ki` alto | baixar `ki` ~50% |
| Chega perto e "arrasta" até o setpoint | `ki` baixo | subir `ki` |
| `u` colado em 255 e resposta truncada | setpoint acima do que o hardware entrega | reduzir setpoint (DAC só chega a 3,3 V de um throttle 0–5 V) |
| `u` e `DAC` separados na subida | rampa limitando | subir `DAC_SLEW_PER_CYCLE` |
| Roda pára e anda perto de velocidade zero | zona morta + integrador (ciclo-limite) | aumentar `dac_min` do feedforward |
| Medida serrilhada | poucos ticks ou `ticks_per_rev` errado | refazer Fase 0 |

**Alvos** para uma malha de velocidade que alimenta o Nav2: overshoot ≤ 10%, acomodação
≤ 0,5 s, erro de regime ≈ 0, sem oscilação sustentada.

O CSV do `--log` guarda os valores **crus em rad/s**, independente do `--radius` usado
na tela.

### Limitação atual: ganhos compartilhados

`K` e `F` valem para as duas rodas. Como os motores têm zona morta e ganho diferentes
(é justamente a assimetria medida na campanha de calibração), o ideal é `kff`/`dac_min`
por roda. Se a assimetria for relevante nos seus dados, vale estender o protocolo para
`FL`/`FR`.

## Lado ROS

O `cmdvel_to_serial.py` do pacote `modubot_serial_bridge` substitui o nó antigo:

```python
vL = vx - (B / 2.0) * wz          # cinemática diferencial (inalterada)
vR = vx + (B / 2.0) * wz
# saturação proporcional: escala AS DUAS rodas, preservando a curvatura do arco
# m/s -> rad/s: é aqui, e só aqui, que o raio entra
self.write_line(f"W {vL / R:.3f} {vR / R:.3f}")
```

Ele é o único proprietário da porta USB. As linhas recebidas da ESP32 são publicadas em
`/modubot/serial_rx`, e o `serial_odom_node.py` consome esse tópico. Isso evita abrir a
mesma porta em dois processos. O bridge também reaplica modo, ganhos e ticks após reset.

Para iniciar a base:

```
ros2 launch modubot_serial_bridge base.launch.py
# comparação em malha aberta:
ros2 launch modubot_serial_bridge base.launch.py closed_loop:=false
```

Para testar PI puro, use `kff: 0.0` e `dac_min: 0.0` no YAML do experimento.

Em malha aberta, o mesmo bridge envia `V`; em malha fechada, envia `W`. Comando zero,
timeout ou encerramento enviam `S`.

### Geometria correta

**`wheel_separation = 0.225 m`** e **`wheel_radius = 0.078 m`** — preenchidos em
`modubot_serial_bridge/config/modubot_params.yaml`, que configura os dois nós.

Os arquivos antigos divergiam e **ambos estavam errados**: o bridge usava bitola 0,28 m
(+24%) e o `serial_odom_node` usava 0,223 m (−0,9%). O erro do bridge afeta o ω
executado; o do odom, a pose estimada. Ensaios anteriores de trajetória e giro
carregam esse viés — vale reavaliar antes de reaproveitar números de campanhas antigas.

## Pinagem (inalterada)

| Função | GPIO |
|---|---|
| DAC esquerdo / direito | 25 / 26 |
| Direção esquerda / direita (open-drain) | 19 / 17 |
| Freio (HIGH = ON) | 16 |
| Pulso S esquerdo / direito | 35 / 34 |
