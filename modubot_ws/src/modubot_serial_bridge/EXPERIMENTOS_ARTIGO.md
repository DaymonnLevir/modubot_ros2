# Roteiro executável dos experimentos do artigo

Os programas desta pasta preservam os dados brutos, geram um resumo por
execução e registram a tensão da bateria recebida da ESP32 no formato
`BAT:<volts>`. Antes de usar qualquer programa que abra a porta serial
diretamente (A, B ou C), pare o `base.launch.py` e o Nav2. Somente um processo
pode abrir a porta da ESP32.

Em modo interativo, `R` no prompt seguinte repete a última execução. A tentativa
anterior não é apagada: recebe o estado `repeated_by_operator`, e a nova recebe
o sufixo `_A02`, `_A03` etc. Assim, o incidente permanece auditável e é
excluído da análise principal.

Durante cada execução de A ou C, pressione `ESPAÇO` ou `E` para enviar o freio sem
precisar de Enter. A tentativa parcial recebe o estado
`emergency_stop_by_operator`, não entra na análise principal e o programa
oferece repetir o mesmo passo depois do reposicionamento.

## Preparação

```bash
cd /workspace/modubot_ws
colcon build --packages-select modubot_battery_monitor modubot_serial_bridge
source install/setup.bash
```

Confirme a porta da ESP32 antes de começar. Os exemplos usam `/dev/ttyUSB0`.
Use primeiro `--dry-run`, que valida e mostra o plano sem abrir a serial.
Para haver dados de bateria, carregue o firmware
`Firmware_Esp32/ModubotFirmwarePID_Battery/ModubotFirmwarePID_Battery.ino`.

## A — Caracterização dos atuadores em malha aberta

O plano recomendado contém oito níveis, cinco repetições e os dois sentidos:
80 execuções por campanha. Faça campanhas separadas para a condição mecânica
que será comparada (por exemplo, alinhamento inicial das casters). O CSV de cada
execução contém todos os ticks, velocidades instantâneas, comando e bateria. O
programa envia `M 0` e exige a confirmação `modo=ABERTA` da ESP32 antes de
iniciar; comandos rejeitados pelo firmware abortam a campanha com o robô
freado.

```bash
ros2 run modubot_serial_bridge feedforward_calibration \
  --port /dev/ttyUSB0 \
  --direction both \
  --mode both \
  --levels 0.02,0.03,0.04,0.05,0.075,0.10,0.20,0.30 \
  --repetitions 5 \
  --command-time 6 \
  --ticks-left 91 --ticks-right 91 \
  --wheel-radius-left 0.078 --wheel-radius-right 0.078 \
  --wheel-separation 0.207 \
  --surface laboratorio \
  --campaign-name article_A_open_loop
```

Use `--automatic` apenas com o robô suspenso e mecanicamente seguro. No chão,
mantenha o modo interativo para reposicionar as casters e o robô.

## B — Resposta ao degrau: malha aberta versus PI

O programa intercala a condição de referência em malha aberta e o PI embarcado.
Com três velocidades, dois sentidos, duas condições e cinco repetições são 60
execuções. Ele ativa a telemetria `T` do firmware e calcula média estacionária,
erro, RMSE, tempo de subida e sobressinal para cada roda, além da bateria.

```bash
ros2 run modubot_serial_bridge pi_step_experiment \
  --port /dev/ttyUSB0 \
  --speeds 0.10,0.20,0.30 \
  --direction both \
  --repetitions 5 \
  --baseline-mode raw \
  --kp 12 --ki 40 --kd 0 --kff 0 --dac-min 0 \
  --pre-time 2 --command-time 6 --post-time 2 \
  --steady-start 3 \
  --wheel-radius-left 0.078 --wheel-radius-right 0.078 \
  --surface laboratorio \
  --campaign-name article_B_pi_steps
```

O PI recebe velocidades por roda em rad/s; a referência em malha aberta recebe
comandos normalizados. Para usar o mapa inverso como referência, selecione
`--baseline-mode feedforward --feedforward-map <inverse_map_seed.csv>`.

## C — Trajetórias com verdade de campo externa

O celular e os AprilTags fornecem a medição física posteriormente. O programa
manda executar a geometria escolhida e para por odometria: 1,5 m para a reta,
ângulo acumulado para rotações e comprimento de caminho para arcos. Antes de
cada execução ele espera o reposicionamento e a confirmação por ENTER.

Retas, dez repetições:

```bash
ros2 run modubot_serial_bridge odometry_trajectory_experiment \
  --port /dev/ttyUSB0 \
  --trajectories straight \
  --repetitions 10 \
  --distance 1.5 \
  --linear-speed 0.20 \
  --control-mode pi \
  --kp 12 --ki 40 --kd 0 --kff 0 --dac-min 0 \
  --ticks-left 91 --ticks-right 91 \
  --wheel-radius-left 0.078 --wheel-radius-right 0.078 \
  --wheel-separation 0.207 \
  --surface laboratorio \
  --video-file article_C_straight_pi.mp4 \
  --campaign-name article_C_straight_pi
```

Rotações de 90 graus, cinco por sentido:

```bash
ros2 run modubot_serial_bridge odometry_trajectory_experiment \
  --port /dev/ttyUSB0 \
  --trajectories rotation_left,rotation_right \
  --repetitions 5 \
  --rotation-angle-deg 90 \
  --angular-speed 0.40 \
  --control-mode pi \
  --kp 12 --ki 40 --kd 0 --kff 0 --dac-min 0 \
  --video-file article_C_rotation_pi.mp4 \
  --campaign-name article_C_rotation_pi
```

Arcos de 90 graus com raio de 0,75 m, cinco por sentido:

```bash
ros2 run modubot_serial_bridge odometry_trajectory_experiment \
  --port /dev/ttyUSB0 \
  --trajectories arc_left,arc_right \
  --repetitions 5 \
  --arc-radius 0.75 \
  --arc-length 1.178097 \
  --linear-speed 0.15 \
  --control-mode pi \
  --kp 12 --ki 40 --kd 0 --kff 0 --dac-min 0 \
  --video-file article_C_arc_pi.mp4 \
  --campaign-name article_C_arc_pi
```

Figura em oito contínua, cinco repetições. Cada execução completa uma volta à
esquerda e outra à direita sem parar no cruzamento central. O sentido inicial
pode ser trocado para `right`; `--figure-eight-cycles 2` repete o desenho duas
vezes dentro da mesma execução para medir acúmulo de erro.

```bash
ros2 run modubot_serial_bridge odometry_trajectory_experiment \
  --port /dev/ttyUSB0 \
  --trajectories figure_eight \
  --repetitions 5 \
  --figure-eight-radius 0.35 \
  --figure-eight-cycles 1 \
  --figure-eight-start-direction left \
  --linear-speed 0.15 \
  --control-mode pi \
  --kp 12 --ki 40 --kd 0 --kff 0 --dac-min 0 \
  --max-duration 60 \
  --video-file article_C_figure_eight_pi.mp4 \
  --campaign-name article_C_figure_eight_pi
```

Repita as quatro campanhas com `--control-mode raw` para obter a comparação sem
PI. O modo `--automatic --inter-run-wait <segundos>` existe para bancada segura,
mas não deve ser usado quando o robô precisa ser reposicionado no chão.

## D — Navegação ROS 2/Nav2

O Nav2 e a base devem estar ativos. O `base.launch.py` agora também inicia o
monitor de bateria; portanto, `/battery/voltage`, `/battery/percentage` e
`/battery_state` ficam disponíveis. O gravador não escolhe o objetivo: o
operador define a pose inicial e envia sempre o mesmo objetivo pelo RViz. Cada
missão fica em uma rosbag independente.

Exemplo da condição PI, no primeiro terminal:

```bash
ros2 launch modubot_nav2 nav2.launch.py \
  map:=/workspace/modubot_ws/src/modubot_nav2/maps/PisoInferiorDC.yaml \
  closed_loop:=true \
  use_rviz:=true
```

Para a condição de referência sem PI, use o mesmo comando com
`closed_loop:=false`. Mantenha mapa, pose inicial, objetivo, obstáculos e demais
parâmetros iguais entre as condições.

Em outro terminal, para dez missões em ambiente com obstáculos e PI:

```bash
ros2 run modubot_serial_bridge nav2_experiment_recorder \
  --scenario obstacles \
  --control-condition pi \
  --repetitions 10 \
  --campaign-name article_D_nav2
```

Faça campanhas separadas para `--scenario free|obstacles` e
`--control-condition baseline|pi`. No final de cada missão, ENTER registra
sucesso, `f` falha, `r` conserva a tentativa e repete, e `q` encerra.

O gravador salva comandos antes e depois do monitor de colisão, odometria,
AMCL, partículas, scans, planos global e local, TF, diagnósticos, zonas de
colisão, bateria e feedback/status da ação de navegação. Isso permite calcular
offline sucesso, duração, comprimento de caminho, erro de localização,
suavidade do comando, eventos de parada/desaceleração e consumo aproximado de
tensão. O resultado externo de colisões e intervenções continua sendo anotado
pelo observador.

## Organização dos dados

Cada campanha contém `metadata.json`, `summary.csv` e dados brutos. A, B e C
guardam um CSV por tentativa; D guarda uma rosbag por tentativa. Não edite os
CSV brutos. As análises do artigo devem usar apenas linhas com `status=completed`
e relatar separadamente falhas e repetições.
