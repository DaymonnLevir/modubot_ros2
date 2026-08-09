# Ensaio de trajetória com parada por odometria

Este ensaio faz somente o necessário na Jetson: envia comandos aos motores,
integra os ticks das duas rodas e para o robô quando a odometria atinge o
comprimento solicitado. O celular grava a arena de cima; os AprilTags no chão e
no robô são medidos posteriormente no vídeo, de forma independente do ROS 2.

## Definição da distância usada para parar

Para cada amostra da ESP32:

```text
delta_centro = (delta_roda_esquerda + delta_roda_direita) / 2
progresso = soma(abs(delta_centro))
```

Em uma reta, `progresso` é a distância odométrica percorrida pelo centro do
robô. Em uma curva, é o comprimento odométrico do arco. O comando de parada é
enviado na primeira atualização em que esse valor alcança ou ultrapassa o alvo.
O CSV registra o pequeno excesso causado pela resolução discreta dos ticks.

O script também integra x, y e yaw pelo modelo diferencial, mas essas grandezas
não realimentam o movimento: servem para comparar depois com a trajetória
extraída do vídeo. Manter a rota depende dos comandos por roda produzidos pelo
mapa feedforward; não há correção visual online neste experimento.

## Bancada e vídeo

- Arena: 2,0 m x 1,2 m, com tags fixos no chão definindo escala e referencial.
- Um tag deve estar rigidamente fixado no robô, preferencialmente com o centro e
  a orientação medidos em relação a `base_link`.
- O celular deve ficar fixo, o mais perpendicular possível ao plano do chão, e
  não pode ser movido durante uma campanha.
- Todos os tags relevantes devem permanecer visíveis durante o trajeto.
- Trave foco, exposição e zoom do celular, se o aplicativo permitir, e use a
  maior taxa de quadros estável disponível.
- Grave uma régua ou duas distâncias conhecidas no plano da arena. Os tags do
  chão corrigem perspectiva; a medida conhecida valida a escala resultante.

Para uma reta de 1,5 m em uma arena de 2,0 m, o centro do robô deixa apenas
0,25 m em cada extremidade quando começa centralizado. Confirme que a metade do
comprimento do robô mais a distância de parada cabe nessa margem. Caso não
caiba, reduza o alvo ou aumente a área.

Para os primeiros arcos, use comprimento de 0,8 m e raio de 0,8 m. Isso gera um
giro ideal de aproximadamente 57,3 graus e deslocamento de cerca de 0,673 m por
0,368 m, compatível com a área quando o robô é bem posicionado.

## Arquitetura e segurança

`odometry_trajectory_experiment` abre `/dev/ttyUSB0` de forma exclusiva. Pare
`cmdvel_to_serial` e `serial_odom_node` antes de executá-lo, pois esses processos
disputariam a mesma porta.

Não há modo automático no chão. Antes de cada repetição o programa espera que o
operador reposicione o robô e pressione ENTER. `Ctrl+C`, ausência de telemetria
ou duração máxima excedida fazem o programa enviar repetidamente o comando de
parada.

## Compilação

```bash
cd /workspace/modubot_ws
colcon build --packages-select modubot_serial_bridge
source install/setup.bash
```

## Validação sem mover o robô

Este comando apenas calcula e mostra o plano e os comandos por roda:

```bash
ros2 run modubot_serial_bridge odometry_trajectory_experiment \
  --trajectories straight \
  --repetitions 1 \
  --distance 1.5 \
  --linear-speed 0.25 \
  --control-mode feedforward \
  --feedforward-map \
  /workspace/modubot_ws/calibration_data/combined_maps_20260806_235048/inverse_map_seed.csv \
  --dry-run
```

## Piloto retilíneo

Comece a gravação do celular, informe o nome do vídeo e rode apenas uma vez:

```bash
ros2 run modubot_serial_bridge odometry_trajectory_experiment \
  --port /dev/ttyUSB0 \
  --trajectories straight \
  --repetitions 1 \
  --distance 1.5 \
  --linear-speed 0.25 \
  --control-mode feedforward \
  --feedforward-map \
  /workspace/modubot_ws/calibration_data/combined_maps_20260806_235048/inverse_map_seed.csv \
  --ticks-left 91 \
  --ticks-right 91 \
  --wheel-radius-left 0.078 \
  --wheel-radius-right 0.078 \
  --wheel-separation 0.225 \
  --surface laboratorio \
  --video-file VID_001.mp4 \
  --campaign-name pilot_odom_straight
```

Ao pressionar ENTER para uma repetição, o terminal mostra o `run_id`, faz uma
contagem regressiva e registra horários UTC de início e parada. O instante em
que as rodas começam e param também permite alinhar visualmente o vídeo com os
dados. É melhor manter um único vídeo contínuo para a campanha e não interromper
a gravação entre repetições.

## Campanha recomendada

Depois de verificar o piloto, uma campanha com dez repetições por geometria:

```bash
ros2 run modubot_serial_bridge odometry_trajectory_experiment \
  --port /dev/ttyUSB0 \
  --trajectories all \
  --repetitions 10 \
  --distance 1.5 \
  --arc-length 0.8 \
  --arc-radius 0.8 \
  --linear-speed 0.25 \
  --control-mode feedforward \
  --feedforward-map \
  /workspace/modubot_ws/calibration_data/combined_maps_20260806_235048/inverse_map_seed.csv \
  --order randomized \
  --seed 42 \
  --surface laboratorio \
  --video-file article_apriltag.mp4 \
  --campaign-name article_odom_feedforward
```

A ordem é aleatória e reproduzível para distribuir efeitos de bateria,
aquecimento e variação do piso entre retas e curvas. Se reposicionar curvas
aleatoriamente for muito trabalhoso, use `--order blocked`; registre essa decisão
no método.

## Arquivos coletados

Cada campanha contém:

- `metadata.json`: argumentos, vídeo associado, ordem e comandos exatos;
- `summary.csv`: ponto de parada, excesso odométrico, duração e pose final;
- `samples/<run_id>.csv`: ticks, velocidades, comandos, odometria integrada e
  trajetória ideal em cada amostra.

Na análise posterior do vídeo, extraia para cada frame o centro e a orientação
do tag do robô no referencial definido pelos tags do chão. Para cada repetição,
compare:

1. deslocamento físico início--fim com os 1,5 m odométricos nas retas;
2. erro longitudinal e lateral do ponto final;
3. erro angular final;
4. RMSE e máximo do erro transversal ao longo da rota;
5. comprimento do arco e erro radial nas curvas;
6. trajetória visual contra x, y e yaw registrados no CSV;
7. média, desvio-padrão e intervalo de confiança entre repetições.

O comprimento bruto quadro a quadro do vídeo tende a ser superestimado pelo
ruído do detector. Use uma trajetória suavizada para comprimento de caminho,
mas preserve e reporte o deslocamento direto início--fim como métrica principal
da reta. Guarde também os dados brutos para permitir reprocessamento.
