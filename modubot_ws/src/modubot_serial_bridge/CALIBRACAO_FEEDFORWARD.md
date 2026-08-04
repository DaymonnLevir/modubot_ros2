# Calibracao feedforward do ModuBot em solo

Este procedimento mede, sob a carga real do robo, a relacao entre o comando
normalizado aplicado a cada driver e a velocidade linear estimada de cada roda.
O ensaio envia o mesmo patamar para as duas rodas, registra todas as amostras
produzidas pela ESP32 e aguarda confirmacao do operador antes de cada repeticao.

## Por que existe um executavel dedicado

No codigo atual, `cmdvel_to_serial` e `serial_odom_node` tentam abrir a mesma
porta (`/dev/ttyUSB0`). Durante a calibracao, `feedforward_calibration` substitui
temporariamente os dois e deve ser o unico processo conectado a essa porta. Isso
garante que comando e telemetria pertençam a uma mesma execucao e evita disputa
por bytes da serial.

O protocolo confirmado no firmware e:

- `V <left> <right>`: comandos normalizados entre -1 e 1;
- `S`: parada/freio;
- `O <ticks_left> <ticks_right> <dt_ms>`: telemetria da ESP32.

O firmware converte a magnitude do comando para DAC de 8 bits. Assim, a variavel
independente correta e o comando normalizado (ou DAC 0--255). A coluna de tensao
nominal e apenas `DAC / 255 * 3.3 V`; ela nao e uma medida com multimetro e nao
representa a alimentacao de 36 V do motor.

## Preparacao na Jetson

Pare Nav2, teleoperacao, `cmdvel_to_serial`, `serial_odom_node` e qualquer outro
programa que use a ESP32. Confirme a porta antes de iniciar.

```bash
cd ~/modubot_ros2/modubot_ws
sudo apt update
sudo apt install -y python3-serial python3-matplotlib
source /opt/ros/humble/setup.bash
colcon build --packages-select modubot_serial_bridge
source install/setup.bash
```

Teste o plano sem abrir a porta nem mover o robo:

```bash
ros2 run modubot_serial_bridge feedforward_calibration --dry-run
```

## Seguranca e configuracao do ensaio

- Use uma area plana, seca, livre e maior que a distancia percorrida no patamar
  mais alto.
- Trabalhe com duas pessoas: uma no terminal e outra junto ao freio fisico.
- Comece com niveis baixos. Interrompa com `Ctrl+C` se a trajetoria for insegura;
  o programa envia `S` repetidamente antes de fechar a serial.
- Mantenha superficie, pneus, massa embarcada e estado da bateria documentados e
  tao constantes quanto possivel.
- Confirme pulsos por volta e raio efetivo. Os valores brutos de ticks e tempo
  ficam guardados para que a velocidade possa ser recalculada depois.

Os padroes iniciais sao conservadores: nove niveis de 0.05 a 0.30, tres
varreduras completas em ordem crescente, 2 s parado, 8 s comandado e 2 s
parado. A media de regime usa o intervalo de 3.0 a 7.5 s dentro do patamar.

Campanha para frente:

```bash
ros2 run modubot_serial_bridge feedforward_calibration \
  --port /dev/ttyUSB0 \
  --direction forward \
  --mode both \
  --levels 0.05,0.075,0.10,0.125,0.15,0.175,0.20,0.25,0.30 \
  --repetitions 3 \
  --command-time 8 \
  --ticks-left 91 --ticks-right 91 \
  --wheel-radius-left 0.078 --wheel-radius-right 0.078 \
  --wheel-separation 0.225 \
  --surface "concreto liso" \
  --robot-mass-kg VALOR_MEDIDO \
  --battery-start-v VALOR_MEDIDO \
  --campaign-name ground_forward
```

Depois, execute uma campanha separada em re:

```bash
ros2 run modubot_serial_bridge feedforward_calibration \
  --port /dev/ttyUSB0 \
  --direction reverse \
  --mode both \
  --levels 0.05,0.075,0.10,0.125,0.15,0.175,0.20,0.25,0.30 \
  --repetitions 3 \
  --campaign-name ground_reverse
```

O programa exige a palavra `INICIAR`. Antes de cada execucao:

- `Enter`: inicia o patamar indicado;
- `s`: registra o patamar como pulado;
- `q`: encerra a campanha preservando tudo que ja foi coletado.

Reposicione fisicamente o robo, alinhe-o e libere a pista antes de pressionar
`Enter`. Nao segure nem empurre o robo durante a medicao.

## Dados gerados

Cada campanha e salva em
`~/calibracao_feedforward_modubot/<nome>_<data-hora>/`:

- `metadata.json`: parametros e plano completo;
- `summary.csv`: media, mediana, desvio, extremos e assimetria por execucao;
- `samples/<run_id>.csv`: todos os ticks e todas as velocidades ao longo do
  pre-stop, comando e pos-stop.

A velocidade de cada roda e calculada a partir dos ticks:

`v = 2*pi*raio*ticks / (ticks_por_volta*dt)`.

Ela e velocidade periferica estimada da roda, nao uma medicao externa do
deslocamento do chassi. O teste em solo incorpora carga, rolamento e a resposta
do conjunto mecanico/eletrico, mas derrapagem exige uma referencia externa
(por exemplo, mocap, video calibrado ou localizacao LiDAR) se tambem for
necessario medir a velocidade real do corpo.

## Geracao dos quatro mapas

Analise conjuntamente as campanhas para frente e em re com:

```bash
ros2 run modubot_serial_bridge analyze_feedforward_calibration \
  --campaign \
  ~/calibracao_feedforward_modubot/ground_forward_AAAAMMDD_HHMMSS \
  ~/calibracao_feedforward_modubot/ground_reverse_AAAAMMDD_HHMMSS
```

Sao gerados `map_points.csv`, `inverse_map_seed.csv`, o grafico conjunto de L+,
R+, L- e R-, e os graficos temporais por execucao. Com duas campanhas, o
resultado vai para uma nova pasta `combined_maps_<data-hora>` ao lado delas.

Nao use automaticamente a tabela inversa no controle. Primeiro verifique a
zona morta, monotonicidade, dispersao entre repeticoes, diferenca esquerda/direita
e possivel histerese. A descricao cientifica deve chamar o sinal contado de
"feedback de velocidade do driver" enquanto a origem Hall nao estiver
fisicamente confirmada.
