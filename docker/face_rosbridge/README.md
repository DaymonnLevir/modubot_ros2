# Rosbridge da face do ModuBot

A aplicação Rive em `RoboDC_face/teste.html` conecta em
`ws://localhost:19090` e assina `/robot/face_state` como `std_msgs/String`.
O conteúdo da string é um objeto JSON com `talking`, `dir`, `blink`, `exp`,
`color`, `pauseLook` e `pauseBlink`.

Na Jetson, construa e mantenha o serviço no domínio ROS 2 da base:

```bash
docker build -t modubot-face-rosbridge:humble docker/face_rosbridge
docker run -d \
  --name modubot-face-bridge \
  --restart unless-stopped \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  -e ROSBRIDGE_PORT=19090 \
  modubot-face-rosbridge:humble
```

Depois de compilar e carregar o workspace, envie um estado de teste:

```bash
ros2 run modubot_serial_bridge face_command \
  --expression happy \
  --direction center
```

Expressões: `neutral`, `happy`, `sad`, `scared`, `surprised`, `angry`,
`disgusted` e `sleepy`. Direções: `center`, `n`, `ne`, `e`, `se`, `s`, `sw`,
`w`, `nw` e as variantes com `+`.
