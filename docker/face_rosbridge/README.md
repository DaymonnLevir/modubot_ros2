# Rosbridge do aplicativo RoboDC

O aplicativo em `RoboDC/src/app/services/face-api.service.ts` conecta ao
rosbridge na porta `9090` e publica em `/robot/face_state`. A mensagem é
`std_msgs/String`; seu conteúdo é um objeto JSON com `talking`, `dir`,
`blink`, `exp`, `color`, `pauseLook` e `pauseBlink`.

Na Jetson, construa e mantenha o serviço no domínio ROS 2 da base:

```bash
docker build -t modubot-face-rosbridge:humble docker/face_rosbridge
docker run -d \
  --name modubot-face-bridge \
  --restart unless-stopped \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  -e ROSBRIDGE_PORT=9090 \
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
