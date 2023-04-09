# AutoModelCar_2023_UPV
Código del equipo Jaguares de la Universidad Politécnica de Victoria para la competencia AutoModelCar del Torneo Mexicano de Robótica 2023 organizado por la Federación Mexicana de Robótica con sede en la Universidad Tecnológica del Sureste de Veracruz.

>Directorios y paquetes

- Bring_up. Contiene los archivos .launch para poder cargar los nodos necesarios. De momento solo se tiene la sintáxis para poder modificarse posteriormente.
- Control. Contiene paquetes para poder conducir el vehiculo
  - Keyboard control. Se suscribe a los tópicos de velocidad y giro para poder controlarse mediante el teclado.
  - Steering control. Publica los valores de steering y velocidad para la conducción automática.
- Hardware. Contiene paquetes que conllevan una suscripción hacia la cámara.
  - Video capture. Publica la imagen percibida por la cámara a un tópico de ROS.
  - Video record. A partir de un tópico de imagen graba lo que se publica y exporta en un archivo .avi.
- Perception. Contiene paquetes que procesan la información percibida para poder arrojar datos que servirán para tomar control sobre el vehículo.
  - Line detector. Detecta los bordes del carril.
  - Signal detector. Detecta señalizaciones de stop en la pista.

>Comandos básicos sobre el Workspace
```
catkin_make
source devel/setup.zsh
rosrun package package_node
```

## Rutinas para el vehículo
>Grabación archivo [avi]
```
rosrun video_capture video_capture_node
rosrun video_record video_record_node

```
>Navegación sin obstáculos
```
rosrun video_capture video_capture_node
rosrun line_detector line_detector_node
rosrun signal_detector signal_detector
rosrun steering_control steering_control
```

## Nota
Fue modificado el archivo de autostart, por ello el archivo Live_UPV.launch se ejecuta al encender el vehículo, es decir, no es necesario ejecutar el comando manualmente.
```
roslaunch autominy Live_UPV.launch
```
Es necesario cargar las variables de entorno en cada terminal para el correcto funcionamiento de los paquetes de ROS desarrollados.
```
source devel/setup.zsh
```
[Arquitectura propuesta para el vehículo autónomo](https://lucid.app/publicSegments/view/bd8d4c51-0755-41a0-af4e-4752ef2aff18/image.pdf)
