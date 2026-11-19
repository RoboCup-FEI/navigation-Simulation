# robocup_ws

> Táto vetva je priamym pokračovaním Filipovej vetvy `ros2-jazzy-isaac-sim`.

Simulácia robota MRVK „shoddy“ v Isaac Sim, v aréne RoboCup@Home HCC alebo
v prázdnom svete, s Nav2, predikciou prekážok a nasledovaním človeka.
ROS 2 Jazzy · Isaac Sim 6.2 · Ubuntu 24.04

## Inštalácia

```bash
git clone <toto repo> ~/robocup_ws && cd ~/robocup_ws
sudo apt install python3-rosdep ros-jazzy-libg2o ros-jazzy-pointcloud-to-laserscan
rosdep update && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
```

Isaac Sim sa očakáva v `~/isaacsim` (inak `ISAACSIM_PATH`). Workspace sa
zbuilduje pri prvom `./run.sh`.

## Spustenie

```bash
./run.sh                          # aréna, robot, RViz, ovládanie cez rqt
./run.sh --nav [--planner dwb]    # + Nav2 (predvolene TEB), cieľ cez 2D Goal Pose
./run.sh --nav --slam             # + slam_toolbox (len mapa pre RViz)
./run.sh --predictor              # + segmentácia zeme a predictor prekážok
./run.sh --world empty --person "2,-5 12,-5 12,5 2,5" --person-speed 0.8 --follow
./run.sh --model v1 | --headless | --x 2 --y -1 --yaw 1.57 | --build
```

`run.sh` spustí Isaac v čistom prostredí, počká na `/clock` a potom spustí ROS
stranu. ROS ide cez Fast DDS, na prezeranie tém z vlastného terminálu treba
`export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`. Prvé spustenie s `--person`
stiahne postavu z internetu a Isaac potom potrebuje ~2 GB RAM navyše.

## Čo beží

- **Isaac** (`src/robocup_sim/isaac`): aréna alebo prázdna podlaha, robot
  model v2 (Hesai XT16, kamera Basler, IMU z Livoxu) na kinematickom podvozku
  riadenom cez `/cmd_vel`. `/odom` je presná svetová póza, rámec `odom` je
  rámec arény.
- **Chodec** (`--person`): postava z IsaacSim Replicator Agent, ktorá chodí
  dokola po zadaných bodoch po navmeshi.
- **Vnímanie** (`--predictor`): Hesai → `ground_segmentation_ros2` →
  `dynamic_obstacle_predictor` (`src/obstacle_predictor`). Predictor sleduje
  pohybujúce sa objekty, publikuje ich predikované dráhy a pre každý TF
  `agent_<id>`.
- **Lokálna costmapa**: obstacle layer, `PeopleFilterLayer` a `SocialLayer`
  zo social_nav2 (sociálna zóna okolo človeka), `PredictedPathObstacleLayer`
  a inflácia.
- **Nasledovanie** (`--follow`): `person_follower.py` posiela Nav2 ciele 1 m
  od detegovaného človeka na spojnici s robotom.

## Zmeny v tejto vetve

**Filip**
- Port z Humble na Jazzy (TEB, costmap_converter, konfigurácie Nav2
  a slam_toolbox).
- Model robota v2 podľa aktuálneho CADu so simulovaným Hesai, kamerou a IMU;
  pôvodný model ostáva ako `--model v1`.
- Výška kinematického podvozku z URDF, nulové trenie, fyzika na 200 Hz.

**Adam**
- Kinematický podvozok sa zapína pred prvým krokom fyziky a odometria je
  svetová póza, takže scéna sa už nenakláňa ani neotáča.
- Predvolený spawn presunutý na (1, 0); v (0, 0) robot zasahoval do rámu dverí.
- Posuvné dvere majú presnú kolíziu; pôvodný kváder bol o 10 cm hrubší, než
  vidí lidar, a robot sa o ne zasekával.
- `/joint_states` z Isaacu s pečiatkou odometrie, kolesá v RVize už
  nezaostávajú za telom.
- Prázdny svet a animovaný chodec.
- Predictor prekážok a segmentácia zeme skopírované z MRVK workspace,
  napojené na Hesai; predictor publikuje TF `agent_<id>`.
- `PredictedPathObstacleLayer` v lokálnej costmape.
- Vrstvy social_nav2 portované na Jazzy vrátane opráv: starí ľudia sa
  zahadzujú a za človekom neostáva stopa zón (zmena `clearArea` v Jazzy).
- Nasledovanie človeka.
- slam_toolbox len s `--slam`, lebo do mapy zapisoval prechádzajúcich ľudí
  ako steny.

## Známe obmedzenia

- Navigácia beží v `odom`, globálna lokalizácia nie je.
- Kolízia chodca je len trigger, robot cez neho prejde.
- Predikcia a sociálna zóna existujú len pri pohybujúcom sa človeku.
- Isaac občas spadne na chybe GPU `DEVICE_LOST`(klasika)_
