# tb3_projet_TD

Package ROS 1 (catkin, C++) pour piloter un **TurtleBot3** : contrôle manuel au clavier, navigation autonome avec évitement d'obstacles (LiDAR), détection de zones par secteurs et serveur/client d'action `Move`.

## Fonctionnalités

- **Contrôleur** (`controleur`) : machine à états avec deux modes, *manuel* et *autonome*.
- **Lecture clavier** (`key_input_node`) : publie les touches pressées sur `/keyboard_input`.
- **Détecteur** (`detector`) : découpe le scan LiDAR en 8 régions de 45° et publie des markers RViz.
- **Service** `set_behavior` : changer de mode à la volée (manuel / auto standard / auto turbo).
- **Action** `Move` : envoi d'un objectif `(x, y)` avec feedback (position, vitesses, distance restante) et résultat (distance totale, temps écoulé).
- **Serveur / client d'action** (`serveur_action`, `client_action`) : exemple d'utilisation de l'action ; le client reçoit une trajectoire sur `/consigne_trajectoire`.

## Structure

```
tb3_projet_TD/
├── action/Move.action          # Goal / Result / Feedback
├── srv/set_behavior.srv        # Changement de mode
├── config/
│   ├── parametres.yaml         # Fréquence de contrôle, mode par défaut
│   └── tb3_lidar.rviz          # Configuration RViz
├── launch/
│   ├── run.launch              # controleur + key_input_node + detector
│   └── rviz_tb3.launch         # bringup TurtleBot3 distant + RViz
├── src/                        # Nœuds C++
├── CMakeLists.txt
└── package.xml
```

## Prérequis

- ROS 1 (Noetic recommandé) et un workspace catkin
- Paquets TurtleBot3 (`turtlebot3_bringup`, ...) et `export TURTLEBOT3_MODEL=burger`
- Dépendances : `actionlib`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`, `tf`, `tf2`, `tf2_ros`, `visualization_msgs`

## Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/thomasdegoul/tb3_projet_TD.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Utilisation

```bash
# 1. Robot (ou simulation) + RViz
roslaunch tb3_projet_TD rviz_tb3.launch model:=burger

# 2. Contrôleur, clavier et détecteur
roslaunch tb3_projet_TD run.launch
```

### Commandes clavier (mode manuel)

| Touche | Action |
|--------|--------|
| `z` / `s` | Avancer / reculer |
| `q` / `d` | Pivoter à gauche / à droite |
| `Espace` | Arrêt |
| `+` / `-` | Vitesse linéaire +/- 0.01 m/s (max 0.15) |
| `*` / `/` | Vitesse angulaire +/- 0.1 rad/s (max 2.84) |
| `r` | Reset de l'odométrie |
| `m` | Basculer manuel ↔ autonome |

### Changer de mode par service

```bash
rosservice call /set_behavior "mode: 0"   # manuel
rosservice call /set_behavior "mode: 1"   # auto standard (0.1 m/s)
rosservice call /set_behavior "mode: 2"   # auto turbo (0.15 m/s)
```

### Action `Move`

```bash
rosrun tb3_projet_TD serveur_action
rosrun tb3_projet_TD client_action
```

## Topics

| Topic | Type | Sens | Nœud |
|-------|------|------|------|
| `/keyboard_input` | `std_msgs/Int8` | pub | `key_input_node` |
| `/odom` | `nav_msgs/Odometry` | sub | `controleur` |
| `/scan` | `sensor_msgs/LaserScan` | sub | `controleur`, `detector` |
| `/cmd_vel` | `geometry_msgs/Twist` | pub | `controleur` |
| `/reset` | `std_msgs/Empty` | pub | `controleur` |
| `/log` | `std_msgs/String` | pub | `controleur` |
| `/lidar_visualization` | `visualization_msgs/MarkerArray` | pub | `detector` |
| `/consigne_trajectoire` | — | sub | `client_action` |

## Paramètres (`config/parametres.yaml`)

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `frequence_controle` | `10.0` | Fréquence de la boucle de contrôle (Hz) |
| `mode_par_defaut` | `"MANUEL"` | `"MANUEL"` ou `"AUTONOME"` |

## Licence

Voir [LICENSE](LICENSE).
