# Sesame ROS 2 — diario di avanzamento

Obiettivo: far muovere il robot fisico Sesame con la cinematica inversa calcolata in ROS 2
(MoveIt 2), eseguita dall'ESP32 sui servo. Fisica e machine learning sono fuori da questo
piano per ora.

| Fase | Contenuto | Stato | Tag |
|---|---|---|---|
| 0 | Git, licenza, crediti | ✅ fatta | `v0.1-rviz` |
| 1 | xacro + ros2_control con hardware finto | ✅ fatta | `v0.2-control` |
| 2 | MoveIt 2 (4 gruppi gamba, pick_ik) | ⏭️ prossima | `v0.3-moveit` |
| 3 | IK verificata (andata e ritorno, marker) | da fare | |
| 4 | Andatura in simulazione | da fare | |
| 5 | ESP32 e ponte ROS (`sesame_hardware`) | da fare | |
| 6 | IK sul robot fisico | da fare | |

---

## Fase 0 — Git e repository (15/11/2026)

- Il workspace `~/workspace/hardware/sesameMuhackROS` è il repository `Sesame_ROS2` (privato su GitHub).
- `LICENSE` Apache-2.0 e `NOTICE` con i crediti: Sesame Robot di Dorian Todd e sesame-ml di Luke Hollis
  (entrambi Apache-2.0).
- `.gitignore` esclude `build/`, `install/`, `log/` e i file temporanei.
- Tag `v0.1-rviz`: stato della relazione v4 (Sesame visibile in RViz2 con GPU).

## Fase 1 — xacro e ros2_control con hardware finto (15/11/2026)

**File nuovi**

- `sesame_description/urdf/sesame.urdf.xacro`: include `sesame.urdf` senza modificarlo, aggiunge il
  link radice `base_footprint` (toglie l'avviso di kdl_parser sull'inerzia di `base`) e il blocco
  `ros2_control`. Argomento `hardware:=mock|real` (`real` riservato alla fase 5).
- `sesame_description/urdf/sesame.ros2_control.xacro`: 8 giunti con comando di posizione e stati di
  posizione e velocità; valori iniziali = posa stand. Hardware finto: `mock_components/GenericSystem`.
- `sesame_bringup/config/controllers.yaml`: `joint_state_broadcaster` + 4
  `joint_trajectory_controller` a 50 Hz:
  - `fr_leg_controller`: R1, R3 (anteriore destra)
  - `rr_leg_controller`: R2, R4 (posteriore destra)
  - `fl_leg_controller`: L1, L3 (anteriore sinistra)
  - `rl_leg_controller`: L2, L4 (posteriore sinistra)
- `sesame_bringup/launch/control.launch.py`: robot_state_publisher, controller_manager, spawner dei
  controllori, RViz con `sesame_description/rviz/sesame.rviz`.
- `docker/Dockerfile`: aggiunti `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`, `ros-jazzy-xacro`.

**Verifiche superate**

- `check_urdf` sul modello generato da xacro: radice `base_footprint`.
- `ros2 control list_controllers`: 5 controllori `active`.
- Una traiettoria pubblicata su `/fr_leg_controller/joint_trajectory` muove la gamba anteriore destra
  in RViz e la riporta in stand.

**Problemi incontrati**

| Problema | Causa | Soluzione |
|---|---|---|
| colcon: `Invalid email` | maintainer `stregatto@archlinux` senza dominio | email valida nei due `package.xml` |
| `Package 'sesame_bringup' not found` | build fallita / shell senza `source` | `colcon build`, poi `source install/setup.bash` in ogni terminale |
| `list_controllers` non risponde | launch non attivo o terminale senza `source` | launch acceso + `source`; controllo con `ros2 node list` |
| `git: not a git repository` nel container | `.git` è sull'host | Git sempre dall'host |
| Avviso `FIFO RT scheduling` | container senza permessi real-time | innocuo con l'hardware finto |

**Come avviare**

```bash
# host
xhost +local:docker
docker start sesame_gpu
docker exec -it sesame_gpu bash

# container
cd /root/ros2_ws
source install/setup.bash
ros2 launch sesame_bringup control.launch.py
```

---

## Convenzioni (contratto PC / ESP32)

- Ordine dei giunti nel firmware: `R1, R2, L1, L2, R4, R3, L3, L4`.
- Angoli del servo in gradi = angoli dell'URDF. Conversione: `servo = 90 + direzione·(angolo − 90) + subtrim`.
- Posa stand (gradi): R1 135, R2 45, L1 45, L2 135, R4 0, R3 180, L3 0, L4 180.
- Limiti (gradi): R1 45–180, R2 0–135, L1 0–135, L2 45–180, R4/R3/L3/L4 0–180.
- Segnale servo: 50 Hz, impulso 732–2929 µs.

## Prossimo passo: fase 2 — MoveIt 2

Installare `ros-jazzy-moveit` e `ros-jazzy-pick-ik`; Setup Assistant con 4 gruppi gamba
(R1–R3 → `r3_foot`, R2–R4 → `r4_foot`, L1–L3 → `l3_foot`, L2–L4 → `l4_foot`), matrice delle
collisioni, pick_ik in solo posizione (`rotation_scale: 0.0`), collegamento ai 4 controllori della fase 1.
Controllo di fine fase: un Plan & Execute dal pannello MotionPlanning muove la gamba.
