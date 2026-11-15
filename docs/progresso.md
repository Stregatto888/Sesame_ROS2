# Sesame ROS 2 — diario di avanzamento

Obiettivo: far muovere il robot fisico Sesame con la cinematica inversa calcolata in ROS 2
(MoveIt 2), eseguita dall'ESP32 sui servo. Fisica e machine learning sono fuori da questo
piano per ora.

| Fase | Contenuto | Stato | Tag |
|---|---|---|---|
| 0 | Git, licenza, crediti | ✅ fatta | `v0.1-rviz` |
| 1 | xacro + ros2_control con hardware finto | ✅ fatta | `v0.2-control` |
| 2 | MoveIt 2 (4 gruppi gamba, pick_ik) | ✅ fatta | `v0.3-moveit` |
| 3 | IK verificata (andata e ritorno, marker) | ✅ fatta | `v0.4-ik` |
| 4 | Andatura in simulazione | ⏭️ prossima | |
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
  controllori, RViz con `sesame_description/rviz/sesame.rviz`. Argomenti `hardware:=mock|real` e
  `rviz:=true|false`.
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

## Fase 2 — MoveIt 2 con pick_ik (15/11/2026)

**Installazione**

- Nel container: `ros-jazzy-moveit` e `ros-jazzy-pick-ik` (aggiunti al `docker/Dockerfile` in fase 3).

**Setup Assistant** (`ros2 launch moveit_setup_assistant setup_assistant.launch.py`)

- Modello: `sesame_description/urdf/sesame.urdf.xacro` con `hardware:=mock`. Nome del robot
  nell'URDF: **`sesame_quadruped`**.
- Self-Collisions: matrice generata. Virtual Joints: nessuno (radice `base_footprint`, IK nel sistema
  del corpo). End Effectors e Passive Joints: nessuno.
- Planning Groups (catene, solver `pick_ik/PickIkPlugin`, planner RRTConnect):

  | Gruppo | Catena | Giunti | Controllore |
  |---|---|---|---|
  | `fr_leg` | `base` → `r3_foot` | R1, R3 | `fr_leg_controller` |
  | `rr_leg` | `base` → `r4_foot` | R2, R4 | `rr_leg_controller` |
  | `fl_leg` | `base` → `l3_foot` | L1, L3 | `fl_leg_controller` |
  | `rl_leg` | `base` → `l4_foot` | L2, L4 | `rl_leg_controller` |

- Pose (rad):
  - `stand`: R1 2.356, R3 3.1415, R2 0.785, R4 0, L1 0.785, L3 0, L2 2.356, L4 3.1415
  - `crouch` (aggiunta a mano nell'SRDF): ginocchia a 150°/30°, cioè R3 2.618, R4 0.524,
    L3 0.524, L4 2.618; anche come in stand.
- ros2_control URDF Modification: saltata (il blocco è già nel nostro xacro).

**File nuovi o sostituiti**

- `sesame_moveit_config/` generato dal Setup Assistant (SRDF `config/sesame_quadruped.srdf`).
- `sesame_moveit_config/config/kinematics.yaml`: pick_ik `mode: global`, `rotation_scale: 0.0`
  (solo posizione: la gamba ha 2 giunti), `position_threshold: 0.001`, timeout 0.05 s.
- `sesame_moveit_config/config/joint_limits.yaml`: per ogni giunto `max_velocity 3.0` rad/s,
  `max_acceleration 5.0` rad/s² (obbligatori su Jazzy), fattori di scala 0.5.
- `sesame_moveit_config/config/moveit_controllers.yaml`: 4 controllori `FollowJointTrajectory`
  (`action_ns: follow_joint_trajectory`) con gli stessi nomi di `controllers.yaml`.
- `sesame_bringup/launch/moveit.launch.py`: include `control.launch.py` con `rviz:=false`, avvia
  `move_group` e RViz con `sesame_moveit_config/config/moveit.rviz`. Il `robot_description` viene
  dal nostro xacro (unica fonte di verità; in fase 5 basta `hardware:=real`). Il `ros2_controllers.yaml`
  e il `demo.launch.py` generati non vengono usati.
- `sesame_bringup/package.xml`: dipendenze `sesame_moveit_config` e `moveit_configs_utils`.
- `tools/joint_degrees.py`: legge `/joint_states` e mostra gradi, ordine del firmware, valore servo
  (`90 + direzione·(angolo − 90) + subtrim`) e avviso fuori limite.

**Verifiche superate**

- `colcon build`: 3 pacchetti senza errori.
- `ros2 launch sesame_bringup moveit.launch.py`: `move_group` parte, RViz mostra il pannello MotionPlanning.
- Plan & Execute sul gruppo `fr_leg` (stand ↔ crouch e trascinando il marker): la gamba si muove
  attraverso `fr_leg_controller`.

**Problemi incontrati**

| Problema | Causa | Soluzione |
|---|---|---|
| Pose `stand` con R1 1.9634 e R2 1.0153 | slider lasciati sul valore iniziale | valori scritti a mano nella casella |
| `Incorrect Folder/Package` alla generazione | percorso = radice del repo | salvare in `.../sesameMuhackROS/sesame_moveit_config` (non aggiungere `.setup_assistant` alla radice) |
| SRDF `sesame_quadruped.srdf`, non `sesame.srdf` | il robot nell'URDF si chiama `sesame_quadruped` | `MoveItConfigsBuilder("sesame_quadruped", ...)` nel launch |
| `Permission denied` eseguendo un `.launch.py` | i launch non si eseguono direttamente | `ros2 launch <pacchetto> <file>` |
| Avvisi `Overrun detected` del controller_manager | macchina carica durante il Setup Assistant | innocui con l'hardware finto; un solo launch alla volta |
| File di root nel repo | generati dal container | `sudo chown -R $USER:$USER .` dall'host |

**Chiusura**

- Merge di `feature/moveit-config` in `main`, tag `v0.3-moveit`.
- `ros-jazzy-moveit` e `ros-jazzy-pick-ik` nel `docker/Dockerfile`: fatto nel branch della fase 3.

**Come avviare**

```bash
# host
xhost +local:docker
docker start sesame_gpu
docker exec -it sesame_gpu bash

# container
cd /root/ros2_ws
source install/setup.bash
ros2 launch sesame_bringup moveit.launch.py      # MoveIt + controllori + RViz
# solo controllori senza MoveIt: ros2 launch sesame_bringup control.launch.py
```

---

## Fase 3 — IK verificata (15/11/2026)

**File nuovi**

- `sesame_locomotion/` (pacchetto Python, `ament_python`), riusato in fase 4:
  - `sesame_locomotion/ik_roundtrip.py` (`ros2 run sesame_locomotion ik_roundtrip`): per ogni gamba
    angoli casuali nei limiti (margine 5°) → `/compute_fk` → `/compute_ik` con seme crouch → FK della
    soluzione → errore di posizione del piede. Parametri `samples`, `margin_deg`, `tolerance_mm`,
    `avoid_collisions`, `seed`, `csv`.
  - `sesame_locomotion/reach_surface.py` (`ros2 run sesame_locomotion reach_surface`): griglia 61×61 di
    (anca, ginocchio) nei limiti → `/compute_fk` dei 4 piedi in una chiamata → un CSV per gamba e
    riepilogo. Parametri `step_deg`, `csv_prefix`.
- `docs/ik/`: `ik_roundtrip.csv` e `reach_<gruppo>.csv` (dati delle verifiche).
- `docker/Dockerfile`: aggiunti `ros-jazzy-moveit` e `ros-jazzy-pick-ik`.

**Verifiche superate**

- Andata e ritorno, 200 campioni per gamba, tolleranza 1 mm: 0 IK fallite, 800/800 sotto tolleranza.

  | Gruppo | Campioni | IK fallite | < 1 mm | Errore max (mm) | Errore medio (mm) |
  |---|---|---|---|---|---|
  | `fr_leg` | 200 | 0 | 200 | 0.0005 | 0.0003 |
  | `rr_leg` | 200 | 0 | 200 | 0.0005 | 0.0003 |
  | `fl_leg` | 200 | 0 | 200 | 0.0006 | 0.0003 |
  | `rl_leg` | 200 | 0 | 200 | 0.0005 | 0.0003 |

- Marker in RViz su tutte e 4 le gambe (partenza da crouch, Approx IK Solutions attivo): il piede segue
  il marker e Plan & Execute porta la gamba nella posizione mostrata.

**Superficie raggiungibile** (frame `base`, mm; griglia 61×61 nei limiti)

| Gruppo | Piede | x | y | z | Stand | Crouch |
|---|---|---|---|---|---|---|
| `fr_leg` | `r3_foot` | −24.3 … 101.5 | −98.0 … −12.2 | −40.0 … 40.2 | (55.0, −39.6, −39.9) | (69.3, −53.9, −35.4) |
| `rr_leg` | `r4_foot` | −101.7 … 24.1 | −98.0 … −12.2 | −40.0 … 40.2 | (−55.2, −39.6, −39.9) | (−69.5, −53.9, −35.4) |
| `fl_leg` | `l3_foot` | −24.3 … 101.5 | 21.7 … 107.6 | −40.0 … 40.2 | (55.0, 49.1, −39.9) | (69.3, 63.4, −35.4) |
| `rl_leg` | `l4_foot` | −101.7 … 24.1 | 21.7 … 107.6 | −40.0 … 40.2 | (−55.2, 49.1, −39.9) | (−69.5, 63.4, −35.4) |

- Gambe speculari: anteriori/posteriori in x (scarto 0.2 mm); destra/sinistra in y a meno di ~9.5 mm,
  perché l'origine di `base` non è al centro (anche destre a y −20.6 mm, sinistre a +30.2 mm).
- Forma: l'anca ha asse verticale, il ginocchio asse orizzontale. Il piede è a ~40 mm dall'asse del
  ginocchio, che sta a z ≈ 0 e a ~36 mm in orizzontale dall'asse dell'anca. Con l'anca ferma il piede
  percorre una semicirconferenza verticale di raggio 40 mm (z −40 con ginocchio a 180°/0°, cioè stand;
  z +40 all'altro estremo); l'anca la fa ruotare attorno all'asse verticale.
- Conseguenze per l'andatura: altezza e distanza dal corpo sono legate (non si alza il piede senza
  spostarlo in fuori); la lunghezza del passo viene dall'anca (arco orizzontale), il sollevamento dal
  ginocchio. In stand il piede è nel punto più basso e il ginocchio quasi non lo solleva (singolarità);
  il crouch abbassa il corpo di soli 4.5 mm: per la fase 4 valutare una posa di partenza con ginocchio
  a 45–60°.

**Problemi incontrati**

| Problema | Causa | Soluzione |
|---|---|---|
| `install/setup.bash: No such file` e `ros2: command not found` | comandi del container lanciati sull'host | `docker exec -it sesame_gpu bash`, poi `source install/setup.bash` |
| Nuovo comando `reach_surface` non trovato | build fatta prima di copiare i file | copiare i file, poi `colcon build --packages-select sesame_locomotion` |
| Il marker in RViz non trascina il piede | la rotazione del marker è un vincolo che la gamba a 2 giunti non rispetta | scheda Context → Approx IK Solutions |
| CSV persi al riavvio | scritti in `/tmp` del container | copiati in `docs/ik/` |

**Come avviare**

```bash
# container, T1
cd /root/ros2_ws
colcon build --symlink-install && source install/setup.bash
ros2 launch sesame_bringup moveit.launch.py

# container, T2
source install/setup.bash
ros2 run sesame_locomotion ik_roundtrip       # andata e ritorno, esito SUPERATO/NON SUPERATO
ros2 run sesame_locomotion reach_surface      # superficie raggiungibile, CSV in /tmp/reach_<gruppo>.csv
```

---

## Convenzioni (contratto PC / ESP32)

- Ordine dei giunti nel firmware: `R1, R2, L1, L2, R4, R3, L3, L4`.
- Angoli del servo in gradi = angoli dell'URDF. Conversione: `servo = 90 + direzione·(angolo − 90) + subtrim`.
- Posa stand (gradi): R1 135, R2 45, L1 45, L2 135, R4 0, R3 180, L3 0, L4 180.
- Posa crouch (gradi): anche come stand, R3 150, R4 30, L3 30, L4 150.
- Limiti (gradi): R1 45–180, R2 0–135, L1 0–135, L2 45–180, R4/R3/L3/L4 0–180.
- Segnale servo: 50 Hz, impulso 732–2929 µs.

## Prossimo passo: fase 4 — andatura in simulazione

- Nodo in `sesame_locomotion` che genera le traiettorie dei piedi: arco in volo + spinta, trotto
  (coppie diagonali `fr`+`rl` e `fl`+`rr`).
- Piedi → IK → angoli; controllo che nessun limite venga superato e che le velocità restino entro
  quelle dei servo (`joint_limits.yaml`: 3.0 rad/s).
- Scegliere la posa di partenza (ginocchio 45–60°, vedi superficie raggiungibile della fase 3).
- Salvare la tabella di un ciclo (angoli in ordine firmware) come riferimento per l'ESP32.
