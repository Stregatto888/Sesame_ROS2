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
| 4 | Andatura in simulazione (trotto) | ✅ fatta | `v0.5-gait` |
| 5 | ESP32 e ponte ROS con micro-ROS (`sesame_hardware`) | 🔄 in corso (passi 1–4 ✅) | |
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

## Fase 4 — Andatura in simulazione (15/11/2026)

**Scelte di progetto**

- IK nelle coordinate che la gamba controlla davvero: con 2 giunti si fissano solo 2 coordinate del
  piede, x (avanti) e z (altezza) nel frame `base`; la y ne è una conseguenza (l'anca ruota attorno a
  un asse verticale). Newton con jacobiano numerico sulla FK letta dall'URDF: niente servizi ROS nel
  ciclo, stesso calcolo riusabile in seguito.
- Trotto: coppie diagonali `fr`+`rl` (fase 0) e `fl`+`rr` (fase 0.5), duty factor 0.5. Appoggio: il
  piede va da x0 + S/2 a x0 − S/2 a quota costante. Volo: ritorno con arco cicloidale alto h.
- Il ciclo è calcolato una volta (tabella a 50 Hz) e poi riprodotto: è lo stesso formato che userà
  l'ESP32 in fase 5.
- Anca espressa come angolo della gamba dal laterale (90°): 45° = stand, 0° = gamba tutta di lato.
  Ginocchio espresso come piega rispetto alla gamba distesa.

**Parametri scelti** (predefiniti di `gait_planner`)

| Parametro | Valore |
|---|---|
| Anca dal laterale | 15° |
| Piega del ginocchio a riposo | 45° |
| Passo | 30 mm |
| Altezza del volo | 12 mm |
| Ciclo | 1.0 s (50 campioni a 50 Hz) → 60 mm/s |
| Posa di partenza (°) | R1 105, R2 75, L1 75, L2 105, R4 45, R3 135, L3 45, L4 135 |
| Piedi a riposo (mm) | fr (48.8, −80.3, −29.3), rr (−49.0, −80.3, −29.3), fl (48.8, 89.8, −29.3), rl (−49.0, 89.8, −29.3) |

**Scelta dell'anca** (passo 30 mm, ginocchio 45°): è un compromesso tra piedi lontani dal centro in x
(base d'appoggio più lunga) e scorrimento laterale del piede in appoggio, inevitabile con 2 giunti.

| Anca dal laterale | Piedi anteriori a x | Scorrimento laterale in appoggio | Esito |
|---|---|---|---|
| 0° | 33 mm | 4 mm | superato |
| 10° | 44 mm | 9 mm | superato |
| **15° (scelta)** | **49 mm** | **12 mm** | superato |
| 20° | 54 mm | 16 mm | superato |
| 25° | 59 mm | 20 mm | superato |
| 30° | 63 mm | 24 mm | superato |
| 45° (come stand) | — | — | IK fallita: le anche anteriori arrivano al limite (180°) |

**File nuovi**

- `sesame_locomotion/sesame_locomotion/kinematics.py`: FK generica letta da `sesame.urdf` (solo
  numpy) e IK (x, z) → (anca, ginocchio). Costanti condivise: gruppi, ordine firmware, limiti, stand.
- `sesame_locomotion/sesame_locomotion/fk_check.py` (`ros2 run sesame_locomotion fk_check`): confronta
  la FK locale con `/compute_fk` (richiede MoveIt).
- `sesame_locomotion/sesame_locomotion/gait.py` (`ros2 run sesame_locomotion gait_planner`): genera il
  ciclo, lo verifica e scrive la tabella. Opzioni `--hip`, `--knee`, `--stride`, `--lift`, `--period`,
  `--rate`, `--out`. Esce con codice 1 se un controllo non passa.
- `sesame_locomotion/sesame_locomotion/gait_player.py` (`ros2 run sesame_locomotion gait_player`):
  porta il robot alla prima riga in `approach_s` (2 s) e ripete il ciclo `cycles` volte (10) sui 4
  `joint_trajectory_controller`. Parametro `table`. Basta `control.launch.py`.
- `sesame_locomotion/setup.py`, `package.xml`: nuovi comandi; dipendenze `trajectory_msgs`,
  `ament_index_python`, `python3-numpy`.
- `docs/gait/trot_cycle.csv`: tabella di un ciclo, colonne `t_s` + giunti in ordine firmware
  (R1, R2, L1, L2, R4, R3, L3, L4), in gradi (= gradi del servo con direzione +1 e subtrim 0).
- `docs/gait/gait_planner.txt`: output completo del planner con i parametri scelti.

**Verifiche superate**

- `fk_check`: 200 campioni per gamba, errore massimo 0.0000 mm su tutte e 4: la FK locale coincide con
  MoveIt (e quindi con la catena verificata in fase 3). Anche stand e crouch coincidono al decimo di mm.
- `gait_planner` con i parametri scelti: ESITO SUPERATO.

  | Giunto | Min ° | Max ° | Limiti ° | Passo max per tick ° | Velocità max °/s |
  |---|---|---|---|---|---|
  | R1 | 91.1 | 120.5 | 45–180 | 2.03 | 101.3 |
  | R2 | 59.5 | 88.9 | 0–135 | 2.03 | 101.3 |
  | L1 | 59.5 | 88.9 | 0–135 | 2.03 | 101.3 |
  | L2 | 91.1 | 120.5 | 45–180 | 2.03 | 101.3 |
  | R4 | 45.0 | 66.4 | 0–180 | 2.67 | 133.6 |
  | R3 | 113.6 | 135.0 | 0–180 | 2.67 | 133.6 |
  | L3 | 45.0 | 66.4 | 0–180 | 2.67 | 133.6 |
  | L4 | 113.6 | 135.0 | 0–180 | 2.67 | 133.6 |

  Soglie: limiti URDF, velocità ≤ 172 °/s (3.0 rad/s di `joint_limits.yaml`), passo per tick ≤ 12°
  (sicurezza del firmware). Scorrimento laterale in appoggio: 12.7 mm (anteriori), 12.0 mm (posteriori).
- Controllo negativo: con `--knee 60 --stride 40 --lift 15 --period 0.6` (133 mm/s) le velocità
  arrivano a 238 °/s e l'esito è NON SUPERATO, come atteso.
- RViz con `gait_player`: il trotto si vede correttamente sulle 4 gambe (corpo fisso, zampe "in aria":
  la fisica è fuori dal piano).

**Problemi incontrati**

| Problema | Causa | Soluzione |
|---|---|---|
| `IK fallita: fr_leg ... x=90.4` con le anche come in stand | con la gamba a 45° le anche anteriori sono vicine al limite di 180°: il piede non può andare 15 mm più avanti | parametro `--hip`: gamba più verso il laterale |
| Piede che scorre di lato 24 mm in appoggio con anca a 30° | con 2 giunti il piede in appoggio segue un arco attorno all'asse dell'anca | anca a 15°: compromesso tra lunghezza della base e scorrimento (12 mm) |
| Output da riportare in chat | — | `comando \| tee <file>` per salvarlo nella repo, oppure Ctrl+Shift+C nel terminale |

**Come avviare**

```bash
# container, T1: controllori + RViz (MoveIt non serve per riprodurre l'andatura)
cd /root/ros2_ws
colcon build --symlink-install && source install/setup.bash
ros2 launch sesame_bringup control.launch.py

# container, T2
source install/setup.bash
D=/root/ros2_ws/src/sesameMuhackROS/docs/gait
ros2 run sesame_locomotion gait_planner --out $D/trot_cycle.csv | tee $D/gait_planner.txt
ros2 run sesame_locomotion gait_player --ros-args -p table:=$D/trot_cycle.csv
# altre prove: gait_planner --hip 20 --stride 40 --lift 15 --period 0.8
# controllo FK (con moveit.launch.py al posto di control.launch.py): ros2 run sesame_locomotion fk_check
```

---

## Convenzioni (contratto PC / ESP32)

- Ordine dei giunti nel firmware: `R1, R2, L1, L2, R4, R3, L3, L4`.
- Angoli del servo in gradi = angoli dell'URDF. Conversione: `servo = 90 + direzione·(angolo − 90) + subtrim`.
- Posa stand (gradi): R1 135, R2 45, L1 45, L2 135, R4 0, R3 180, L3 0, L4 180.
- Posa crouch (gradi): anche come stand, R3 150, R4 30, L3 30, L4 150.
- Limiti (gradi): R1 45–180, R2 0–135, L1 0–135, L2 45–180, R4/R3/L3/L4 0–180.
- Segnale servo: 50 Hz, impulso 732–2929 µs.
- Interfaccia micro-ROS (fase 5): nodo `sesame_esp32`, `ROS_DOMAIN_ID` 0, QoS best-effort;
  `/sesame/joint_commands` (PC → ESP32) e `/sesame/joint_echo` (ESP32 → PC, 50 Hz), entrambi
  `std_msgs/Float32MultiArray` con 8 valori in ordine firmware, in gradi del servo (decimali).

## Prossimo passo: fase 5 — ESP32 e ponte ROS con micro-ROS

**Decisione (15/11/2026): piano A, micro-ROS sulla Lolin S2 Mini**

- Scheda confermata: Lolin S2 Mini (ESP32-S2, un core, USB nativa).
- Correzione rispetto alla nota precedente ("micro-ROS supporta solo esp32dev"): vale solo per la
  libreria Arduino/PlatformIO `micro_ros_platformio`. Il componente ufficiale `micro_ros_espidf_component`
  è testato su ESP-IDF 5.2–5.5 con ESP32-S2 e ha un esempio di trasporto USB CDC per S2/S3.
- Alternative valutate e scartate: `micro_ros_platformio` adattato alla S2 (non ufficiale), cambio
  scheda verso `esp32dev` (ricablaggio), protocollo seriale nostro (non è micro-ROS).

| Conseguenza | Dove | Cosa fare |
|---|---|---|
| Build da Arduino puro a ESP-IDF + Arduino come componente | firmware | prova minima, poi migrazione del progetto |
| USB nativa riservata a micro-ROS | firmware | log su UART a pin o spenti, CLI seriale off in modalità ROS |
| `setAngle` bloccante (20 ms per servo) e con log su `Serial` | `motors/servo_driver` | driver non bloccante a 50 Hz |
| Subtrim sommato nel firmware e previsto anche nel plugin | firmware + `sesame_hardware` | applicarlo in un solo punto (proposta: plugin) |
| Comandi web e comandi ROS sullo stesso robot | `core/tasks`, `command_queue` | una modalità alla volta |
| RAM limitata (un core, ~320 KB) | firmware | misurare heap/stack con WiFi + web + display |
| Agente micro-ROS sul PC | `docker/Dockerfile`, container | agente Jazzy e accesso a `/dev/ttyACM*` |
| Hardware interface verso i topic | `sesame_hardware` | plugin C++ con `hardware:=real` |

**Passi 1–2 — prova minima micro-ROS sulla S2 (15/11/2026)** ✅

Esempio ufficiale `int32_publisher_custom_transport_usbcdc` (branch `jazzy` del componente) compilato,
flashato sulla Lolin S2 Mini e letto da ROS 2 nel container `sesame_gpu`, fuori dalle due repo.

- Cartella di prova sull'host: `~/workspace/hardware/microros_test` (clone del componente).
- Container di build `sesame_idf` (immagine `espressif/idf:release-v5.5`, `--privileged -v /dev:/dev`,
  cartella montata in `/work`); dipendenze Python `catkin_pkg colcon-common-extensions lark`.
  Rientro: `docker start -ai sesame_idf`.
- Chip letto da esptool: **ESP32-S2FNR2** rev. 1.0, 4 MB di flash e **2 MB di PSRAM** integrati, USB-OTG.
- Agente sull'host: immagine `microros/micro-ros-agent:jazzy`, `--net=host`, `/dev/shm` condiviso.
- L'esempio usa `ROS_DOMAIN_ID=100` (fisso nel codice) e espone due porte USB CDC:
  `ttyACM0` = micro-ROS, `ttyACM1` = log. ID USB dell'applicazione: `303a:4002`.

| Verifica | Esito |
|---|---|
| `idf.py set-target esp32s2 && idf.py build` | `Project build complete` |
| Flash con `idf.py -p /dev/ttyACM0 flash` dal bootloader (0 + RST) | `Hash of data verified` su bootloader, app e partizioni |
| Dopo il reset | `303a:4002` e due porte `ttyACM0`/`ttyACM1` |
| Agente seriale su `ttyACM0` + RST della scheda | sessione stabilita, publisher creato; dopo altri due RST l'agente si riaggancia da solo |
| Log su `ttyACM1` (`idf.py -p /dev/ttyACM1 monitor --no-reset`) | `Message published: N` ogni ~1 s |
| `ros2 topic echo /esp32s2/int32_publisher_usbcdc` in `sesame_gpu` (`ROS_DOMAIN_ID=100`) | messaggi ricevuti, `data` crescente |

**Problemi incontrati**

| Problema | Causa | Soluzione |
|---|---|---|
| `docker: invalid reference format` | comando dell'agente abbreviato con `...` nelle istruzioni | comando completo `docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -v4` |
| Senza agente all'avvio il task micro-ROS si chiude | l'esempio cerca l'agente solo per pochi secondi dopo il reset | avviare prima l'agente, poi RST della scheda |
| Il contatore nell'echo torna a 0 | RST della scheda: nuova sessione | normale; l'agente resta acceso (`Serial port not found... waiting for connection`, poi `session established`) |
| Comandi Git eseguiti nel container `sesame_idf` (`dubious ownership`, `cd` fallito) | terminale sbagliato | nessun effetto sulla repo; terminali con nomi fissi T1–T5 e Git sempre in T5 (host) |
| Firmware Sesame sovrascritto dalla prova | stessa flash | si ricarica da Sesame_MuHack con `pio run -t upload` (0 + RST se serve) |

**Come rifare la prova**

```bash
# T1, host: controllo USB
watch -n 0.5 'lsusb | grep -i 303a; echo; ls -l /dev/ttyACM* 2>/dev/null'
# T4, host → container sesame_idf: build e flash (scheda in download: tieni 0, premi RST, rilascia 0)
docker start -ai sesame_idf
cd /work/micro_ros_espidf_component/examples/int32_publisher_custom_transport_usbcdc
idf.py build && idf.py -p /dev/ttyACM0 flash          # poi RST
# T2, host: agente (resta acceso), poi RST della scheda
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -v4
# T3, container sesame_gpu
export ROS_DOMAIN_ID=100
ros2 topic echo /esp32s2/int32_publisher_usbcdc
# T4, container sesame_idf: log della scheda (uscita con Ctrl+])
idf.py -p /dev/ttyACM1 monitor --no-reset
```

**Passo 3 — build ESP-IDF, micro-ROS nel firmware, decisioni (16/11/2026)** ✅

Firmware `Sesame_MuHack`: prima `develop` riallineato (commit locale 89c3d3f "servo names (R1-L4)"
unito con merge `83bc277`), poi branch `feature/ros-bridge` da `develop`.

- 3a, commit `25fed30`: progetto ESP-IDF 5.5 nella radice del firmware, stessi `src/` e `include/`.
  - `CMakeLists.txt` (`MINIMAL_BUILD`, `PROJECT_VER` fisso: niente Git nel container), `main/`
    (sorgenti da `../src`), `main/idf_component.yml` (`espressif/arduino-esp32 ==3.3.12`),
    `sdkconfig.defaults` (autostart Arduino, tick 1000 Hz, 4 MB, PSRAM, `-Os`), `partitions.csv`
    (= `huge_app.csv`).
  - Librerie come sottomoduli in `components/<nome>/` con un nostro `CMakeLists.txt`: ESP32Servo 3.2.1,
    Adafruit BusIO 1.17.4, GFX 1.13.0, SSD1306 2.5.17.
  - Arduino non usa la USB (`ARDUINO_USB_CDC_ON_BOOT` non definito): `Serial` va su UART0, che sulla
    Lolin S2 Mini non è sui pin. La CLI seriale è quindi già spenta.
- 3b, commit `9d44f6b`: micro-ROS dentro il firmware.
  - Sottomodulo `components/micro_ros_espidf_component` (branch `jazzy`, `5cd71f5`); componenti
    `esp_usbcdc_transport` e `esp_usbcdc_logging` presi dall'esempio nel sottomodulo
    (`EXTRA_COMPONENT_DIRS`); `esp_tinyusb` 1.7.6~2; `app-colcon.meta` con trasporto custom.
  - `src/ros/ros_bridge.cpp`: 2 porte CDC (`ttyACM0` micro-ROS, `ttyACM1` log), task `TaskRos`,
    riconnessione automatica all'agente, log della memoria ogni 5 s.
  - `esp_tinyusb` attiva `CONFIG_TINYUSB_CDC_ENABLED` e Arduino compilerebbe `cores/esp32/USBCDC.cpp`
    (richiede lo stack USB di Arduino, assente): escluso dal `CMakeLists.txt` principale con
    `HEADER_FILE_ONLY`, senza toccare il componente.
  - WiFi e lwIP fuori dalla IRAM (`CONFIG_ESP_WIFI_IRAM_OPT=n`, `..._RX_IRAM_OPT=n`,
    `CONFIG_LWIP_IRAM_OPTIMIZATION=n`): sulla S2 IRAM e DRAM sono la stessa memoria.
- Container `sesame_idf` ricreato da `docker commit` (immagine `sesame_idf:passo2`) con in più
  `~/workspace/hardware/Sesame_MuHack` montata in `/fw`; `/work` invariata.

| Verifica | Esito |
|---|---|
| 3a: `idf.py build` | app 1,06 MB su 3 MB; DIRAM 79,8% |
| 3a: flash e prova (robot sollevato) | OLED, AP `Sesame MuHack`, portale, tutte le pose, `R1 90` dal terminale web; nessun `303a` su USB |
| 3b: `idf.py build` | app 1,13 MB su 3 MB; DIRAM 75,2% |
| 3b: USB dopo il reset | `303a:4002`, `ttyACM0` + `ttyACM1` |
| 3b: agente + `ros2 topic echo /sesame/heartbeat` (`ROS_DOMAIN_ID=0`) | sessione stabilita, `data` +1 al secondo |
| 3b: agente fermato e riavviato; RST della scheda | `agente perso`, poi `agente collegato` senza RST; dopo RST si ricollega da sola |
| 3b: pose dal web con micro-ROS collegato | funzionano, il heartbeat non si interrompe |
| 3b: memoria (collegato) | heap interno libero ~55 KB, minimo 8,3 KB con il web in uso (24,6 KB senza); PSRAM quasi inutilizzata; stack `TaskRos` usato ~3 KB su 16 |

**Decisioni del passo 3**

1. Build: `idf.py` in `sesame_idf`, ESP-IDF 5.5, Arduino 3.3.12 e micro-ROS `jazzy` come componenti a
   versione fissata; su `feature/ros-bridge` PlatformIO non compila più (ripiego: `develop` con
   `pio run -t upload`).
2. Interfaccia: nodo `sesame_esp32`, `ROS_DOMAIN_ID` 0; `/sesame/joint_commands` e `/sesame/joint_echo`,
   `std_msgs/Float32MultiArray`, 8 valori in ordine firmware in gradi del servo, best-effort, 50 Hz.
3. Conversione e sicurezze: conversione e subtrim solo nel plugin (in modalità ROS il firmware ignora
   `Motors::subtrim`); servo scritti in µs con i decimali; nel firmware limiti per giunto, passo massimo
   12° ogni 20 ms, watchdog 250 ms → fermo nella posa corrente con i servo alimentati
   (`disable_timeout_ms` di `default.yaml` non usato); eco = angoli effettivamente scritti. All'avvio
   i servo vanno a 90° come oggi: il plugin parte dalla posa letta dall'eco e il passo massimo evita
   scatti.
4. Modalità e memoria: modalità WEB all'avvio, ROS dal primo comando ROS valido (pose e servo dal web
   rifiutati); ritorno a WEB solo con `mode web` dal terminale web dopo 1 s senza comandi ROS, oppure
   riavvio. Allocazioni da 4 KB in su in PSRAM; stack di `TaskRos` 10 KB; trasporto nostro che
   installa TinyUSB una volta sola.

**Problemi incontrati**

| Problema | Causa | Soluzione |
|---|---|---|
| HEAD staccato su Sesame_MuHack con commit 89c3d3f mai pubblicato e modifiche non salvate | lavoro locale di aprile | branch `salvataggio/servo-names`, stash della sola formattazione, merge in `develop` |
| `tar: Cannot open` | archivio non scaricato dopo aver riaperto la sessione | riscaricarlo in `~/Downloads` |
| `+` in `git submodule status` | commit fissato con `checkout` ma non registrato | `git add` del percorso del sottomodulo |
| `/fw` assente in `sesame_idf` | i montaggi si decidono alla creazione del container | `docker commit` + nuovo `docker run` con `-v ~/workspace/hardware/Sesame_MuHack:/fw` |
| `idf.py: command not found` | `exit` dal container, comando lanciato sull'host | `docker start -ai sesame_idf`, `cd /fw` |
| `usb_phy: selected PHY is in use` in attesa dell'agente | il trasporto dell'esempio reinstalla TinyUSB a ogni tentativo | trasporto nostro nel passo 4 |
| Minimo di heap interno 8,3 KB | allocazioni del web sotto 16 KB tutte in RAM interna | soglia PSRAM a 4 KB (passo 4) |

**Come avviare (firmware)**

```bash
# T4, host → container sesame_idf
docker start -ai sesame_idf
cd /fw
idf.py build
# scheda in download (tieni 0, premi RST, rilascia 0), poi:
idf.py -p /dev/ttyACM0 flash
# log della scheda (uscita con Ctrl+])
idf.py -p /dev/ttyACM1 monitor --no-reset
# T2, host: agente
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -v4
```

**Passo 4 — firmware in modalità ROS (16/11/2026)** ✅

Firmware `Sesame_MuHack`, branch `feature/ros-bridge`, commit `6325c58`.

**File nuovi o modificati**

- `include/core/mode.h`, `src/core/mode.cpp`: modalità WEB (all'avvio) / ROS (dal primo comando ROS
  valido). `Mode::requestWeb()` torna a WEB solo dopo 1 s senza comandi ROS.
- `include/motors/servo_driver.h`, `src/motors/servo_driver.cpp`: `writeDegrees()` (scrittura immediata
  in µs con i decimali, senza subtrim né attese), `lastAngle()` (ultimo angolo scritto, anche dal web);
  in modalità ROS `setAngle()` non muove i servo.
- `src/core/tasks.cpp`: in modalità ROS `TaskMotor` scarta i comandi di posa.
- `src/web/web_server.cpp`: comandi `mode` e `mode web` nel terminale web, modalità in `status`; in
  modalità ROS i comandi di movimento sono rifiutati.
- `include/core/config.h`: limiti dei giunti (gradi del servo, ordine firmware) e parametri
  `ROS_CONTROL_PERIOD_MS` 20, `ROS_MAX_STEP_DEG` 12, `ROS_COMMAND_TIMEOUT_MS` 250, `ROS_IDLE_FOR_WEB_MS` 1000.
- `src/ros/ros_bridge.cpp`, `include/ros/ros_bridge.h`: sottoscrizione `/sesame/joint_commands` e
  pubblicazione `/sesame/joint_echo` (best-effort, 50 Hz); ciclo a 50 Hz in `TaskRos` con limiti, passo
  massimo e watchdog; trasporto USB nostro (TinyUSB installato una sola volta, porta lasciata aperta);
  agente perso dopo 3 ping falliti; stato ogni 5 s su `ttyACM1` (modalità, comandi ricevuti, scartati e
  limitati, memoria). Stack di `TaskRos` 10 KB.
- `sdkconfig.defaults`: `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096`, `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`.

**Verifiche superate** (robot sollevato)

| Verifica | Esito |
|---|---|
| `idf.py build` | immagine 1,17 MB su 3 MB; DIRAM 77,3% |
| Eco `/sesame/joint_echo` | 8 valori in ordine firmware = angoli scritti |
| Un solo comando (`--once`) | ogni servo si sposta al massimo di 12°, poi watchdog |
| Stand, crouch, posa di partenza del trotto a 50 Hz | movimenti fluidi, log `modo ROS` senza `fermo` |
| R1 a 10° | fermo a 45° (limite), log `fuori dai limiti` |
| Comando con 2 valori | scartato, nessun movimento |
| Stop dei comandi (Ctrl+C sul `ros2 topic pub`) | `watchdog` dopo 240 ms, robot fermo nella posa |
| Agente fermato con i comandi in corso | `watchdog` subito, `agente perso` dopo ~2,5 s; agente riavviato → si ricollega e riprende i comandi |
| Terminale web in modalità ROS | pose e `R1 90` rifiutati; `mode web` rifiutato con comandi in arrivo, accettato dopo 1 s |
| Memoria con telefono collegato e pagina web in uso | heap interno minimo 29,4 KB (prima 8,2 KB); PSRAM usata ~16 KB; stack `TaskRos` usato ~2,3 KB su 10 |

**Problemi incontrati**

| Problema | Causa | Soluzione |
|---|---|---|
| `modo ROS fermo` con i comandi in arrivo, servo a scatti | l'ora del ciclo era letta prima dell'executor: il comando risultava 1 ms "nel futuro" e la differenza senza segno diventava enorme, quindi scattava il watchdog | ora riletta in `controlTick()` e differenza con segno |
| Heap interno minimo 8 KB con la pagina web | i buffer di trasmissione di WiFi e lwIP stanno per default solo nella RAM interna | `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` |
| `mode web` accettato e agente fermato senza effetto, in una prova | in quel momento i comandi non arrivavano (contatore `comandi` fermo) | prima di ogni prova controllare che il contatore salga di ~250 ogni 5 s |
| Opzioni nuove di `sdkconfig.defaults` non applicate | `sdkconfig` già esistente | `idf.py set-target esp32s2` (rigenera `sdkconfig`) quando cambia `sdkconfig.defaults` |

**Come provare il firmware in modalità ROS**

```bash
# T2, host: agente
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -v4
# T4, container sesame_idf: log della scheda (uscita con Ctrl+])
cd /fw
idf.py -p /dev/ttyACM1 monitor --no-reset
# T3, container sesame_gpu (ROS_DOMAIN_ID 0)
export ROS_DOMAIN_ID=0
ros2 topic echo --once --qos-reliability best_effort /sesame/joint_echo
ros2 topic pub --rate 50 /sesame/joint_commands std_msgs/msg/Float32MultiArray "{data: [135.0, 45.0, 45.0, 135.0, 0.0, 180.0, 0.0, 180.0]}"
# terminale web: status, mode, mode web
```

**Passi rimanenti della fase**

- Passo 5 — plugin ros2_control `sesame_hardware` (C++, `hardware:=real`): pubblica i comandi, legge
  l'eco, conversione `servo = 90 + direzione·(angolo − 90) + subtrim` da `calibration/default.yaml`.
- Passo 6 — calibrazione dei subtrim in stand; verifica: robot sollevato, stand/crouch da RViz/MoveIt
  uguali alle pose reali, agente fermo → fermo entro 250 ms; poi diario, merge e tag.
