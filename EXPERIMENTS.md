# Guía de Experimentos — Detección Colaborativa de Meaconing GNSS

**TFM: Arquitectura de Seguridad para Navegación Autónoma de Robots**  
Antonio García Alcón — Universidad Europea de Madrid, 2026

---

## 1. Setup del entorno

```bash
# 1. Activar RoboStack (Jazzy)
conda activate base
cd ~/robostack
pixi run -e jazzy

# 2. Build del workspace
cd ~/tfm_meaconing_ws
colcon build --packages-select collaborative_detection
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
```

---

## 2. Smoke test rápido (verificar que todo funciona)

### 2.1 Lanzar el experimento

```bash
ros2 launch collaborative_detection experiment.launch.py
```

Se abre Gazebo con 2 TurtleBots. A los 7s empiezan a moverse en círculos.

### 2.2 Verificar topics (en otra terminal)

```bash
# Terminal 2 — sourcear el entorno primero
source ~/tfm_meaconing_ws/install/setup.bash

# Listar todos los topics
ros2 topic list
# Deberías ver: /robot1/gnss_clean, /robot2/gnss_spoofed,
#               /robots/uwb_distance, /system/cusum_value,
#               /system/meaconing_alert, /meaconing/active, etc.

# Monitorizar CUSUM (debe fluctuar cerca de 0)
ros2 topic echo /system/cusum_value

# Alerta (debe ser False sin ataque)
ros2 topic echo /system/meaconing_alert
```

### 2.3 Activar el ataque manualmente (sin esperar 30s)

```bash
ros2 service call /meaconing/set_active std_srvs/srv/SetBool "{data: true}"
```

**Resultado esperado:**
- `S_k` empieza a subir monótonamente
- En ~2-10s (según `drift_velocity`), `S_k` cruza `tau` → `🚨 MEACONING DETECTED!`

### 2.4 Resetear entre pruebas

```bash
ros2 service call /system/reset_cusum std_srvs/srv/Trigger
ros2 service call /meaconing/set_active std_srvs/srv/SetBool "{data: false}"
```

---

## 3. Escenarios de experimento (E0-E4)

Cada escenario requiere cambiar parámetros en `src/collaborative_detection/config/params.yaml` y reconstruir.

### E0 — Baseline (sin ataque, medir FAR)

**Objetivo:** Verificar que sin ataque no hay falsas alarmas.

```yaml
# params.yaml
activation_delay: 9999    # Nunca se activa solo
tau: 2.0
beta: 0.5
drift_velocity: 0.2
```

```bash
colcon build --packages-select collaborative_detection && source install/setup.bash
ros2 bag record -o bags/E0_baseline \
  /robot1/gnss_spoofed /robot2/gnss_spoofed \
  /robots/uwb_distance /system/cusum_value \
  /system/delta_value /system/meaconing_alert \
  /meaconing/active
# Dejar correr 10 min → Ctrl+C
```

### E1 — Meaconing lento (medir TTD con ataque sutil)

**Objetivo:** Medir tiempo de detección con drift lento.

```yaml
activation_delay: 30.0    # Ataque a los 30s
drift_velocity: 0.1       # 0.1 m/s
tau: 2.0
beta: 0.5
```

```bash
colcon build --packages-select collaborative_detection && source install/setup.bash
ros2 bag record -o bags/E1_slow_attack \
  /robot1/gnss_spoofed /robot2/gnss_spoofed \
  /robots/uwb_distance /system/cusum_value \
  /system/delta_value /system/meaconing_alert \
  /meaconing/active
# Dejar correr 10 min → Ctrl+C
```

### E2 — Meaconing rápido (medir TTD con ataque obvio)

**Objetivo:** Medir TTD con drift rápido.

```yaml
activation_delay: 30.0
drift_velocity: 0.5       # 0.5 m/s
tau: 2.0
beta: 0.5
```

```bash
colcon build --packages-select collaborative_detection && source install/setup.bash
ros2 bag record -o bags/E2_fast_attack \
  /robot1/gnss_spoofed /robot2/gnss_spoofed \
  /robots/uwb_distance /system/cusum_value \
  /system/delta_value /system/meaconing_alert \
  /meaconing/active
# Dejar correr ~5 min → Ctrl+C
```

### E3 — Arranque en caliente (ataque desde t=0)

**Objetivo:** Medir TTD cuando el sistema arranca ya bajo ataque.

```yaml
activation_delay: 2.0      # Ataque al inicio
drift_velocity: 0.2
tau: 2.0
beta: 0.5
```

```bash
colcon build --packages-select collaborative_detection && source install/setup.bash
ros2 bag record -o bags/E3_hot_start \
  /robot1/gnss_spoofed /robot2/gnss_spoofed \
  /robots/uwb_distance /system/cusum_value \
  /system/delta_value /system/meaconing_alert \
  /meaconing/active
# Dejar correr 10 min → Ctrl+C
```

### E4 — Variación de distancia entre robots

**Objetivo:** Ver si la separación inicial afecta al TTD.

```yaml
activation_delay: 30.0
drift_velocity: 0.2
tau: 2.0
beta: 0.5
```

```bash
colcon build --packages-select collaborative_detection && source install/setup.bash

# Lanzar con robots más separados (x2=5.0)
ros2 launch collaborative_detection experiment.launch.py x2:=5.0

ros2 bag record -o bags/E4_varied_distance \
  /robot1/gnss_spoofed /robot2/gnss_spoofed \
  /robots/uwb_distance /system/cusum_value \
  /system/delta_value /system/meaconing_alert \
  /meaconing/active
# Dejar correr 10 min → Ctrl+C
```

---

## 4. Barrido de τ (curva ROC)

Para generar la curva ROC (TTD vs FAR), repetir E1 variando `tau`:

| τ | Esperado |
|---|---|
| 0.5 | TTD muy rápido, FAR alto |
| 1.0 | TTD rápido, FAR moderado |
| 2.0 | Equilibrio |
| 5.0 | TTD lento, FAR bajo |
| 10.0 | TTD muy lento, FAR ~0 |

```bash
# Para cada valor de tau, editar params.yaml y:
colcon build --packages-select collaborative_detection && source install/setup.bash
ros2 bag record -o bags/E1_tau_5.0 \
  /robot1/gnss_spoofed /robot2/gnss_spoofed \
  /robots/uwb_distance /system/cusum_value \
  /system/delta_value /system/meaconing_alert \
  /meaconing/active
```

---

## 5. Análisis offline con Jupyter

Una vez grabados todos los rosbags:

```bash
cd ~/tfm_meaconing_ws/src/collaborative_detection/analysis

# Instalar dependencias si no están
pip install jupyter matplotlib numpy

# Lanzar notebook
jupyter notebook plot_results.ipynb
```

El notebook contiene funciones para:
1. Cargar rosbags con `rosbag2_py`
2. Extraer series temporales de `S_k`, `Δ_k`, alertas
3. Calcular TTD (tiempo hasta primera alerta tras activación)
4. Calcular FAR (falsas alarmas/minuto durante periodo sin ataque)
5. Generar las 4 gráficas de la memoria (§4.3)

---

## 6. Resumen de parámetros clave

| Parámetro | Default | Significado |
|---|---|---|
| `sigma_gnss` | 2.0 | Ruido GNSS (m) — GPS civil |
| `sigma_uwb` | 0.24 | Ruido UWB (m) — Fishberg 2024 |
| `beta` | 0.5 | Sesgo mínimo detectable CUSUM (m) |
| `tau` | 2.0 | Umbral de alarma CUSUM |
| `drift_velocity` | 0.2 | Velocidad de arrastre del ataque (m/s) |
| `activation_delay` | 30.0 | Tiempo hasta auto-activación (s) |
| `attack_type` | single_antenna | Tipo de ataque |
| `random_seed` | 42 | Semilla numpy (reproducibilidad) |
| `robot1_linear_vel` | 0.15 | Velocidad lineal robot 1 (m/s) |
| `robot1_angular_vel` | 0.30 | Velocidad angular robot 1 (rad/s) |
| `robot2_linear_vel` | 0.12 | Velocidad lineal robot 2 (m/s) |
| `robot2_angular_vel` | 0.25 | Velocidad angular robot 2 (rad/s) |

---

## 7. Troubleshooting

| Problema | Solución |
|---|---|
| `ros_gz_sim` no encontrado | Usar `pixi run -e jazzy` (no humble) |
| `turtlebot3_gazebo` no encontrado | Añadir `ros-jazzy-turtlebot3-simulations` a `pixi.toml` |
| Error `oneshot` en `create_timer` | Ya corregido — usa `timer.cancel()` en su lugar |
| OGRE rendering errors | Cosmético en macOS — no afecta al experimento |
| Thread affinity warnings | Normal en macOS con DDS — ignorar |
| Gazebo no muestra GUI | Verificar que `gz sim` está instalado (`brew install gz-harmonic`) |

---
