# Detección Colaborativa de Meaconing GNSS mediante Ranging UWB y CUSUM Secuencial

## Documento de Contexto para Desarrollo — TFM Antonio García Alcón

---

## 1. Resumen del proyecto

Este repositorio implementa la simulación software-in-the-loop (SITL) de un **detector colaborativo de ataques de meaconing GNSS entre robots**, propuesto como contribución original del TFM *"Arquitectura de Seguridad para Navegación Autónoma de Robots"* (Universidad Europea de Madrid, 2026).

### 1.1 ¿Qué es el meaconing?

El meaconing es un ataque a sistemas GNSS en el que el atacante **recibe, retarda y retransmite** la señal legítima. A diferencia del spoofing (que genera señales falsas), el meaconing repite la señal real con retardo, lo que desplaza progresivamente la posición calculada por el receptor. **OSNMA (Open Service Navigation Message Authentication) de Galileo no protege contra meaconing**, ya que la señal en sí no está alterada — solo retardada. Esta limitación ha sido confirmada directamente por EUSPA en consultas realizadas para este TFM.

### 1.2 La idea central

Dos (o más) robots que operan cerca comparan dos fuentes de información sobre la distancia que los separa:

- **$D_{GNSS}$**: distancia calculada a partir de las posiciones GNSS reportadas por cada robot
- **$D_{UWB}$**: distancia medida físicamente con sensores Ultra-Wideband (UWB)

En condiciones normales: $D_{GNSS} \approx D_{UWB}$ (dentro del margen de ruido).

Bajo meaconing con una sola antena atacante: ambos robots reciben **la misma señal falsa** → sus posiciones GNSS colapsan a un mismo punto → $D_{GNSS} \approx 0$ mientras $D_{UWB}$ sigue midiendo la distancia real → **la divergencia es masiva e inmediatamente detectable**.

### 1.3 Brecha de investigación

- **Heng, Work & Gao (2015)** validaron experimentalmente la detección cooperativa de spoofing, pero usando correlación del código militar cifrado P(Y) — inaccesible para robots comerciales.
- **Psiaki et al. (2013)** propusieron la correlación cruzada entre receptores duales del código P(Y), validada con datos reales de RF, pero requiere acceso a señales cifradas militares.
- **Siguo Bi et al. (2024)** propusieron (en simulación) usar distancias inter-UAV con optimización SDP, pero sin validación experimental en hardware real.
- **Fishberg et al. (2024, MURP)** demostraron que el UWB comercial mide distancias entre robots con ~0.24 m de error, pero **no abordaron seguridad**.
- **Nadie ha validado experimentalmente la combinación UWB comercial + CUSUM secuencial como detector de meaconing en tiempo real.**

Este proyecto llena ese vacío mediante simulación SITL con ROS 2 + Gazebo.

### 1.4 Referencias clave

| Ref | Paper | Relevancia |
|---|---|---|
| [1] | Bhatti & Humphreys (2017). *Hostile Control of Ships via False GPS Signals: Demonstration and Detection*. NAVIGATION. | Marco de detección secuencial basado en innovaciones; validado en el experimento *White Rose of Drachs* |
| [2] | Psiaki et al. (2013). *GPS Spoofing Detection via Dual-Receiver Correlation of Military Signals*. | Correlación cruzada de código P(Y) entre dos receptores; validado con datos RF reales |
| [3] | Heng, Work & Gao (2015). *GPS Signal Authentication From Cooperative Peers*. IEEE TITS. | Detección cooperativa validada experimentalmente con múltiples receptores y código P(Y) |
| [4] | Siguo Bi et al. (2024). *Detection and Mitigation of Position Spoofing Attacks on Cooperative UAV Swarm Formations*. arXiv. | Propone SDP + distancias inter-UAV para detectar spoofing (simulación) |
| [5] | Fishberg et al. (2024). *MURP: Multi-Agent Ultra-Wideband Relative Pose Estimation*. IEEE RAL. | Validación experimental: UWB comercial logra ~0.24 m de precisión en pose relativa entre robots |
| [6] | EUSPA (2026). *OSNMA definition, status and future developments*. OSNMA Day 2026. | Confirmación oficial de que OSNMA no protege frente a meaconing |
| [7] | EUSPA (2026). *Galileo Second Generation Authentication Capabilities*. OSNMA Day 2026. | OSNMA evolucionará en G2G pero la protección adicional queda en manos de fabricantes |

---

## 2. Objetivos

### 2.1 Objetivo general

Demostrar, mediante simulación SITL (ROS 2 + Gazebo), que la comparación entre la distancia GNSS y la distancia UWB entre dos robots, procesada por un detector secuencial CUSUM, permite detectar ataques de meaconing en tiempo real con una tasa de falsas alarmas controlable.

### 2.2 Objetivos específicos

1. **O1.** Implementar un entorno de simulación con dos robots móviles (TurtleBot3) equipados con GNSS simulado y ranging UWB simulado.
2. **O2.** Implementar un nodo atacante que inyecte meaconing a nivel de datos (arrastre de posiciones GNSS de ambos robots a un mismo punto falso).
3. **O3.** Implementar un detector CUSUM que compare $D_{GNSS}$ y $D_{UWB}$ y publique alertas.
4. **O4.** Evaluar el detector con métricas objetivas: *Time-To-Detect* (TTD) y *False Alarm Rate* (FAR).
5. **O5.** (Opcional) Extender a perfiles multientorno (terrestre, aéreo, marítimo) variando los parámetros de ruido de los sensores.

---

## 3. Stack tecnológico

| Capa | Herramienta | Versión |
|---|---|---|
| Sistema operativo | macOS | — |
| Gestor de entorno | conda + pixi (RoboStack) | — |
| Middleware robótico | ROS 2 | Humble |
| Simulador 3D | Gazebo | Classic 11 (Ignition compatible) |
| Modelo de robot | TurtleBot3 | waffle |
| Lenguaje | Python | 3.10+ |
| Análisis de resultados | Jupyter + matplotlib + numpy | — |
| Grabación de datos | rosbag2 | incluido en ROS 2 Humble |
| Control de versiones | Git + GitHub | — |

> **Nota sobre el entorno:** ROS 2 Humble está instalado vía RoboStack usando `pixi` (no `apt`, ya que el desarrollo se realiza en macOS). Toda la configuración del entorno vive en `/Users/toni/robostack/pixi.toml`. Ver sección "Entorno de trabajo (pixi/RoboStack)" más abajo para el flujo de arranque y cómo añadir dependencias nuevas.

---

## 4. Estructura del proyecto

```
tfm_meaconing_ws/
└── src/
    └── collaborative_detection/
        ├── launch/
        │   ├── two_robots.launch.py      # Gazebo + 2 TurtleBots
        │   └── experiment.launch.py      # Full stack: robots + GNSS + UWB + atacante + detector
        ├── nodes/
        │   ├── gnss_sim_node.py          # GNSS simulado con ruido Gaussiano
        │   ├── uwb_sim_node.py           # Distancia UWB simulada con ruido
        │   ├── meaconing_injector.py     # Nodo atacante
        │   └── cusum_detector_node.py    # Detector CUSUM (corazón del TFM)
        ├── config/
        │   └── params.yaml               # β, τ, σ_UWB, σ_GNSS
        ├── analysis/
        │   └── plot_results.ipynb        # Gráficas para la memoria
        ├── launch/
        └── package.xml
```

---

## 5. Formalización matemática

### 5.1 Definiciones

Dados dos robots A y B con posiciones reales $\mathbf{p}_A, \mathbf{p}_B \in \mathbb{R}^3$:

- **Distancia GNSS:**

$$D_{GNSS}(t) = \lVert \mathbf{p}_A^{GNSS}(t) - \mathbf{p}_B^{GNSS}(t) \rVert$$

donde $\mathbf{p}^{GNSS} = \mathbf{p}^{real} + \boldsymbol{\epsilon}_{GNSS}$, con $\boldsymbol{\epsilon}_{GNSS} \sim \mathcal{N}(0, \sigma_{GNSS}^2)$.

- **Distancia UWB:**

$$D_{UWB}(t) = \lVert \mathbf{p}_A^{real}(t) - \mathbf{p}_B^{real}(t) \rVert + \epsilon_{UWB}$$

con $\epsilon_{UWB} \sim \mathcal{N}(0, \sigma_{UWB}^2)$, $\sigma_{UWB} \approx 0.24$ m (Fishberg et al. 2024).

- **Señal de innovación (residuo):**

$$\Delta(t) = |D_{GNSS}(t) - D_{UWB}(t)|$$

- **Hipótesis:**

$$H_0: \Delta(t) \leq \beta \quad \text{(operación normal)}$$
$$H_1: \Delta(t) > \beta \quad \text{(meaconing activo)}$$

### 5.2 Detector CUSUM

$$
S_k = \max\left(0,\; S_{k-1} + \Delta_k - \beta\right)
$$

$$
\text{Alarma si } S_k > \tau
$$

Donde:
- $\beta$ = sesgo mínimo detectable (*drift* esperado del ataque, ej. 0.5 m)
- $\tau$ = umbral de detección (controla el compromiso TTD vs FAR)
- $S_0 = 0$

### 5.3 Por qué CUSUM distingue ruido de ataque

| Fenómeno | Comportamiento temporal | Efecto en $S_k$ |
|---|---|---|
| **Ruido UWB** | Media cero, Gaussiano, sin estructura | $\max(0, ...)$ lo mantiene cerca de 0 |
| **Meaconing** | Sesgo sistemático, direccional, persistente | $S_k$ crece monótonamente hasta cruzar $\tau$ |

Bhatti & Humphreys [1] validaron experimentalmente que un detector secuencial sobre las innovaciones supera a los detectores de umbral fijo frente a ataques de arrastre lento.

### 5.4 Casos de ataque y detectabilidad

| Caso | Descripción | Recursos del atacante | ¿Detectable? | Motivo |
|---|---|---|---|---|
| **Meaconing 1 antena** | Ambos robots arrastrados al mismo punto falso | Mínimos (SDR básico) | ✅ Sí | $D_{GNSS} \approx 0$, $D_{UWB}$ mide distancia real → $\Delta$ masiva |
| **Spoofing con patrón** | Posiciones falsas distribuidas (crop circles) | Moderados | ✅ Sí | El atacante desconoce la geometría real del enjambre |
| **Phased array individualizado** | $\delta_A = \delta_B$ (traslación sin deformación) | Muy altos (nivel estatal) | ⚠️ Punto ciego | $D_{GNSS} = D_{UWB}$ → $\Delta \approx 0$ |

El punto ciego del Caso 3 se mitiga añadiendo una baliza UWB fija en tierra como referencia absoluta, o combinando con otros sensores (IMU, odometría).

---

## 6. Fases de desarrollo

### Fase 0 — Setup del entorno (1-2 días)

**Objetivo:** ROS 2 + Gazebo funcionales con dos TurtleBots navegando.

> ROS 2 Humble ya está disponible vía RoboStack/pixi — **no requiere instalación con `apt`**. El entorno se activa así:

```bash
# Activar el entorno base de conda (necesario para que funcione ROS)
conda activate base

# Entrar en el directorio del entorno RoboStack
cd robostack  # desde la raíz

# Arrancar el entorno ROS 2 Humble gestionado por pixi
pixi run -e humble
```

> Si es necesario lanzar RViz:

```bash
pixi run -e humble rviz2
```

> Toda la configuración de dependencias (paquetes ROS, TurtleBot3, Gazebo, etc.) vive en `/Users/toni/robostack/pixi.toml`. Si el proyecto necesita algo nuevo (un paquete ROS adicional, una librería, etc.), se añade ahí y luego, estando dentro de `robostack`, se ejecuta:

```bash
pixi install
# o, para actualizar dependencias existentes
pixi update
```

Con el entorno activo, se crea el workspace del proyecto de forma habitual:

```bash
# Crear workspace
mkdir -p ~/tfm_meaconing_ws/src
cd ~/tfm_meaconing_ws
colcon build
source install/setup.bash
```

**Entregable:** Dos TurtleBots visibles en Gazebo, teleoperables o navegando con nav2.

### Fase 1 — GNSS + UWB simulados (3-4 días)

**Objetivo:** Dos nodos Python que publiquen datos "limpios".

**Nodo `gnss_sim_node.py`**

```
INPUT:  /robot1/odom, /robot2/odom (ground truth de Gazebo)
OUTPUT: /robot1/gnss_clean, /robot2/gnss_clean  (PoseStamped o Odometry con ruido)
```

Lógica:

```python
gnss_x = odom_x + np.random.normal(0, sigma_gnss)
gnss_y = odom_y + np.random.normal(0, sigma_gnss)
```

Parámetros: `sigma_gnss = 2.0` (metros, ~ precisión GPS civil sin correcciones).

**Nodo `uwb_sim_node.py`**

```
INPUT:  /robot1/odom, /robot2/odom
OUTPUT: /robots/uwb_distance  (Float64, distancia en metros)
```

Lógica:

```python
dist_real = np.linalg.norm(pos_A - pos_B)
dist_uwb = dist_real + np.random.normal(0, sigma_uwb)
```

Parámetros: `sigma_uwb = 0.24` (metros, basado en Fishberg et al. 2024 MURP).

**Verificación:** Sin ataque, $\Delta = |D_{GNSS} - D_{UWB}|$ debe fluctuar aleatoriamente cerca de 0 con varianza $\sigma_{GNSS}^2 + \sigma_{UWB}^2$.

### Fase 2 — Nodo atacante: `meaconing_injector.py` (3-4 días)

**Objetivo:** Inyectar meaconing a nivel de datos sin modificar la simulación física.

```
INPUT:  /robot1/gnss_clean, /robot2/gnss_clean
OUTPUT: /robot1/gnss_spoofed, /robot2/gnss_spoofed
PARAM:  /meaconing/active (Bool, activable por servicio ROS 2)
```

**Ataque implementado (Meaconing de 1 antena)**

Se define un punto de atracción $\mathbf{p}_{fake}$ que se aleja lentamente de la posición real (ej. 0.2 m/s). Ambos robots reportan la misma posición falseada: $\mathbf{p}_A^{spoofed} = \mathbf{p}_B^{spoofed} = \mathbf{p}_{fake} + \boldsymbol{\epsilon}_{GNSS}$.

Resultado: $D_{GNSS}^{spoofed} \approx 0$ mientras que $D_{UWB}$ sigue midiendo la distancia real entre robots.

**Parámetros configurables:**

- `drift_velocity`: velocidad de alejamiento del punto falso (m/s)
- `activation_delay`: tiempo hasta que el ataque se activa (s)
- `attack_type`: "single_antenna" (implementado), "pattern" (opcional futuro)

### Fase 3 — Detector CUSUM: `cusum_detector_node.py` (3-4 días)

**CORAZÓN DEL TFM.** Este nodo implementa la detección colaborativa.

```
INPUT:  /robot1/gnss_spoofed, /robot2/gnss_spoofed, /robots/uwb_distance
OUTPUT: /system/meaconing_alert  (Bool)
        /system/cusum_value      (Float64, para monitorización)
PARAM:  beta, tau
```

**Pseudocódigo**

```python
class CUSUMDetector(Node):
    def __init__(self):
        self.S = 0.0
        self.beta = self.declare_parameter('beta', 0.5).value   # sesgo mínimo
        self.tau = self.declare_parameter('tau', 2.0).value      # umbral de alarma
        self.alert_pub = self.create_publisher(Bool, '/system/meaconing_alert', 10)
        self.cusum_pub = self.create_publisher(Float64, '/system/cusum_value', 10)

    def update(self, pos_A, pos_B, dist_uwb):
        D_gnss = np.linalg.norm(pos_A - pos_B)
        delta = abs(D_gnss - dist_uwb)

        self.S = max(0.0, self.S + delta - self.beta)

        alert = Bool()
        alert.data = (self.S > self.tau)
        self.alert_pub.publish(alert)

        cusum_msg = Float64()
        cusum_msg.data = self.S
        self.cusum_pub.publish(cusum_msg)

        return alert.data
```

**Notas de implementación**

- Frecuencia de actualización: misma que el topic UWB (~10-40 Hz típicamente)
- Almacenar $S_k$ en rosbag2 para análisis offline
- No resetear $S_k$ tras detección durante experimentos (para medir correctamente FAR y TTD)

### Fase 4 — Experimentos y recolección de métricas (1-2 semanas)

#### 4.1 Escenarios de experimentación

| Escenario | Duración | Ataque | Objetivo |
|---|---|---|---|
| E0 — Baseline | 10 min | Sin ataque | Medir FAR (debe ser ~0) |
| E1 — Meaconing lento | 10 min | drift 0.1 m/s | Medir TTD para ataque sutil |
| E2 — Meaconing rápido | 5 min | drift 0.5 m/s | Medir TTD para ataque obvio |
| E3 — Arranque en caliente | 10 min | Ataque ya activo desde t=0 | Medir TTD desde estado atacado |
| E4 — Variación de distancia entre robots | 10 min | drift 0.2 m/s | ¿Afecta la separación inicial al TTD? |

#### 4.2 Métricas

| Métrica | Definición | Cómo medirla |
|---|---|---|
| TTD (Time-To-Detect) | Tiempo desde activación del ataque hasta $S_k > \tau$ | Extraer del rosbag: timestamp de activación → timestamp de alerta |
| FAR (False Alarm Rate) | Nº de falsas alarmas / tiempo total sin ataque | Contar alertas durante E0 |
| Curva ROC | TTD vs FAR para distintos valores de $\tau$ | Barrer $\tau \in [0.5, 10.0]$ y graficar |

#### 4.3 Gráficas esperadas (para la memoria)

1. Evolución temporal de $S_k$ para E0, E1, E2 (3 subplots superpuestos)
2. TTD vs drift velocity (curva de sensibilidad)
3. Curva ROC (trade-off TTD-FAR)
4. $\Delta(t)$ vs $S(t)$ comparando detector de umbral fijo vs CUSUM (demostrando la ventaja del CUSUM)

### Fase 5 — Extensión multientorno (opcional, 3-5 días)

Generalizar el detector para UAVs y USVs cambiando solo los perfiles de ruido:

| Entorno | $\sigma_{GNSS}$ | $\sigma_{ranging}$ | Sensor de ranging |
|---|---|---|---|
| Terrestre (UGV) | 2.0 m | 0.24 m | UWB |
| Aéreo (UAV) | 2.0 m | 0.15 m | UWB + LiDAR |
| Marítimo (USV) | 2.0 m | 10.0 m | Radar banda X |

Para marítimo, aumentar $\beta$ y $\tau$ proporcionalmente para mantener FAR controlado.

---

## 7. Grabación y análisis de datos

### 7.1 Grabar rosbags

```bash
# Grabar todos los topics relevantes
ros2 bag record -o experimento_E1 \
  /robot1/gnss_spoofed \
  /robot2/gnss_spoofed \
  /robots/uwb_distance \
  /system/cusum_value \
  /system/meaconing_alert \
  /robot1/odom \
  /robot2/odom
```

### 7.2 Análisis offline con Jupyter

El notebook `analysis/plot_results.ipynb` debe:

1. Cargar el rosbag con `rosbag2_py`
2. Extraer series temporales de $S_k$, $\Delta_k$, alertas
3. Calcular TTD: `t_alerta - t_ataque_activado`
4. Calcular FAR: contar alertas durante periodo sin ataque
5. Generar las 4 gráficas de la sección 4.3

---

## 8. Validación científica

### 8.1 Lo que este experimento demuestra

- Que la comparación $D_{GNSS}$ vs $D_{UWB}$ es una señal viable para detectar meaconing
- Que el CUSUM supera al detector de umbral fijo (se demuestra comparando las curvas TTD-FAR)
- Que el método es agnóstico a la plataforma (solo cambian los hiperparámetros)

### 8.2 Lo que este experimento NO demuestra

- No valida funcionamiento con hardware real (UWB físico, GNSS real, RF real) — requiere trabajo futuro
- No cubre el ataque con phased array individualizado (caso 3 de la sección 5.4)
- No prueba el sistema con más de 2 robots

### 8.3 Limitaciones honestas (para la memoria)

- La simulación asume ruido Gaussiano para GNSS y UWB; en la realidad el ruido UWB tiene componentes multi-path no Gaussianos
- No se modela pérdida de línea de visión (NLoS) entre robots
- El meaconing se inyecta a nivel de datos, no de señal RF

---

## 9. Notas para el agente de IA desarrollador

- Empieza por la Fase 0 y Fase 1. Sin ellas, nada funciona. Verifica que los topics se publican con `ros2 topic echo`.
- Parámetros en `params.yaml`, no hardcodeados. Esto permite barrer valores en la Fase 4 sin recompilar.
- Usa `ros2 service` para activar/desactivar el ataque. Facilita la automatización de experimentos.
- Publica $S_k$ en un topic separado. Necesario para el análisis offline.
- Graba rosbags de todos los experimentos. El análisis se hace offline en Jupyter, no en tiempo real.
- Haz que el ataque sea determinista (semilla fija). Reproducibilidad ante todo.

---

## 10. Entorno de trabajo (pixi/RoboStack) y dependencias

Este proyecto se desarrolla en **macOS**, por lo que ROS 2 y sus paquetes **no se instalan con `apt`**. Todo el stack (ROS 2 Humble, Gazebo, TurtleBot3, rosbag2, etc.) se gestiona mediante **RoboStack + pixi** dentro de conda.

### 10.1 Arranque del entorno

```bash
# 1. Activar el entorno base de conda (necesario para que funcione ROS)
conda activate base

# 2. Entrar en el directorio del entorno RoboStack (desde la raíz)
cd robostack

# 3. Levantar el entorno ROS 2 Humble gestionado por pixi
pixi run -e humble
```

Para lanzar RViz dentro de este entorno:

```bash
pixi run -e humble rviz2
```

### 10.2 Dónde viven las dependencias

Todas las dependencias del stack ROS (paquetes ROS 2 Humble, TurtleBot3, Gazebo, rosbag2, librerías Python, etc.) están declaradas en:

```
/Users/toni/robostack/pixi.toml
```

**No se debe usar `sudo apt install` en ningún caso.** Si el proyecto necesita algo que no está ya en el entorno (un paquete ROS adicional, una librería Python, etc.), se añade directamente en `pixi.toml` y después, estando dentro de `robostack`, se ejecuta:

```bash
# Si se ha añadido una dependencia nueva
pixi install

# Si se quiere actualizar dependencias ya existentes
pixi update
```

### 10.3 Dependencias Python específicas del proyecto

Las librerías de análisis (numpy, matplotlib, jupyter, scipy) que no formen ya parte del entorno pixi pueden instalarse dentro del entorno activo con `pip`, o añadirse igualmente a `pixi.toml` para mantener todo reproducible desde un único fichero:

```bash
pip install numpy matplotlib jupyter scipy
```

### 10.4 Configuración del modelo TurtleBot3

Dentro del entorno pixi activo, exportar el modelo del robot:

```bash
export TURTLEBOT3_MODEL=waffle
```

(Si se desea persistente en cada arranque del entorno, puede añadirse como variable de entorno en `pixi.toml` en lugar de en `~/.bashrc`, ya que el flujo de activación pasa por `pixi run`.)

---

*Documento generado para el TFM de Antonio García Alcón — Universidad Europea de Madrid, 2026. Supervisado por Alberto Partida Rodríguez.*