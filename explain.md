# Memoria técnica del proyecto
## Detección colaborativa de meaconing GNSS mediante ranging UWB y CUSUM secuencial

**Proyecto:** Arquitectura de seguridad para navegación autónoma de robots  
**Autor:** Antonio García Alcón  
**Institución:** Universidad Europea de Madrid  
**Año:** 2026  
**Tipo de validación:** simulación software-in-the-loop (SITL) con ROS 2, Gazebo Sim y dos TurtleBot3 Waffle

> Este documento reúne la información necesaria para redactar una memoria de varias páginas. Distingue entre el diseño implementado, los resultados medidos directamente en los rosbags disponibles y las limitaciones que deben explicarse en el informe final.

---

## 1. Resumen ejecutivo

El proyecto implementa y evalúa un sistema colaborativo para detectar ataques de **meaconing GNSS** contra una pareja de robots móviles. El meaconing consiste en recibir señales GNSS legítimas, introducir un retardo y retransmitirlas. El receptor sigue procesando una señal aparentemente auténtica, pero su posición se desplaza progresivamente. Esto hace que el ataque sea especialmente difícil de identificar con un único receptor y puede inducir a error a un sistema autónomo de navegación.

La propuesta usa una propiedad física que el atacante no puede modificar fácilmente: la distancia real entre los robots. Cada robot produce una posición GNSS simulada y la pareja dispone de una distancia inter-robot medida mediante UWB. El detector calcula:

- `D_GNSS`: distancia entre las posiciones GNSS reportadas por los dos robots.
- `D_UWB`: distancia física medida con UWB a partir de la geometría real de Gazebo.
- `delta_raw = D_UWB - D_GNSS`: innovación firmada entre ambas fuentes.

En operación normal, ambas distancias deberían ser compatibles, aunque el ruido GNSS introduce un sesgo debido a la norma Euclídea. Durante un ataque de una antena, las posiciones de los dos receptores son arrastradas hacia un punto falso común. Por tanto, `D_GNSS` tiende a colapsar mientras `D_UWB` continúa midiendo la separación física. La discrepancia se convierte en un sesgo persistente y positivo que el CUSUM acumula.

El detector implementado es un **CUSUM secuencial de dos colas**, con calibración inicial de la línea base, media móvil de la innovación y confirmación temporal de la alarma. Los parámetros principales son:

- `beta = 0.5`: deriva mínima que se resta en cada actualización.
- `tau = 3.0`: umbral del estadístico CUSUM.
- `filter_window = 30`: media móvil de aproximadamente un segundo a 30 Hz.
- `alert_confirm_time = 2.0 s`: tiempo que el estadístico debe permanecer sobre el umbral antes de confirmar la alarma.
- `startup_delay = 10.0 s`: periodo normal utilizado para estimar la línea base.

Los nuevos rosbags contienen ocho escenarios: E0, E1, E2, E3, E4, una referencia sin ataque para E5, E5 y E6. En esta campaña todos los escenarios atacados produjeron una alarma confirmada y los dos controles sin ataque no produjeron falsas alarmas:

- **E0, sin ataque:** `TTD = N/A`, falsas alarmas `= 0`.
- **E1, deriva lenta de 0.1 m/s:** `TTD = 2.71 s`, falsas alarmas `= 0`.
- **E2, deriva rápida de 0.5 m/s:** `TTD = 3.93 s`, falsas alarmas `= 0`.
- **E3, hot start:** `TTD = 4.34 s`, falsas alarmas `= 0`; el pre-roll permite medir el arranque atacado.
- **E4, separación de 5 m:** `TTD = 2.73 s`, falsas alarmas `= 0`.
- **E5 reference, navegación por waypoints sin ataque:** `TTD = N/A`, falsas alarmas `= 0`.
- **E5, sólo robot 1 meaconado:** `TTD = 5.70 s`, falsas alarmas `= 0`; la deriva de R1 en el instante de detección es `0.091 m`.
- **E6, ambos robots meaconados:** `TTD = 5.51 s`, falsas alarmas `= 0`; la deriva de R1 en la detección es `0.218 m` y las derivas máximas/finales de R1 y R2 alcanzan `7.032 m` y `8.979 m`.

El resultado más importante no es una diferencia puntual entre E5 y E6, sino que el detector identifica el ataque incluso cuando se modifica la navegación de uno o de los dos robots. Para evitar la detección, un atacante tendría que mantener una estructura geométrica coherente entre todos los receptores y con la distancia física medida por UWB. A medida que aumenta el número de robots, aumenta el número de restricciones relativas que el atacante debe satisfacer simultáneamente, haciendo más compleja la construcción de un ataque geométricamente consistente.

La conclusión debe mantenerse dentro del alcance de la validación: el prototipo demuestra la viabilidad del principio en SITL para el modelo de drag-off implementado. No constituye una garantía contra todos los ataques GNSS ni una validación de hardware real.

---

## 2. Contexto y problema de seguridad

### 2.1 GNSS en navegación autónoma

Los robots autónomos utilizan GNSS para estimar su posición global. Un controlador puede transformar esa posición en comandos de velocidad y utilizarla para seguir una ruta, mantener una formación o coordinarse con otros vehículos. La seguridad de esa información es crítica: una posición incorrecta no sólo degrada la localización, sino que puede provocar una acción física incorrecta.

La autenticación de mensajes GNSS no resuelve necesariamente todos los ataques de retransmisión. En un ataque de meaconing, el atacante puede retransmitir una señal legítima con un retardo o una modificación temporal. El receptor puede observar una señal válida desde el punto de vista criptográfico, pero asociarla con una posición incorrecta. Por ello son necesarias comprobaciones de consistencia independientes de la autenticidad del mensaje.

### 2.2 Diferencia entre spoofing y meaconing

En el uso cotidiano del proyecto aparecen las palabras `spoofed`, `spoofing` y `meaconed`. Los nombres de los topics conservan `gnss_spoofed` por compatibilidad con la primera implementación, pero el modelo de ataque que se estudia es meaconing:

1. El atacante observa la señal GNSS legítima.
2. La retarda o reinyecta gradualmente.
3. El receptor informa de una posición que se aleja de la verdadera.
4. En el caso de una antena común, varios receptores pueden converger hacia una posición falsa semejante.

El nodo `meaconing_injector.py` aproxima este comportamiento mediante un **drag-off** software: a partir de la posición limpia en el inicio del ataque, arrastra la posición publicada hacia el punto medio entre ambos robots a una velocidad configurable y añade ruido GNSS independiente.

### 2.3 Problema que resuelve el proyecto

Un único robot no puede determinar con facilidad si su posición GNSS es incorrecta sólo observando el mensaje recibido. Dos robots próximos pueden comparar una magnitud relativa que se obtiene de dos sensores distintos:

- GNSS: estimación vulnerable a meaconing.
- UWB: distancia radioeléctrica local, simulada aquí como medida no afectada por GNSS.

La pregunta de investigación puede formularse así:

> ¿Puede la comparación colaborativa entre la distancia GNSS y la distancia UWB, procesada mediante un detector CUSUM secuencial, identificar un meaconing persistente con un tiempo de detección razonable y sin falsas alarmas ante transitorios normales de navegación?

---

## 3. Objetivos

### 3.1 Objetivo general

Demostrar mediante simulación SITL que una pareja de robots móviles puede detectar un ataque de meaconing GNSS comparando la distancia estimada con GNSS y la distancia física medida con UWB, utilizando un detector CUSUM de dos colas.

### 3.2 Objetivos específicos

1. Construir un entorno reproducible con dos TurtleBot3 Waffle en Gazebo Sim.
2. Publicar posiciones GNSS simuladas con ruido Gaussiano.
3. Publicar una distancia UWB simulada a partir de la posición física de los robots.
4. Implementar un inyector de meaconing activable automáticamente o mediante servicio ROS 2.
5. Implementar un detector secuencial CUSUM con ramas positiva y negativa.
6. Calibrar el sesgo normal producido por el cálculo de la norma GNSS.
7. Evaluar falsos positivos, tiempo de detección y respuesta ante distintos escenarios.
8. Comparar el comportamiento en movimiento circular y en navegación por waypoints.
9. Registrar los datos con rosbag2 y generar gráficas reproducibles.
10. Extender E5 a E6, donde ambos robots navegan utilizando GNSS meaconado.

---

## 4. Contribución y relevancia

La contribución del proyecto no es crear un nuevo receptor GNSS ni modificar el protocolo GNSS. Es construir una **capa de monitorización colaborativa** que use un sensor relativo independiente para descubrir incoherencias entre robots.

La idea es relevante por cuatro motivos:

1. **Independencia de fuentes:** el detector no confía únicamente en GNSS; contrasta GNSS con UWB.
2. **Detección de ataques lentos:** CUSUM acumula pequeñas discrepancias persistentes que un umbral aplicado a una muestra aislada puede ignorar.
3. **Aplicación a flotas:** la información relativa entre robots está disponible en escenarios cooperativos, formaciones y enjambres.
4. **Implementación accesible:** el prototipo usa sensores y mensajes que pueden aproximarse con hardware comercial, sin requerir acceso a códigos GNSS militares.

La relevancia debe expresarse con precisión: el sistema es un detector de consistencia para un modelo de ataque concreto. No autentica por sí mismo la señal GNSS y no elimina los puntos ciegos de un atacante capaz de producir posiciones falsas coherentes con la geometría física real.

---

## 5. Arquitectura general

### 5.1 Capas del sistema

La arquitectura se divide en cinco capas:

1. **Capa física:** Gazebo Sim ejecuta el mundo vacío y los dos TurtleBot3.
2. **Capa de movimiento:** `robot_mover_node.py` genera movimiento circular o `waypoint_follower_node.py` controla rutas mediante Pure Pursuit.
3. **Capa sensorial:** `gnss_sim_node.py` y `uwb_sim_node.py` convierten la odometría en mediciones simuladas.
4. **Capa de ataque:** `meaconing_injector.py` pasa las mediciones GNSS sin cambios o publica posiciones arrastradas hacia un objetivo común.
5. **Capa de detección y análisis:** `cusum_detector_node.py` calcula la innovación y la alarma; rosbag2 y los scripts de análisis generan métricas y gráficas.

### 5.2 Flujo de datos

```text
Gazebo / odometría real
        |
        +--> GNSS simulator --> /robot1/gnss_clean
        |                       /robot2/gnss_clean
        |                              |
        |                              v
        |                    Meaconing injector
        |                              |
        |                              +--> /robot1/gnss_spoofed
        |                              +--> /robot2/gnss_spoofed
        |
        +--> UWB simulator --> /robots/uwb_distance

/robot1/gnss_spoofed + /robot2/gnss_spoofed
                  + /robots/uwb_distance
                              |
                              v
                    CUSUM detector
                              |
       /system/cusum_* + /system/meaconing_alert
```

### 5.3 Topics principales

| Topic | Tipo | Papel |
|---|---|---|
| `/robot1/odom`, `/robot2/odom` | `nav_msgs/msg/Odometry` | Posición física simulada y trayectoria real. |
| `/robot1/gnss_clean`, `/robot2/gnss_clean` | `geometry_msgs/msg/PoseStamped` | GNSS simulado con ruido antes del ataque. |
| `/robot1/gnss_spoofed`, `/robot2/gnss_spoofed` | `geometry_msgs/msg/PoseStamped` | GNSS que consume el detector y el controlador. Puede ser limpio o meaconado. |
| `/robots/uwb_distance` | `std_msgs/msg/Float64` | Distancia física entre robots con ruido UWB. |
| `/system/delta_raw` | `std_msgs/msg/Float64` | Innovación sin calibrar. |
| `/system/delta_value` | `std_msgs/msg/Float64` | Innovación corregida por línea base y filtrada. |
| `/system/cusum_plus` | `std_msgs/msg/Float64` | Rama que detecta `D_GNSS` demasiado pequeña. |
| `/system/cusum_minus` | `std_msgs/msg/Float64` | Rama que detecta `D_GNSS` demasiado grande. |
| `/system/cusum_value` | `std_msgs/msg/Float64` | Máximo de las dos ramas. |
| `/system/meaconing_alert` | `std_msgs/msg/Bool` | Alarma confirmada. |
| `/meaconing/active` | `std_msgs/msg/Bool` | Estado de activación del atacante. | 
| `/meaconing/activation_event` | `std_msgs/msg/Float64` | Marca única publicada en el callback exacto de activación; se usa para medir TTD. |

### 5.4 Servicios

- `/meaconing/set_active`, tipo `std_srvs/srv/SetBool`: activa o desactiva manualmente el ataque.
- `/system/reset_cusum`, tipo `std_srvs/srv/Trigger`: reinicia estado y calibración del detector.

---

## 6. Simulación física y coordinación de robots

### 6.1 Gazebo y TurtleBot3

`two_robots.launch.py` inicia dos modelos TurtleBot3 Waffle en un mundo vacío. Cada modelo se denomina `robot1` o `robot2` y se crea con posiciones independientes.

Una dificultad importante fue que el plugin DiffDrive y los puentes de Gazebo podían publicar topics globales compartidos como `/odom` o `/cmd_vel`. Si ambos robots compartían esos topics, los nodos podían recibir datos duplicados o cruzados. La solución fue:

1. Leer el SDF original del TurtleBot3.
2. Sustituir los topics de Gazebo por nombres específicos del modelo, por ejemplo `/model/robot1/odom`.
3. Crear un bridge independiente por robot.
4. Publicar en topics ROS absolutos `/robot1/odom` y `/robot2/odom`.

Esta separación es esencial para que la distancia UWB y las trayectorias pertenezcan a robots distintos.

### 6.2 Marcos de coordenadas

La odometría de cada DiffDrive empieza en un marco local aproximadamente centrado en `(0, 0)`, aunque el robot se haya creado en otra posición del mundo. Para que las dos fuentes de distancia sean comparables, GNSS y UWB suman el desplazamiento de spawn:

```text
world_x = odom_x + spawn_x
world_y = odom_y + spawn_y
```

La misma conversión se aplica al análisis de las trayectorias. Este detalle resolvió un problema inicialmente observado como distancia UWB cercana a cero o trayectorias superpuestas.

### 6.3 Movimiento circular: E0-E4

`robot_mover_node.py` publica `TwistStamped` a 20 Hz con velocidades lineales y angulares constantes. Cada robot describe una circunferencia de radio aproximado:

```text
radio = velocidad lineal / velocidad angular
```

Los parámetros por defecto son:

- Robot 1: `v = 0.15 m/s`, `omega = 0.30 rad/s`, radio aproximado `0.50 m`.
- Robot 2: `v = 0.12 m/s`, `omega = 0.25 rad/s`, radio aproximado `0.48 m`.

La diferencia de velocidad y radio produce una distancia inter-robot variable, haciendo el escenario más realista que una geometría completamente estática.

### 6.4 Navegación por waypoints: E5 y E6

`waypoint_follower_node.py` sustituye al movimiento circular en E5 y E6. Implementa un controlador **Pure Pursuit**:

1. Proyecta la posición actual sobre el segmento de ruta.
2. Busca un punto de seguimiento a una distancia `lookahead`.
3. Transforma ese punto del mundo al marco del robot.
4. Calcula una curvatura proporcional al desplazamiento lateral.
5. Genera velocidad lineal y angular, reduciendo la velocidad en curvas pronunciadas.
6. Avanza al siguiente waypoint cuando la distancia es menor que `waypoint_arrival_dist`.

Las rutas son:

```text
Robot 1: (5,0) -> (5,5) -> (0,5)
Robot 2: (5,2) -> (5,7) -> (0,7)
```

La separación inicial de 2 m en el eje Y facilita que la distancia física permanezca en un rango útil para observar el desacuerdo con GNSS.

En E5, robot 1 usa `/robot1/gnss_spoofed` y robot 2 usa `/robot2/gnss_clean`. En E6 ambos usan sus respectivos topics `gnss_spoofed`, mediante el parámetro:

```yaml
r2_gnss_source: clean    # E5
r2_gnss_source: spoofed  # E6
```

Esto permite medir no sólo una alarma estadística, sino también la deriva física causada por un controlador que confía en una posición GNSS manipulada.

---

## 7. Modelos sensoriales

### 7.1 GNSS simulado

`gnss_sim_node.py` recibe odometría física, convierte al marco global, añade ruido independiente en X e Y y publica `PoseStamped`:

\[
\mathbf{p}^{GNSS}_i = \mathbf{p}^{real}_i + \boldsymbol{\epsilon}_i,
\]

con:

\[
\epsilon_{i,x}, \epsilon_{i,y} \sim \mathcal{N}(0, \sigma_{GNSS}^2).
\]

La configuración usa:

```text
sigma_gnss = 1.0 m
update_rate = 30 Hz
random_seed = 42
```

El valor de 1 m representa un GNSS civil con correcciones razonables dentro del modelo del proyecto. No debe interpretarse como una caracterización universal de cualquier receptor real.

### 7.2 UWB simulado

`uwb_sim_node.py` obtiene las posiciones físicas de Gazebo, suma los desplazamientos de spawn y calcula:

\[
D_{real} = \left\|\mathbf{p}^{real}_1 - \mathbf{p}^{real}_2\right\|.
\]

Publica:

\[
D_{UWB} = D_{real} + \epsilon_{UWB},
\qquad \epsilon_{UWB} \sim \mathcal{N}(0, \sigma_{UWB}^2),
\]

con `sigma_uwb = 0.24 m`, valor motivado en el proyecto por resultados de ranging UWB comercial reportados en MURP (Fishberg et al., 2024).

En el modelo actual UWB no es atacado. Por eso funciona como fuente independiente de referencia física. En un sistema real habría que considerar pérdida de paquetes, NLOS, multipath y ataques al propio ranging.

### 7.3 Reproducibilidad

GNSS, UWB y el inyector configuran la semilla NumPy a `42`. Esto facilita comparar experimentos, aunque hay una precisión importante: varios nodos inicializan su propio generador global, y el orden de ejecución y temporización puede afectar la secuencia efectiva. Por tanto, la semilla mejora la reproducibilidad del escenario, pero no sustituye a repetir el experimento varias veces para obtener intervalos estadísticos.

---

## 8. Modelo del ataque de meaconing

### 8.1 Inyector

`meaconing_injector.py` recibe las posiciones limpias de ambos robots. Mientras está inactivo, las republica como `gnss_spoofed`. Cuando se activa:

1. Captura las posiciones limpias actuales `p0_a` y `p0_b`.
2. Define como objetivo falso el punto medio:

\[
\mathbf{p}_{fake} = \frac{\mathbf{p}_{0,a} + \mathbf{p}_{0,b}}{2}.
\]

3. Calcula la distancia de cada robot a ese objetivo.
4. Avanza linealmente hacia el objetivo a `drift_velocity`.
5. Añade ruido GNSS a cada salida.
6. Publica ambos resultados a 30 Hz.

Para cada robot puede expresarse como:

\[
\mathbf{p}^{spoofed}_i(t) = \mathbf{p}_{0,i} + \alpha_i(t)(\mathbf{p}_{fake}-\mathbf{p}_{0,i}) + \boldsymbol{\epsilon}_i(t),
\]

con:

\[
\alpha_i(t) = \min\left(1, \frac{v_d t}{\|\mathbf{p}_{0,i}-\mathbf{p}_{fake}\|}\right).
\]

### 8.2 Ataques implementados

- `single_antenna`: modelo de drag-off común para los dos receptores.
- `pattern`: reconocido por la configuración, pero actualmente redirigido a la misma implementación de `single_antenna`; el ataque basado en patrones queda como trabajo futuro.

El nombre `single_antenna` indica el supuesto de una señal o punto falso común, no que sólo se modifique un robot. E5 y E6 diferencian qué controlador consume la medición meaconada:

- E5: sólo robot 1 navega con GNSS meaconado; robot 2 navega con GNSS limpio.
- E6: ambos robots navegan con GNSS meaconado.

### 8.3 Qué se espera observar

Bajo un ataque común, la distancia GNSS debería disminuir respecto a la distancia física. Idealmente:

\[
D_{GNSS} \rightarrow 0,
\qquad D_{UWB} \approx D_{real} > 0,
\]

por lo que:

\[
\delta = D_{UWB} - D_{GNSS} > 0.
\]

Sin embargo, en E5 y E6 los robots no son puntos matemáticos inmóviles: sus controladores reaccionan a la posición manipulada y cambian la trayectoria física. Por eso el ataque no produce necesariamente una señal perfectamente monótona ni un TTD menor en E6. La comparación E5-E6 debe interpretarse junto con las trayectorias y la evolución de `D_UWB`.

---

## 9. Detector CUSUM

### 9.1 Distancia GNSS e innovación

El detector calcula a partir de los dos mensajes `PoseStamped`:

\[
D_{GNSS}(k) = \sqrt{(x_1-x_2)^2 + (y_1-y_2)^2 + (z_1-z_2)^2}.
\]

La innovación sin calibrar es:

\[
\delta_{raw}(k) = D_{UWB}(k)-D_{GNSS}(k).
\]

La implementación publica este valor en `/system/delta_raw` para poder separar el fenómeno físico del tratamiento estadístico.

### 9.2 Por qué fue necesaria la calibración

Aunque el ruido de las coordenadas GNSS tiene media cero, la distancia es una norma no lineal. En general:

\[
E[\|\mathbf{p}+\boldsymbol{\epsilon}\|] \neq \|\mathbf{p}\|.
\]

Por tanto, la distancia GNSS puede presentar un sesgo de operación normal. En una versión anterior, el CUSUM absoluto acumulaba ese sesgo y generaba una alarma de la rama negativa antes de iniciar el ataque. En el rosbag histórico del E5 se observó una alerta en `4.326 s` mientras el topic de ataque se marcaba activo en `20.933 s`; el acumulador negativo llegó aproximadamente a `42.5`.

La solución implementada fue estimar la mediana de `delta_raw` durante `startup_delay`, siempre que el topic `/meaconing/active` indique que no hay ataque:

\[
\delta_0 = median\{\delta_{raw}(k): k \in startup\}.
\]

Después se usa:

\[
\delta(k) = \delta_{raw}(k)-\delta_0.
\]

El valor corregido se almacena en una ventana de longitud `filter_window` y se calcula:

\[
\bar{\delta}(k) = \frac{1}{W}\sum_{j=0}^{W-1}\delta(k-j).
\]

La implementación publica `delta_value = delta_filtered`, no el valor crudo. Esta diferencia debe explicarse en el informe para no confundir las gráficas.

### 9.3 CUSUM de dos colas

La rama positiva detecta el colapso de la distancia GNSS:

\[
S^+_k = \max\left(0, S^+_{k-1} + \bar{\delta}(k)-\beta\right).
\]

La rama negativa detecta una distancia GNSS excesivamente grande:

\[
S^-_k = \max\left(0, S^-_{k-1} - \bar{\delta}(k)-\beta\right).
\]

El valor publicado como estadístico global es:

\[
S_k = \max(S^+_k,S^-_k).
\]

En el ataque de una antena, la rama esperada es `S_plus`. La rama `S_minus` permite extender el detector a ataques que inflen artificialmente `D_GNSS`.

### 9.4 Confirmación temporal

Cruzar `tau` no equivale inmediatamente a emitir la alarma. Cuando `S_k > tau`, el detector guarda `candidate_since`. La alarma sólo se confirma si el estadístico continúa sobre el umbral durante al menos:

```text
alert_confirm_time = 2.0 s
```

Si el estadístico baja antes, la candidatura se borra. Este mecanismo permite que excursiones transitorias producidas por la navegación normal no se conviertan en alarmas confirmadas, aunque el estadístico supere temporalmente el umbral.

### 9.5 Interpretación de los parámetros

- **Beta:** controla la deriva mínima que debe superar la innovación filtrada. Un beta demasiado pequeño hace que el ruido acumule evidencia; uno demasiado grande retrasa o impide detectar ataques lentos.
- **Tau:** controla el compromiso entre sensibilidad y falsas alarmas. Un tau bajo acelera la detección, pero aumenta cruces espurios.
- **Filter window:** reduce la variabilidad rápida. Una ventana mayor mejora la estabilidad, pero añade latencia y puede ocultar ataques muy breves.
- **Alert confirmation:** rechaza cruces transitorios. Aumentarlo reduce falsas alarmas, pero retrasa la alarma real y puede perder ataques que no permanezcan activos.
- **Startup delay:** permite estimar el punto de operación. Si el ataque está activo durante la calibración, la línea base queda contaminada; E3 es precisamente un caso que obliga a tratar este problema con cuidado.

La configuración actual es razonable para los datos disponibles. E0 no cruza el umbral y E5 reference sí presenta cruces, pero todos son rechazados por la confirmación. No hay evidencia en los rosbags de que sea necesario cambiar `beta`, `tau` o `alert_confirm_time` para estos escenarios. La calibración no debe considerarse universal: habría que repetirla con diferentes semillas, geometrías, pérdidas de mensajes y perfiles de ruido.

---

## 10. Lanzamiento, parámetros y automatización

### 10.1 `experiment.launch.py`

Este launch inicia, con retardos para permitir que Gazebo esté disponible:

| Tiempo aproximado | Acción |
|---:|---|
| 0 s | Gazebo Sim y configuración de dos robots. |
| 2-3 s | Creación de robot 1 y robot 2 y sus bridges. |
| 5 s | Simulador GNSS y simulador UWB. |
| 5.5 s | Inyector de meaconing. |
| 6 s | Detector CUSUM. |
| 7 s | `robot_mover_node` o `waypoint_follower_node`. |

En modo waypoint, se desactiva el robot mover para evitar dos publicadores compitiendo por `/cmd_vel`.

### 10.2 `params.yaml`

El archivo usa el wildcard `/**`, de modo que los parámetros se aplican a los nodos. Contiene ruido, CUSUM, ataque, posiciones de spawn, movimiento circular y rutas waypoint.

### 10.3 `run_experiment.sh`

El script ejecuta una sola prueba de extremo a extremo:

1. Mata procesos residuales.
2. Restaura los parámetros base.
3. Aplica las modificaciones del escenario.
4. Construye el paquete con `colcon`.
5. Copia y verifica `params.yaml` en el árbol `install`.
6. Inicia rosbag2 antes del launch para conservar un pre-roll de arranque.
7. Lanza Gazebo y la pila ROS.
8. Graba topics con rosbag2 en formato MCAP, incluyendo `/meaconing/activation_event`.
9. Detiene la grabación y todos los procesos.
10. Guarda una instantánea de los parámetros usados.

Este flujo solucionó un problema histórico importante: una compilación accidental desde un subdirectorio podía crear `src/install`, `src/build` y `src/log`, que contaminaban `AMENT_PREFIX_PATH` y hacían que los nodos leyeran parámetros antiguos. El script incorpora una guardia para eliminar esos árboles anidados.

El bug específico de E6 se produjo al escribir un string mediante una función que esperaba una expresión Python. El valor `"'spoofed'"` generaba comillas anidadas en el `print`. Se añadió `set_param_str`, que escribe correctamente `r2_gnss_source: spoofed` como string YAML.

---

## 11. Diseño y justificación de los experimentos

Cada experimento responde a una pregunta diferente. El tiempo de los rosbags se expresa desde el inicio de la captura, no desde el arranque absoluto de Gazebo. En los bags históricos la grabación comenzaba después de la espera inicial de 15 s, por lo que un ataque configurado a 30 s aparecía cerca de 20-21 s dentro del bag. Las nuevas ejecuciones empiezan a grabar antes del launch y conservarán un pre-roll de arranque; por tanto, sus eventos aparecerán más cerca del tiempo absoluto relativo al inicio del launch.

### E0: baseline sin ataque

**Configuración:** `activation_delay = 9999 s`; movimiento circular; separación inicial de 3 m.  
**Pregunta:** ¿El detector permanece silencioso durante una operación normal prolongada?

Es el control negativo del estudio. Permite estimar el comportamiento de los acumuladores ante ruido GNSS, ruido UWB y movimiento relativo sin meaconing. El criterio de éxito es no confirmar ninguna alarma.

### E1: deriva lenta

**Configuración:** `drift_velocity = 0.1 m/s`, activación a 30 s.  
**Pregunta:** ¿Puede el CUSUM detectar un ataque persistente y gradual?

Representa un ataque menos abrupto. El interés no es sólo alcanzar una discrepancia grande, sino comprobar que la acumulación secuencial descubre una deriva que puede no superar un umbral fijo en cada muestra.

### E2: deriva rápida

**Configuración:** `drift_velocity = 0.5 m/s`, activación a 30 s.  
**Pregunta:** ¿La velocidad del drag-off reduce el tiempo de detección?

Sirve como control positivo fuerte. Se espera una subida más rápida que en E1, aunque la respuesta también depende de la trayectoria y de la geometría instantánea.

### E3: hot start

**Configuración:** activación temprana (`activation_delay = 2 s` en el snapshot), `startup_delay = 3 s`.
**Pregunta:** ¿Qué ocurre si el sistema comienza prácticamente bajo ataque?

Este caso examina el arranque adversarial. La versión actual del runner inicia rosbag2 antes del launch y registra `/meaconing/activation_event`, por lo que la última campaña permite medir el TTD de forma explícita. El escenario sigue siendo exigente porque la activación ocurre antes de que finalice la fase de calentamiento nominal.

### E4: separación amplia

**Configuración:** robot 2 en `x = 5 m`, activación a 30 s, `startup_delay = 5 s`.  
**Pregunta:** ¿Cómo cambia la detección cuando la separación física es mayor?

Una separación mayor genera una discrepancia potencial más grande cuando GNSS colapsa, pero también modifica la distribución normal de la distancia GNSS y del sesgo. Es un análisis de sensibilidad geométrica.

### E5 reference: ruta por waypoints sin ataque

**Configuración:** ambos robots siguen rutas desplazadas 2 m; `activation_delay = 9999 s`; robot 1 y robot 2 usan GNSS limpio.  
**Pregunta:** ¿El controlador Pure Pursuit y la geometría de waypoints producen cruces naturales del umbral?

Es el control negativo específico de E5. Es deliberadamente más exigente que E0 porque la navegación por waypoints genera cambios de orientación, curvas y variación de distancia. Permite evaluar la confirmación temporal.

### E5: un robot navega con GNSS meaconado

**Configuración:** robot 1 usa `gnss_spoofed`, robot 2 usa `gnss_clean`; `drift_velocity = 0.2 m/s`; activación a 30 s.  
**Pregunta:** ¿La misma discrepancia estadística produce deriva física de un robot antes de que llegue la alarma?

Este es el experimento de demostración más cercano al caso de uso: el robot 1 toma decisiones de navegación basadas en una posición alterada, mientras el robot 2 proporciona una referencia física y de navegación no atacada.

### E6: ambos robots meaconados

**Configuración:** mismo escenario waypoint que E5, pero `r2_gnss_source = spoofed`; ambos controladores consumen GNSS meaconado.  
**Pregunta:** ¿Qué cambia cuando la manipulación afecta a los dos controladores?

E6 evalúa un ataque más amplio. No se debe asumir que la alarma será más rápida: si los dos robots modifican sus trayectorias, cambia simultáneamente la separación UWB, la geometría de las rutas y el residuo `delta`. Por ello el resultado debe analizarse junto con `D_UWB`, las dos trayectorias y la rama CUSUM activa.

---

## 12. Resultados medidos en los nuevos rosbags

### 12.1 Resumen cuantitativo

Los valores siguientes corresponden a la última campaña, generada después de volver a ejecutar todos los experimentos. El análisis utiliza las marcas de activación disponibles en los nuevos rosbags y cuenta como falsa alarma cualquier `True` de `/system/meaconing_alert` anterior a la activación.

| Caso | Ataque | TTD | Falsas alarmas | Resultado |
|---|---:|---:|---:|---|
| E0 baseline | Nunca | N/A | 0 | Operación normal sin alarma. |
| E1 slow drift | 32.0 s | 2.71 s | 0 | Ataque lento detectado. |
| E2 fast drift | 32.5 s | 3.93 s | 0 | Ataque rápido detectado. |
| E3 hot start | 4.6 s | 4.34 s | 0 | Ataque de arranque detectado. |
| E4 wide separation | 32.5 s | 2.73 s | 0 | Ataque detectado con separación de 5 m. |
| E5 reference | Nunca | N/A | 0 | Navegación por waypoints sin ataque. |
| E5 waypoint attack | 32.5 s | 5.70 s | 0 | R1 atacado; alarma confirmada. |
| E6 dual meaconing | 32.4 s | 5.51 s | 0 | Ambos robots atacados; alarma confirmada. |

El resultado global es consistente: los dos escenarios sin ataque no producen falsas alarmas y los seis escenarios con ataque producen detección. La campaña respalda que la calibración de la línea base, el CUSUM de dos colas y la confirmación temporal funcionan conjuntamente bajo las geometrías evaluadas.

### 12.2 E0: baseline sin ataque

E0 permanece sin alarma durante toda la ejecución y registra cero falsas alarmas. Es el control negativo principal: el movimiento circular y el ruido de GNSS/UWB no producen una condición que el detector confirme como meaconing.

La conclusión debe expresarse como resultado de esta ejecución, no como una FAR universal. Para estimar una tasa de falsas alarmas con significación estadística habría que repetir E0 con varias semillas, duraciones y geometrías.

### 12.3 E1 y E2: sensibilidad a la velocidad de ataque

E1 utiliza una deriva lenta de `0.1 m/s` y produce `TTD = 2.71 s`. E2 utiliza `0.5 m/s` y produce `TTD = 3.93 s`. En esta campaña E1 resulta algo más rápido, por lo que no es correcto afirmar que una mayor velocidad nominal siempre reduce el TTD.

La razón es que el TTD depende de la señal observada por el detector, no sólo de `drift_velocity`: también influyen la geometría instantánea, la distancia UWB, el ruido, la calibración y la respuesta de los robots. La conclusión defendible es que ambos niveles de deriva son detectables, incluso cuando el ataque es gradual.

### 12.4 E3: hot start

E3 produce `TTD = 4.34 s` y cero falsas alarmas. Gracias al registro con pre-roll y al evento de activación, este resultado es utilizable: ya no debe describirse como un TTD censurado o como una alarma aparentemente inmediata causada por el inicio tardío del rosbag.

El caso demuestra que el detector puede iniciar su operación normal, calibrar la línea base durante el periodo configurado y detectar posteriormente un ataque que se activa muy al principio de la ejecución. La interpretación exacta debe conservar la diferencia entre `activation_delay = 2 s` y `startup_delay = 3 s`: el escenario es adversarial porque la activación ocurre antes de que finalice la fase de calentamiento nominal.

### 12.5 E4: separación amplia

Con robot 2 situado a 5 m, E4 produce `TTD = 2.73 s` y cero falsas alarmas. La mayor separación ofrece una discrepancia GNSS-UWB más visible cuando el GNSS es arrastrado, pero también cambia la distribución normal de las distancias. El resultado muestra que el detector continúa funcionando al cambiar la geometría inicial.

### 12.6 E5 reference: waypoints sin ataque

E5 reference no genera ninguna alarma ni falsa alarma. Las gráficas muestran las trayectorias de ambos robots siguiendo las rutas desplazadas y sirven como control para interpretar la deriva física de E5 y E6.

Este caso es importante porque somete al detector a curvas, cambios de orientación y movimiento waypoint, no sólo al movimiento circular. La ausencia de alarma confirma que la navegación normal no se confunde con meaconing en esta ejecución.

### 12.7 E5: un robot atacado y deriva física

E5 produce `TTD = 5.70 s` sin falsas alarmas. La gráfica de deriva física muestra:

- deriva en el momento de activar el ataque: `0.008 m`;
- deriva en el momento de la detección: `0.091 m`;
- deriva máxima y final registrada: `6.603 m`.

Esto es una evidencia relevante para el objetivo de seguridad: la alarma se confirma cuando el robot todavía está muy cerca de la trayectoria de referencia, mientras que la desviación crece de forma pronunciada si el ataque continúa. La detección estadística precede al daño físico máximo.

La gráfica de trayectorias muestra que R1, guiado por GNSS meaconado, abandona progresivamente la ruta prevista, mientras R2 mantiene la navegación basada en GNSS limpio. Esta separación entre estado físico real, estado reportado y respuesta del controlador es la principal aportación visual de E5.

### 12.8 E6: ambos robots atacados

E6 produce `TTD = 5.51 s` y cero falsas alarmas. La gráfica de deriva física muestra, al final de la ejecución:

- deriva máxima/final de R1: `7.032 m`;
- deriva máxima/final de R2: `8.979 m`;
- deriva de R1 en la activación: `0.057 m`;
- deriva de R1 en la detección: `0.218 m`.

El interés de E6 no es compararlo numéricamente con E5, sino mostrar que el mecanismo sigue detectando el ataque cuando ambos lazos de navegación consumen GNSS meaconado. La manipulación modifica las dos trayectorias y, por tanto, la geometría física evoluciona durante el ataque.

### 12.9 Geometría y escalabilidad de la detección

El detector no depende únicamente de que un robot tenga una posición GNSS incorrecta. Comprueba si las relaciones geométricas reportadas por GNSS son compatibles con las distancias físicas medidas por UWB. Para evitar la detección, un atacante tendría que construir posiciones falsas que mantuvieran simultáneamente una estructura geométrica compatible con todos los robots y con sus distancias relativas reales.

Con dos robots existe una restricción principal: la distancia entre ambos. Con una flota de `N` robots aparecen múltiples relaciones de distancia, y una red de ranging aporta aproximadamente `N(N-1)/2` pares potenciales si todos los robots se miden entre sí. El atacante tendría que conservar una configuración GNSS falsa coherente en todas esas relaciones, no sólo desplazar a todos los receptores hacia un punto común.

Esto permite formular una conclusión de seguridad razonable: **cuantos más robots participen en la cooperación, más difícil resulta para el atacante mantener una estructura geométrica falsa que sea consistente con todos los sensores relativos**. La afirmación supone que las mediciones UWB son suficientemente independientes y que el atacante no controla simultáneamente la red de ranging. No implica invulnerabilidad frente a un atacante capaz de falsificar todos los sensores o de producir una transformación geométrica global perfectamente coherente.

---

## 13. Interpretación de las nuevas gráficas

### 13.1 `cusum_evolution.png`

La figura reúne la evolución del CUSUM de los ocho escenarios. Las marcas de activación y las regiones de alarma permiten distinguir operación normal, ataque y confirmación.

Lectura recomendada:

- E0 y E5 reference permanecen sin alarma confirmada.
- E1, E2 y E4 muestran detección después de la activación en movimiento circular.
- E3 muestra una detección medible pese a la activación temprana.
- E5 y E6 producen una subida persistente del estadístico después del ataque y confirman la alarma.
- La diferencia exacta de TTD entre E5 y E6 no debe interpretarse como una clasificación de ataques; el resultado relevante es que ambos escenarios son detectables bajo la geometría simulada.

### 13.2 Gráficas `threshold_vs_cusum_*`

Estas figuras comparan una decisión basada en un umbral fijo con la evolución del CUSUM. Un umbral fijo aplicado a una innovación instantánea puede responder a picos aislados. El CUSUM incorpora memoria: acumula sólo la parte de la innovación que supera `beta` y se reinicia hacia cero cuando la evidencia cambia de signo o deja de ser suficiente.

Debe aclararse que en versiones históricas del README la línea de innovación se describía como valor absoluto. La implementación final usa innovación firmada, corregida por línea base y filtrada. El informe debe basarse en esta última definición.

### 13.3 Gráficas de distancia UWB

Las gráficas `uwb_distance_*.png` muestran la distancia física, que no se altera directamente por el inyector GNSS. En E0-E4 circular, la distancia oscila en torno a la geometría de spawn. En E5 y E6 la distancia depende de las rutas waypoint y de la respuesta de los controladores.

El comportamiento de UWB es importante porque impide interpretar una subida de `S_k` como una simple variación de GNSS sin contexto físico. Si `D_UWB` cambia al mismo tiempo, la innovación puede aumentar o disminuir aun cuando el ataque sea idéntico.

### 13.4 `e5_physical_drift_e5_waypoint_attack.png`

Muestra la distancia entre la trayectoria física atacada de R1 y la trayectoria de referencia sin ataque. Incluye:

- activación del ataque;
- primera alerta confirmada;
- TTD;
- deriva estimada en el momento de detección cuando los datos están disponibles.

Esta gráfica conecta la métrica estadística con el impacto operacional: cuánto se ha separado el robot de la ruta que habría seguido sin ataque antes de que el sistema lo identifique.

### 13.5 `e6_physical_drift_e6_dual_meaconing.png`

Muestra por separado la deriva física de R1 y R2 respecto a la referencia waypoint. Es la figura adecuada para explicar que E6 afecta a dos lazos de control, no sólo a dos mensajes de entrada del detector.

### 13.6 Geometría multirobot

La comparación puntual entre E5 y E6 no se considera una métrica principal del proyecto. Lo relevante es que ambos escenarios son detectables incluso cuando el ataque afecta a uno o a los dos lazos de navegación.

La evidencia gráfica muestra que las posiciones GNSS manipuladas dejan de ser compatibles con la geometría física medida por UWB. En una flota mayor, el atacante tendría que mantener simultáneamente más relaciones de distancia y una estructura geométrica global coherente. Por ello, la cooperación multirobot aumenta la dificultad de construir un meaconing que pase todas las comprobaciones relativas, siempre que el canal UWB y parte de los sensores permanezcan fuera del control del atacante.

---

## 14. Dificultades encontradas y soluciones

### 14.1 Sesgo de la distancia GNSS

**Problema:** el ruido de posición de media cero no produce una distancia de media cero. La norma introduce un sesgo y el CUSUM puede acumularlo.

**Síntoma:** falsa alarma en la rama negativa antes del ataque, con `S_minus` creciendo de forma sostenida.

**Solución:** calibración de la mediana durante `startup_delay`, innovación firmada, dos ramas y topic separado `delta_raw` para diagnóstico.

### 14.2 Confusión entre detección y predicción

El detector no predice un ataque antes de que ocurra. Sólo puede confirmar una anomalía después de observarla. Una alerta con timestamp anterior al topic `/meaconing/active` suele indicar un problema de sincronización, una línea base incorrecta o un rosbag antiguo, no capacidad predictiva.

### 14.3 Sincronización y frecuencia de topics

ROS 2 ejecuta callbacks y timers de forma asíncrona. GNSS, UWB, ataque y detector no publican exactamente en el mismo instante. Además, el detector puede recibir la última muestra disponible de cada sensor. Por eso la comparación de timestamps debe tolerar pequeñas diferencias.

El inyector se ajustó a 30 Hz. Una versión anterior republicaba a 10 Hz, lo que reducía la efectividad real de la ventana móvil de 30 muestras y dejaba más ruido correlacionado. Igualar la frecuencia de publicación mejoró la coherencia del filtro.

### 14.4 Topics compartidos entre robots

Gazebo podía entregar odometría y comandos indistinguibles para los dos robots. Se resolvió parcheando el SDF y generando bridges con topics específicos por modelo.

### 14.5 Marcos locales y mundo global

La odometría empezaba en `(0,0)` para ambos robots aunque sus posiciones físicas fueran diferentes. Sin sumar los offsets de spawn, el UWB simulado podía calcular una separación errónea. La conversión explícita a world frame se incorporó en GNSS, UWB y los scripts de análisis.

### 14.6 Transporte DDS en macOS

FastDDS Shared Memory producía bloqueos o errores al arrancar muchos nodos simultáneamente. Se creó `fastdds_udp_only.xml` y los launch files lo aplican mediante `FASTRTPS_DEFAULT_PROFILES_FILE`. La configuración desactiva SHM y deja UDPv4 como transporte.

### 14.7 Parámetros obsoletos en el árbol de instalación

ROS 2 puede ejecutar la copia instalada del paquete, no necesariamente el archivo fuente que se acaba de editar. El runner reconstruye, copia explícitamente el YAML a `install` y verifica que ambos archivos coincidan. También elimina instalaciones anidadas que podían sombrear la instalación correcta.

### 14.8 Error de strings al configurar E6

La función original para parámetros numéricos insertaba el valor mediante `eval`. Al escribir `'spoofed'` generaba una expresión inválida con comillas dobles. La solución fue separar `set_param` de `set_param_str`.

### 14.9 E3 y la ventana de grabación

En las primeras ejecuciones E3 el ataque aparecía al inicio del rosbag y el TTD quedaba censurado por el comienzo tardío de la grabación. Se corrigió iniciando rosbag2 antes del launch y registrando `/meaconing/activation_event`. En la campaña actual E3 produce un TTD medible de `4.34 s`.

### 14.10 Tests automatizados

La versión actual incorpora seis tests unitarios y `colcon test` los ejecuta correctamente. Cubren las dos ramas del CUSUM, el reinicio ante cambio de signo, la ausencia de medición en Pure Pursuit, el seguimiento rectilíneo y el avance de waypoint. Esta cobertura no sustituye las pruebas de integración con Gazebo ni la validación estadística con múltiples semillas, pero elimina una parte de la deuda de verificación de la lógica central.

---

## 15. Limitaciones científicas y de ingeniería

1. **SITL, no hardware real:** GNSS y UWB son modelos de ruido, no mediciones capturadas en robots.
2. **UWB idealizado:** no se modelan NLOS, multipath, pérdidas, interferencias ni ataques al sensor.
3. **Ataque simplificado:** `pattern` aún no implementa una geometría específica; el inyector usa un objetivo medio común.
4. **Una pareja de robots:** no se evalúa escalabilidad a enjambres mayores.
5. **Una sola semilla registrada:** los números son resultados de ejecuciones concretas, no intervalos de confianza.
6. **FAR insuficientemente estimada:** E0 dura aproximadamente 75 s en el bag y no permite estimar una tasa anual o una probabilidad robusta.
7. **Calibración estacionaria:** la mediana se estima al inicio y no se adapta a cambios prolongados de geometría o entorno.
8. **Posibles puntos ciegos:** un atacante con capacidad de generar posiciones GNSS que mantengan la distancia relativa correcta puede no ser detectado por este residuo.
9. **Dependencia del controlador:** E5 y E6 mezclan detección y respuesta de navegación; las trayectorias pueden cambiar la señal que se quiere medir.
10. **Sin sincronización temporal estricta de sensores:** el análisis dispone de una marca precisa de activación y un pre-roll, pero el detector todavía combina las últimas muestras disponibles de GNSS y UWB. Una evaluación de fusión sensorial de mayor precisión requeriría timestamps en la medición UWB y asociación temporal aproximada.
11. **No hay mitigación automática:** la alarma se publica, pero el proyecto no implementa todavía parada segura, cambio de sensor, exclusión de GNSS o replanificación.
12. **No se modela autenticación GNSS:** el sistema complementa la autenticación, no la sustituye.

Estas limitaciones no invalidan el objetivo del TFM. Delimitan qué se demuestra: consistencia colaborativa frente a un ataque de drag-off en un entorno simulado y controlado.

### 15.1 Limitaciones con solución sencilla

No todas las limitaciones tienen el mismo coste. Algunas pueden reducirse con cambios pequeños y no requieren modificar la hipótesis principal del proyecto:

| Limitación | Solución de bajo coste | Beneficio |
|---|---|---|
| Falta de tests automatizados | Extraer el estado del CUSUM a una clase o función independiente y añadir tests con secuencias sintéticas de `delta` | Verificar calibración, ramas, umbral, reset y confirmación temporal sin arrancar Gazebo. |
| Timestamps asíncronos | Publicar `/meaconing/activation_event` en el callback exacto y usar su timestamp del rosbag para el análisis | Medir TTD y primer cruce sin depender sólo del primer `True` observado en un topic periódico. |
| E3 comienza bajo ataque | Iniciar rosbag2 antes del launch, con 2 s de pre-roll, y registrar el evento de activación | Medir el TTD completo y observar la fase de arranque. |
| Parámetros históricos en la documentación | Mantener una única tabla generada desde `params.yaml` y revisar los nombres antiguos (`waypoint_x`, `linear_gain`, etc.) | Evitar que una guía de ejecución describa una interfaz que el código ya no utiliza. |
| Una sola ejecución por escenario | Repetir cada caso con unas pocas semillas adicionales antes de hacer afirmaciones fuertes | Obtener una primera estimación de dispersión del TTD y de los máximos normales. |
| Alarma sin acción posterior | Añadir un estado de seguridad que marque GNSS como no confiable, reduzca velocidad y registre el incidente | Convertir la detección en una respuesta operacional mínima. |

Estas medidas no sustituyen una validación con hardware ni corrigen por completo los modelos simplificados de UWB y del ataque, pero sí aumentan rápidamente la reproducibilidad, la trazabilidad y la calidad de la evaluación.

### 15.2 Impacto de la sincronización aplicada

La sincronización implementada en esta revisión afecta al **registro y a la referencia temporal del análisis**, no a las muestras que consume el CUSUM. El detector sigue ejecutándose con sus mismos timers, callbacks, ventana móvil, `beta`, `tau` y `alert_confirm_time`; no se ha añadido interpolación ni realineamiento de GNSS y UWB dentro de `cusum_detector_node.py`.

Por ello:

- E0-E6 mantienen la misma dinámica física y la misma lógica de detección.
- Los valores del CUSUM que se obtengan en una nueva ejecución deberían ser comparables, aunque una nueva ejecución nunca será numéricamente idéntica por la temporización de Gazebo y DDS.
- Los tiempos reportados pueden cambiar porque ahora se usa el timestamp exacto del evento de activación, en lugar de la primera muestra periódica `True` de `/meaconing/active`.
- E3 ha cambiado de forma importante como métrica: al conservar el pre-roll, el TTD es ahora observable y queda registrado como `4.34 s`.
- Las gráficas nuevas tienen un origen temporal y un periodo de arranque más completos; los rosbags históricos no se han alterado retroactivamente.

Los bags antiguos siguen siendo compatibles: `plot_results.py` y `make_video.py` usan `/meaconing/activation_event` cuando existe y vuelven a `/meaconing/active` para los registros históricos.

---

## 16. Trabajo futuro

### 16.1 Validación estadística

- Repetir cada escenario con al menos 20-30 semillas.
- Calcular media, mediana, desviación y percentiles del TTD.
- Estimar FAR por hora o por número de muestras.
- Construir curvas ROC variando `tau` y `beta`.
- Separar la variación debida al detector de la variación debida a Gazebo.

### 16.2 Mejoras del detector

- Usar estimadores robustos o adaptativos de la línea base.
- Incorporar sincronización temporal aproximada entre GNSS y UWB.
- Modelar explícitamente la distribución de la innovación según la separación.
- Evaluar CUSUM con parámetros normalizados por varianza.
- Añadir un estado de calidad de UWB para no alertar ante sensor degradado.
- Implementar detección multirrobot y votación distribuida.

### 16.3 Ataques y sensores más realistas

- Implementar el modo `pattern` con posiciones falsas no coincidentes.
- Modelar un atacante individualizado por receptor.
- Simular señales GNSS retardadas con dinámicas temporales más físicas.
- Incluir IMU, odometría visual, LiDAR y balizas UWB fijas.
- Simular NLOS, pérdidas de paquetes y errores de ranging.
- Evaluar ataques coordinados a GNSS y UWB.

### 16.4 Respuesta, seguridad operacional y fallback colaborativo

Cuando la alarma se confirma, una versión de producción debería:

1. Marcar GNSS como fuente no confiable.
2. Determinar si el ataque afecta a un solo robot o a toda la pareja.
3. Cambiar a odometría inercial, visual o relativa.
4. Reducir velocidad y mantener una separación segura.
5. Informar a los demás robots.
6. Guardar evidencia forense con timestamps sincronizados.
7. Recuperar la navegación GNSS sólo después de una nueva calibración.

Una mitigación futura especialmente interesante para el caso E5 sería un **fallback de seguimiento colaborativo**. Si el detector confirma que robot 1 está meaconado y robot 2 conserva sensores confiables, robot 1 podría dejar de utilizar su posición GNSS y pasar a seguir al robot 2 usando la estimación relativa disponible. El robot 2 actuaría como líder temporal y robot 1 como seguidor:

```text
GNSS meaconado en R1 -> se descarta tras la alarma
UWB / sensores relativos -> estiman relación R1-R2
R2 no afectado -> proporciona referencia de movimiento
Controlador de R1 -> mantiene distancia, orientación y velocidad seguras
```

La versión mínima consistiría en mantener una distancia objetivo respecto al líder y limitar la velocidad del seguidor. Una versión más completa permitiría reproducir la trayectoria del líder mediante odometría relativa y una estimación de orientación. Esta estrategia sería adecuada cuando sólo un robot está afectado, como en E5, pero no resolvería por sí sola E6 si ambos robots están meaconados: en ese caso se necesitaría una tercera referencia, una baliza UWB fija, odometría visual, IMU o un robot adicional confiable.

También debe aclararse la limitación geométrica: una única medida escalar `D_UWB` informa de la distancia entre los robots, pero no determina el vector 2D que apunta al líder. Para seguirlo espacialmente se necesitaría al menos una de estas fuentes adicionales:

- bearing o pose relativa UWB;
- varias balizas UWB para trilateración;
- odometría visual o LiDAR relativa;
- IMU y odometría local fusionadas;
- intercambio de posición entre robots sólo después de validarla con una fuente independiente.

El fallback no está implementado en la versión actual. Se propone como una extensión de mitigación que reutiliza la arquitectura existente: el topic de alarma activa el cambio de modo y el controlador waypoint sustituye la referencia GNSS por una referencia relativa validada.

### 16.5 Tests y calidad de software

Crear tests unitarios sin necesidad de lanzar Gazebo. Las funciones Pure Pursuit y el CUSUM pueden probarse con secuencias sintéticas conocidas. También conviene añadir una prueba de integración mínima que verifique que todos los topics aparecen y que la alarma tarda al menos `alert_confirm_time` desde el cruce cuando el candidato empieza dentro de la ventana observada.

---

## 17. Propuesta de estructura para el informe Word

Una IA puede convertir este documento en un informe con la siguiente organización:

### Capítulo 1. Introducción

- Dependencia de GNSS en robots autónomos.
- Amenaza de meaconing.
- Motivación de la detección colaborativa.
- Pregunta de investigación y contribuciones.

### Capítulo 2. Estado del arte y fundamentos

- GNSS spoofing y meaconing.
- Autenticación GNSS y sus límites frente a retransmisión.
- Localización relativa UWB.
- Detección secuencial y CUSUM.
- Trabajos relacionados: Bhatti y Humphreys, cooperación GNSS, enjambres UAV y MURP.

### Capítulo 3. Diseño del sistema

- Arquitectura ROS 2.
- Gazebo y TurtleBot3.
- Modelos GNSS y UWB.
- Inyector de ataque.
- Marcos de coordenadas.
- Topics y servicios.

### Capítulo 4. Algoritmo de detección

- Definición de `D_GNSS`, `D_UWB` y `delta_raw`.
- Sesgo de la norma Euclídea.
- Calibración inicial.
- Media móvil.
- CUSUM de dos colas.
- Confirmación temporal.
- Selección de parámetros.

### Capítulo 5. Implementación

- Launch files.
- Nodos Python.
- Controlador Pure Pursuit.
- Runner de experimentos.
- Grabación y análisis MCAP.
- Soluciones específicas de macOS/DDS.

### Capítulo 6. Metodología experimental

- Parámetros comunes.
- Duración y sincronización.
- Definición de TTD y falsas alarmas.
- E0-E6 y sus hipótesis.
- Referencia de E5.

### Capítulo 7. Resultados

- Tabla de métricas.
- E0 como control negativo.
- E1 y E2 como sensibilidad a la velocidad.
- E3 como arranque adversarial con limitación de captura.
- E4 como sensibilidad a geometría.
- E5 reference y justificación de la confirmación.
- E5 como impacto físico.
- E6 como ataque dual.
- Interpretación de cada gráfica.

### Capítulo 8. Discusión

- Qué demuestra el proyecto.
- Por qué E6 no necesariamente es más rápido.
- Relación entre cruce, confirmación y alarma.
- Robustez observada y margen de E0.
- Amenazas a la validez.

### Capítulo 9. Conclusiones y trabajo futuro

- Respuesta a la pregunta de investigación.
- Aportaciones.
- Limitaciones.
- Repetición estadística y validación hardware como siguientes pasos.

### Anexos

- Tabla completa de parámetros.
- Árbol del proyecto.
- Topics ROS 2.
- Fragmentos del algoritmo.
- Comandos de reproducción.
- Capturas y gráficas.

---

## 18. Comandos de reproducción

### Construcción

```bash
cd ~/tfm_meaconing_ws
source /Users/toni/robostack/.pixi/envs/jazzy/setup.bash
colcon build --packages-select collaborative_detection
source install/setup.bash
```

### Ejecutar escenarios

```bash
export TURTLEBOT3_MODEL=waffle

bash src/collaborative_detection/scripts/run_experiment.sh e0
bash src/collaborative_detection/scripts/run_experiment.sh e1
bash src/collaborative_detection/scripts/run_experiment.sh e2
bash src/collaborative_detection/scripts/run_experiment.sh e3
bash src/collaborative_detection/scripts/run_experiment.sh e4
bash src/collaborative_detection/scripts/run_experiment.sh e5_ref
bash src/collaborative_detection/scripts/run_experiment.sh e5
bash src/collaborative_detection/scripts/run_experiment.sh e6
```

Para observar Gazebo:

```bash
bash src/collaborative_detection/scripts/run_experiment.sh e6 --gui
```

### Generar gráficas

```bash
source /Users/toni/robostack/.pixi/envs/jazzy/setup.bash
source install/setup.bash
python3 src/collaborative_detection/analysis/plot_results.py
```

### Generar vídeos E5/E6

```bash
python3 src/collaborative_detection/analysis/make_video.py e5_waypoint_attack
python3 src/collaborative_detection/analysis/make_video.py e6_dual_meaconing
```

Los scripts de análisis requieren `rosbag2_py`, `rclpy`, `numpy` y `matplotlib` del entorno Jazzy. Los resultados se guardan en `results/plots/` y `results/videos/`.

---

## 19. Inventario de ficheros relevantes

| Fichero | Función |
|---|---|
| `src/collaborative_detection/config/params.yaml` | Parámetros comunes y de los escenarios. |
| `src/collaborative_detection/launch/two_robots.launch.py` | Gazebo, robots, bridges y topics por modelo. |
| `src/collaborative_detection/launch/experiment.launch.py` | Orquestación de toda la pila. |
| `src/collaborative_detection/collaborative_detection/nodes/gnss_sim_node.py` | GNSS con ruido y conversión a world frame. |
| `src/collaborative_detection/collaborative_detection/nodes/uwb_sim_node.py` | Distancia UWB física con ruido. |
| `src/collaborative_detection/collaborative_detection/nodes/meaconing_injector.py` | Drag-off hacia un objetivo falso común. |
| `src/collaborative_detection/collaborative_detection/nodes/cusum_detector_node.py` | Calibración, filtro, CUSUM y alarma. |
| `src/collaborative_detection/collaborative_detection/nodes/robot_mover_node.py` | Movimiento circular de E0-E4. |
| `src/collaborative_detection/collaborative_detection/nodes/waypoint_follower_node.py` | Pure Pursuit y rutas de E5/E6. |
| `src/collaborative_detection/scripts/run_experiment.sh` | Automatización, parametrización y rosbag. |
| `src/collaborative_detection/analysis/plot_results.py` | Métricas y gráficas de todos los rosbags. |
| `src/collaborative_detection/analysis/make_video.py` | Vídeos sincronizados de E5 y E6. |
| `src/collaborative_detection/config/fastdds_udp_only.xml` | Transporte UDP-only para estabilidad en macOS. |
| `README.md` | Guía principal del proyecto. |
| `EXPERIMENTS.md` | Guía de ejecución y parámetros. |
| `plan.md` | Plan y contexto original del TFM; contiene partes históricas que deben contrastarse con el código final. |
| `results/*_params.yaml` | Snapshot de parámetros efectivos por experimento. |
| `results/*/*.mcap` | Datos registrados de cada experimento. |
| `results/plots/*.png` | Gráficas generadas a partir de los rosbags. |

---

## 20. Glosario

- **GNSS:** sistema global de navegación por satélite.
- **Meaconing:** recepción, retardo y retransmisión de una señal legítima para desplazar la solución del receptor.
- **Spoofing:** término general para la falsificación de señales o datos; en este proyecto aparece también en nombres históricos de topics.
- **UWB:** Ultra-Wideband; tecnología utilizada aquí para ranging relativo.
- **SITL:** Software-in-the-loop; simulación en la que el software de control se ejecuta contra un entorno virtual.
- **CUSUM:** Cumulative Sum; detector secuencial que acumula evidencia de un cambio persistente.
- **TTD:** Time-To-Detect, tiempo entre la activación observada y la primera alarma confirmada.
- **FAR:** False Alarm Rate, frecuencia de falsas alarmas bajo operación normal.
- **GNSS range bias:** sesgo producido al transformar posiciones ruidosas mediante una norma Euclídea.
- **Drag-off:** desplazamiento gradual de la solución de navegación hacia una posición falsa.
- **Pure Pursuit:** controlador geométrico que sigue una ruta buscando un punto a distancia de lookahead.
- **MCAP:** formato de almacenamiento utilizado por rosbag2 en las ejecuciones registradas.

---

## 21. Conclusión técnica

El proyecto demuestra una cadena completa de seguridad para robots cooperativos: simulación física, generación de sensores, inyección de meaconing, control basado en GNSS, detección estadística, grabación y análisis offline. El resultado más sólido no es simplemente que el CUSUM llegue a valores altos durante un ataque, sino que el sistema separa tres situaciones distintas:

1. **Ruido normal sin ataque:** E0 permanece sin alarma.
2. **Navegación por waypoints sin ataque:** E5 reference permanece sin alarma.
3. **Ataques persistentes:** E1, E2, E3, E4, E5 y E6 generan alarmas confirmadas.
4. **Impacto físico:** E5 y E6 muestran que la deriva puede crecer varios metros si el ataque continúa, aunque la alarma aparece cuando la desviación todavía es pequeña.

La calibración de la línea base fue necesaria para eliminar el sesgo normal de la distancia GNSS. El diseño de dos colas conserva capacidad para detectar tanto el colapso como la inflación de la distancia GNSS. La navegación por waypoints añade valor al experimento porque permite cuantificar el daño físico, mientras que la campaña actual demuestra que el pre-roll y la marca de activación permiten medir también el caso hot start.

La conclusión defendible para el informe es:

> En el entorno SITL desarrollado, la comparación colaborativa entre GNSS y UWB, combinada con un CUSUM de dos colas calibrado y una ventana de confirmación temporal, detecta el modelo de meaconing de drag-off en los seis escenarios atacados sin generar falsas alarmas en los dos escenarios de control. En E5 y E6 la alarma se confirma cuando la deriva física todavía es reducida, antes de que el error de navegación alcance varios metros. Además, el mecanismo dificulta los ataques contra flotas mayores porque el atacante tendría que mantener una estructura geométrica falsa coherente con un número creciente de relaciones UWB. La validación futura debe incluir repetición estadística, hardware real, degradaciones UWB, ataques coordinados y mecanismos de mitigación posteriores a la alarma, entre ellos un modo de seguimiento relativo de un robot no afectado cuando sólo uno de los robots esté comprometido.
