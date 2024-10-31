# Proyecto de Sistemas Expertos para Control de Robots

Este proyecto implementa un sistema experto en Python para controlar el movimiento de un robot móvil en un entorno simulado sin obstáculos. El objetivo es que el robot recorra un segmento de línea en el plano con precisión y eficiencia. Para lograr esto, el proyecto está estructurado en varias clases que gestionan diferentes aspectos de la simulación y el control del robot.

# Sistema Experto 1
## Descripción de las Clases

### 1. `P1Launcher.py`
Esta clase inicializa y gestiona el entorno gráfico, asegurando la comunicación entre todos los objetos que intervienen en la simulación. Además, incluye los métodos necesarios para dibujar los elementos en la pantalla. 

### 2. `Robot.py`
Esta clase representa al robot móvil, almacenando su pose actual (posición y orientación en el plano) y procesando los comandos de velocidad lineal y angular que recibe del sistema experto. Internamente, esta clase se encarga de actualizar la pose del robot de acuerdo a su dinámica de movimiento, pero no se interactúa directamente con ella. Los datos de la pose del robot se utilizan como entrada en el sistema experto.

### 3. `Objetivo.py`
Esta clase define un segmento lineal que representa el camino que el robot debe seguir en el plano. También permite configurar un triángulo de restricción: un área triangular definida por tres vértices donde el robot no debe ingresar. Esta configuración es especialmente útil para ejercicios avanzados en los que el robot debe evitar ciertas áreas mientras sigue el segmento objetivo.

### 4. `expertSystem.py`
Esta clase es el núcleo del proyecto, en la que se implementa el sistema experto. En ella se desarrollan los métodos para calcular, en cada instante, los comandos de velocidad lineal y angular que el robot debe seguir para alcanzar el objetivo. Estos comandos se generan en función de la pose del robot y del segmento objetivo configurado.

---

## Clase: ExpertSystem

### Parámetros de Inicialización

#### Indicadores y estados de trayectoria
- `objetivoAlcanzado` (bool): Indicador de que se ha alcanzado el objetivo final
- `segmentoObjetivo` (object): Segmento de la trayectoria actual
- `VOLVER_AL_INICIO` (bool): Control para el regreso del robot al punto inicial
- `GO_AROUND_TRIANGLE` (bool): Indicación para rodear un obstáculo triangular
- `FRENAR` (bool): Indicación para frenar

#### Parámetros de velocidad y movimiento
- `velocidad` (float): Velocidad lineal (m/s)
- `velocidad_angular` (float): Velocidad angular (rad/s)
- `reverse` (bool): Modo de movimiento en reversa

#### Parámetros de la trayectoria lineal
- `check_point_segmento` (int): Estado de la trayectoria (punto inicial/final)
- `LINE_CHECKPOINTS` (int): Número de puntos de control en la trayectoria lineal
- `line_trayectory` (list): Coordenadas de la trayectoria lineal
- `start_point` (tuple): Punto de inicio de la trayectoria
- `segment_number` (int): Número de segmento actual
- `STOP_DISTANCE` (float): Distancia de parada
- `CHECKPOINT_DISTANCE_ACTIVATOR` (float): Distancia de activación del cambio de punto de control


- **Parámetros**:
  - `poseRobot`: Pose actual del robot (coordenadas y ángulo)
- **Retorna**: Velocidades lineal y angular


## Índice
1. [Funciones principales](#funciones-principales)
   - [cubic_bezier](#cubic_bezier)
   - [generate_trajectory](#generate_trajectory)
   - [generate_linear_path](#generate_linear_path)
   - [calcular_offset](#calcular_offset)
   - [calculate_control_points](#calculate_control_points)
2. [Funciones auxiliares](#funciones-auxiliares)
   - [normalize_angle](#normalize_angle)
   - [calcular_angulo](#calcular_angulo)
   - [decidir_modo_movimiento](#decidir_modo_movimiento)
   - [verificar_proximidad_objetivo](#verificar_proximidad_objetivo)
   - [obtener_coordenadas_objetivo](#obtener_coordenadas_objetivo)

---

## Funciones principales

### `cubic_bezier`
Calcula la posición en una curva cúbica de Bézier a partir de un valor `t` y cuatro puntos de control.

- **Parámetros**:
  - `t` (float): Valor en el rango [0, 1] para indicar la progresión a lo largo de la curva.
  - `P0`, `P1`, `P2`, `P3` (`numpy.array`): Puntos de control de la curva.
- **Retorno**:
  - `numpy.array`: Coordenadas del punto en la curva correspondiente al valor `t`.

### `generate_trajectory`
Genera una trayectoria curva usando dos curvas de Bézier entre puntos de referencia dados.

- **Parámetros**:
  - `B`, `C`, `D` (`numpy.array`): Puntos clave de la trayectoria.
  - `CP1`, `CP2` (`numpy.array`): Puntos de control para las curvas.
- **Retorno**:
  - `numpy.array`: Conjunto de puntos de la trayectoria generada.

### `generate_linear_path`
Genera una trayectoria lineal entre dos puntos `A` y `B`.

- **Parámetros**:
  - `A`, `B` (tupla o `numpy.array`): Coordenadas de los puntos de inicio y fin.
- **Retorno**:
  - Lista de puntos intermedios generados entre `A` y `B`.

### `calcular_offset`
Calcula un valor de "offset" para ajustar la curva basada en la distancia entre puntos de una trayectoria triangular.

- **Parámetros**:
  - `A`, `B`, `C` (tuplas): Coordenadas de los vértices del triángulo.
- **Retorno**:
  - `float`: Offset ajustado en un rango entre 0.5 y 1.5.

### `calculate_control_points`
Calcula los puntos de control que generan curvas suaves en la trayectoria.

- **Parámetros**:
  - `B`, `C`, `D` (`numpy.array`): Puntos clave de la trayectoria.
- **Retorno**:
  - `tuple`: Puntos de control `CP1` y `CP2`.

---

## Funciones auxiliares

### `normalize_angle`
Normaliza un ángulo para que esté dentro del rango [-180, 180).

- **Parámetro**:
  - `angle` (float): Ángulo a normalizar.
- **Retorno**:
  - Ángulo normalizado (float).

### `calcular_angulo`
Calcula el ángulo entre la orientación actual y el objetivo.

- **Parámetros**:
  - `x_target`, `y_target` (float): Coordenadas del objetivo.
  - `x_robot`, `y_robot` (float): Coordenadas actuales del robot.
  - `current_angle` (float): Ángulo de orientación del robot.
- **Retorno**:
  - `tuple`: Ángulo hacia el objetivo y diferencia con el ángulo actual.

### `decidir_modo_movimiento`
Decide si el robot debe moverse en reversa según la diferencia angular.

- **Parámetro**:
  - `turn_angle_deg` (float): Diferencia angular.
- **Retorno**:
  - None.

### `verificar_proximidad_objetivo`
Determina si el robot ha alcanzado su objetivo y ajusta la velocidad.

- **Parámetro**:
  - `distance` (float): Distancia actual al objetivo.
- **Retorno**:
  - None.

### `obtener_coordenadas_objetivo`
Obtiene las coordenadas del próximo objetivo del robot en la trayectoria.

- **Retorno**:
  - `tuple`: Coordenadas `(x_target, y_target)` del objetivo.

---

## Uso y Dependencias

Este código depende de las librerías `numpy` y `math`. Para instalarlas:

```bash
pip install numpy
