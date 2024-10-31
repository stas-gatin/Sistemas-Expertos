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

### Índice

1. [Parámetros de Inicialización](#parámetros-de-inicialización)
   - [Indicadores y Estados de Trayectoria](#indicadores-y-estados-de-trayectoria)
   - [Parámetros de Velocidad y Movimiento](#parámetros-de-velocidad-y-movimiento)
   - [Parámetros de la Trayectoria Lineal](#parámetros-de-la-trayectoria-lineal)
   
2. [Metodos Principales](#funciones-principales)
   - [cubic_bezier](#cubic_bezier)
   - [generate_trajectory](#generate_trajectory)
   - [generate_linear_path](#generate_linear_path)
   - [calcular_offset](#calcular_offset)
   - [calculate_control_points](#calculate_control_points)
   - [normalize_angle](#normalize_angle)
   - [calcular_angulo](#calcular_angulo)
   - [decidir_modo_movimiento](#decidir_modo_movimiento)
   - [verificar_proximidad_objetivo](#verificar_proximidad_objetivo)
   - [obtener_coordenadas_objetivo](#obtener_coordenadas_objetivo)
   - [calcular_distancia_objetivo](#calcular_distancia_objetivo)
   - [calcular_velocidad_lineal](#calcular_velocidad_lineal)
   - [calcular_velocidad_angular](#calcular_velocidad_angular)
   - [tomar_decision](#tomar_decision)
   - [es_objetivo_alcanzado](#es_objetivo_alcanzado)
   - [hay_parte_optativa](#hay_parte_optativa)

---

### Parámetros de Inicialización

#### Indicadores y Estados de Trayectoria
- `objetivoAlcanzado` (bool): Indicador de si el robot ha alcanzado su objetivo final.
- `segmentoObjetivo` (object): Segmento de destino actual en el trayecto.
- `VOLVER_AL_INICIO` (bool): Controla si el robot debe regresar al punto inicial del trayecto.
- `GO_AROUND_TRIANGLE` (bool): Indica si el robot debe rodear un obstáculo triangular.
- `FRENAR` (bool): Indica si el robot debe frenar.

#### Parámetros de Velocidad y Movimiento
- `velocidad` (float): Velocidad lineal inicial (en m/s).
- `velocidad_angular` (float): Velocidad angular inicial (en rad/s).
- `reverse` (bool): Modo de reversa inicial.

#### Parámetros de la Trayectoria Lineal
- `check_point_segmento` (int): Estado del trayecto (punto inicial (False) o final (True)).
- `LINE_CHECKPOINTS` (int): Cantidad de puntos de control en trayecto lineal.
- `line_trayectory` (list): Lista de coordenadas del trayecto lineal.
- `start_point` (tuple): Punto de inicio del trayecto.
- `segment_number` (int): Número del segmento actual.
- `distance` (int): Distancia hasta el punto objetivo.
- `STOP_DISTANCE` (float): Distancia para detener el robot al final del segmento.
- `CHECKPOINT_DISTANCE_ACTIVATOR` (float): Distancia que activa el cambio de punto de control.
- `CONSTANTE_AUMENTAR_VELOCIDAD` (float): Constante para aumentar velocidad en trayecto.
- `FIRST_SEGMENT_INDEX` (int): Índice del primer segmento del trayecto.
- `TOTAL_SEGMENT_NUMBER` (int): Número total de segmentos en el trayecto.

#### Parámetros de la Trayectoria Triangular
- `check_point_triangulo` (int): Contador de puntos de control en el trayecto triangular.
- `TRIANGLE_CHECKPOINTS` (int): Cantidad de puntos de control en trayecto triangular.
- `triangle_trayectory` (list): Lista de coordenadas del trayecto triangular.
- `CONTROL_POINT_CONSTANT` (float): Constante de ajuste de puntos de control en trayecto triangular.
- `TRIANGLE_SPEED` (float): Velocidad para movimiento triangular.
- `MINIMUM_DISTANCE_TRIANGLE_CP` (int): Distancia mínima para activar puntos de control.

#### Ángulos y Control de Giro
- `turn_angle_rad` (float): Ángulo de giro en radianes.
- `turn_angle_deg` (float): Ángulo de giro en grados.
- `REVERSE_THRESHOLD` (int): Umbral en grados para activar marcha atrás.
- `MAXIMUM_ANGLE_DEG` (int): Ángulo máximo antes de desviarse del trayecto.
- `DISTANCE_TURN_CONSTANT` (float): Constante para ajustar el ángulo máximo según la distancia.
- `VELOCIDAD_ANGULAR_CONSTANT` (int): Constante para ajustar velocidad angular en giros.

---

### Funciones Principales

#### `cubic_bezier`
Calcula la posición en una curva cúbica de Bézier a partir de un valor `t` y cuatro puntos de control.

- **Parámetros**:
  - `t` (float): Valor en el rango [0, 1] para indicar la progresión a lo largo de la curva.
  - `P0`, `P1`, `P2`, `P3` (`numpy.array`): Puntos de control de la curva.
- **Retorno**:
  - `numpy.array`: Coordenadas del punto en la curva correspondiente al valor `t`.

#### `generate_trajectory`
Genera una trayectoria curva usando dos curvas de Bézier entre puntos de referencia dados.

- **Parámetros**:
  - `B`, `C`, `D` (`numpy.array`): Puntos clave de la trayectoria.
  - `CP1`, `CP2` (`numpy.array`): Puntos de control para las curvas.
- **Retorno**:
  - `numpy.array`: Conjunto de puntos de la trayectoria generada.

#### `generate_linear_path`
Genera una trayectoria lineal entre dos puntos `A` y `B`.

- **Parámetros**:
  - `A`, `B` (tupla o `numpy.array`): Coordenadas de los puntos de inicio y fin.
- **Retorno**:
  - Lista de puntos intermedios generados entre `A` y `B`.

#### `calcular_offset`
Calcula un valor de "offset" para ajustar la curva basada en la distancia entre puntos de una trayectoria triangular.

- **Parámetros**:
  - `A`, `B`, `C` (tuplas): Coordenadas de los vértices del triángulo.
- **Retorno**:
  - `float`: Offset ajustado en un rango entre 0.5 y 1.5.

#### `calculate_control_points`
Calcula los puntos de control que generan curvas suaves en la trayectoria.

- **Parámetros**:
  - `B`, `C`, `D` (`numpy.array`): Puntos clave de la trayectoria.
- **Retorno**:
  - `tuple`: Puntos de control `CP1` y `CP2`.

#### `normalize_angle`
Normaliza un ángulo para que esté dentro del rango [-180, 180).

- **Parámetro**:
  - `angle` (float): Ángulo a normalizar.
- **Retorno**:
  - Ángulo normalizado (float).

#### `calcular_angulo`
Calcula el ángulo entre la orientación actual y el objetivo.

- **Parámetros**:
  - `x_target`, `y_target` (float): Coordenadas del objetivo.
  - `x_robot`, `y_robot` (float): Coordenadas actuales del robot.
  - `current_angle` (float): Ángulo de orientación del robot.
- **Retorno**:
  - `tuple`: Ángulo hacia el objetivo y diferencia con el ángulo actual.

#### `decidir_modo_movimiento`
Decide si el robot debe moverse en reversa según la diferencia angular.

- **Parámetro**:
  - `turn_angle_deg` (float): Diferencia angular.
- **Retorno**:
  - None.

#### `verificar_proximidad_objetivo`
Determina si el robot ha alcanzado su objetivo y ajusta la velocidad.

- **Parámetro**:
  - `distance` (float): Distancia actual al objetivo.
- **Retorno**:
  - None.

#### `obtener_coordenadas_objetivo`
Obtiene las coordenadas del próximo objetivo del robot en la trayectoria.

- **Retorno**:
  - `tuple`: Coordenadas `(x_target, y_target)` del objetivo.

#### `calcular_distancia_objetivo`
Calcula la distancia euclidiana entre el robot y el objetivo.

**Parámetros:**
- `x_target` (float): Coordenada X del objetivo.
- `y_target` (float): Coordenada Y del objetivo.
- `x_robot` (float): Coordenada X actual del robot.
- `y_robot` (float): Coordenada Y actual del robot.

**Retorna:**
- `float`: Distancia euclidiana entre el robot y el objetivo.


#### `calcular_velocidad_lineal`
Calcula y actualiza la velocidad lineal del robot en función del ángulo de giro y la distancia al objetivo.

**Parámetros:**
- `turn_angle_rad` (float): Ángulo de giro en radianes.
- `distance` (float): Distancia euclidiana entre el robot y el objetivo.


#### `calcular_velocidad_angular`
Calcula y actualiza la velocidad angular del robot en función del ángulo de giro.

**Parámetros:**
- `turn_angle_rad` (float): Ángulo de giro en radianes.


#### `tomar_decision`
Toma una decisión de movimiento para el robot basado en su posición actual y la posición del objetivo en el segmento.

Este método calcula las velocidades lineal y angular necesarias para mover al robot hacia el objetivo de manera eficiente, ajustando su trayectoria en función del ángulo de giro y la distancia al objetivo. También considera la posibilidad de moverse en reversa para optimizar el movimiento en casos donde el ángulo de giro es mayor al umbral definido.

**Parámetros:**
- `poseRobot` (tuple): Una tupla que contiene la posición actual del robot y su ángulo de orientación en grados, en el formato `(x_robot, y_robot, current_angle)`.

**Retorna:**
- `tuple`: Una tupla con las velocidades lineal y angular calculadas `(velocidad, velocidad_angular)`, donde:
  - `velocidad` (float): Velocidad lineal en m/s.
  - `velocidad_angular` (float): Velocidad angular en rad/s.


#### `es_objetivo_alcanzado`
Devuelve `True` cuando el punto final del objetivo ha sido alcanzado. Es responsabilidad de la alumna o alumno cambiar el valor de la variable `objetivoAlcanzado` cuando se detecte que el robot ha llegado a su objetivo. Esto se llevará a cabo en el método `tomarDecision`. Este método NO debería ser modificado.

**Retorna:**
- `bool`: `True` si el objetivo ha sido alcanzado, de lo contrario `False`.


#### `hay_parte_optativa`
Método para verificar si hay una parte optativa en el proceso. Este método debe ser definido en base a la lógica del robot y su implementación.


## Uso y Dependencias

Este código depende de las librerías `numpy` y `math`. Para instalarlas:

```bash
pip install numpy
```
