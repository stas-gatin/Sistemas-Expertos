import numpy as np
import math
from fuzzy_expert.variable import FuzzyVariable
from fuzzy_expert.rule import FuzzyRule
from fuzzy_expert.inference import DecompositionalInference
from robot import WACC, WMAX, VACC, VMAX
import matplotlib.pyplot as plt
from segmento import *

class FuzzySystem:
    def __init__(self) -> None:
        from P1Launcher import objectiveSet  # Importación de los objetivos del trayecto
        
        # --- Estados generales ---
        self.objetivoAlcanzado: bool = False          # Indica si el robot alcanzó su objetivo
        self.segmentoObjetivo: object = None          # Segmento objetivo actual
        self.VOLVER_AL_INICIO: bool = True            # Indica si el robot debe regresar al inicio
        self.FRENAR: bool = None                      # Indica si el robot debe frenar

        # --- Velocidades y movimiento ---
        self.velocidad: float = 0                     # Velocidad lineal inicial (m/s)
        self.velocidad_angular: float = 0             # Velocidad angular inicial (rad/s)
        self.reverse: bool = False                    # Indica si el robot está en modo reversa
        self.distance: float = 0.0                    # Distancia al objetivo actual

        # --- Parámetros de trayectoria lineal ---
        self.check_point_segmento: int = 0            # Índice del punto de control actual
        self.LINE_CHECKPOINTS: int = 20               # Total de puntos de control en trayectoria lineal
        self.line_trayectory = None                   # Coordenadas de la trayectoria lineal
        self.start_point: tuple = None                # Punto de inicio de la trayectoria
        self.segment_number: int = 0                  # Número del segmento actual
        self.STOP_DISTANCE: float = 0.5               # Distancia para detenerse al final del segmento
        self.CHECKPOINT_DISTANCE_ACTIVATOR: float = 4.25 # Distancia que activa cambio de punto de control
        self.CONSTANTE_AUMENTAR_VELOCIDAD: float = 1.5 # Constante para aumentar velocidad
        self.FIRST_SEGMENT_INDEX: int = 0             # Índice del primer segmento
        self.TOTAL_SEGMENT_NUMBER: int = len(objectiveSet) # Número total de segmentos

        # --- Parámetros de trayectoria triangular ---
        self.check_point_triangulo: int = 0           # Índice del punto de control actual en trayectoria triangular
        self.TRIANGLE_CHECKPOINTS: int = 5            # Total de puntos de control en trayectoria triangular
        self.triangle_trayectory = None               # Coordenadas de la trayectoria triangular
        self.CONTROL_POINT_CONSTANT: float = 0.7      # Constante para ajustar los puntos de control
        self.MINIMUM_DISTANCE_TRIANGLE_CP: int = 4    # Distancia mínima para activar puntos de control


        self.variables = {
            "distance": FuzzyVariable(
                universe_range=(0, 120),
                terms={
                    "super_close": [(0, 1), (0.6, 0)],
                    "close": [(0.5, 0), (1, 1), (2, 0)],
                    "far": [(1.5, 0), (8, 1), (100, 1), (120, 1)],
                }
            ),
            
            "angle": FuzzyVariable(
                universe_range=(0, 180),
                terms={
                    "super_small": [(0, 1), (0.25, 1), (1, 0)],
                    "small": [(0, 0), (0.5, 1), (5, 1), (10, 0)],
                    "medium": [(5, 0), (20, 1), (45, 1), (60, 0)],
                    "large": [(50, 0), (170, 1), (180, 1)]
                }
            ),

            "linear_velocity": FuzzyVariable(
                universe_range=(0, 3),
                terms={
                    "super_slow": [(0, 1), (0.01, 0)],
                    "slow": [(0.1, 1), (2.5, 0)],
                    "super_fast": [(2.99, 0), (3, 1)]
                }
            ),
            "angular_velocity": FuzzyVariable(
            universe_range=(0, 1),
            terms={
                "super_slow": [(0, 1), (0.01, 0)],
                "slow": [(0.005, 0), (0.1, 1), (0.2, 0)],
                "medium": [(0.3, 0), (0.6, 1), (0.61, 0)],
                "fast": [(0.99, 0), (1, 1)]
            }
        )             
}
        self.rules = [
        
            # --- SUPER CLOSE ---
        FuzzyRule(
            premise=[
                ("distance", "super_close"),
                ("AND", "angle", "super_small")
            ],
            consequence=[
                ("linear_velocity", "super_slow"),
                ("angular_velocity", "super_slow")
            ]
        ),
        FuzzyRule(
            premise=[
                ("distance", "super_close"),
                ("AND", "angle", "small")
            ],
            consequence=[
                ("linear_velocity", "super_slow"),
                ("angular_velocity", "slow")
            ]
        ),
        FuzzyRule(
            premise=[
                ("distance", "super_close"),
                ("AND", "angle", "medium")
            ],
            consequence=[
                ("linear_velocity", "super_slow"),
                ("angular_velocity", "medium")
            ]
        ),
        FuzzyRule(
            premise=[
                ("distance", "super_close"),
                ("AND", "angle", "large")
            ],
            consequence=[
                ("linear_velocity", "super_slow"),
                ("angular_velocity", "fast")
            ]
        ),

        # ---CLOSE---        
        FuzzyRule(
            premise=[
                ("distance", "close"),
                ("AND", "angle", "super_small")
            ],
            consequence=[
                ("linear_velocity", "slow"),
                ("angular_velocity", "super_slow")
            ]
        ),
        FuzzyRule(
            premise=[
                ("distance", "close"),
                ("AND", "angle", "small")
            ],
            consequence=[
                ("linear_velocity", "slow"),
                ("angular_velocity", "slow")
            ]
        ),
        FuzzyRule(
            premise=[
                ("distance", "close"),
                ("AND", "angle", "medium")
            ],
            consequence=[
                ("linear_velocity", "slow"),
                ("angular_velocity", "medium")
            ]
        ),
        FuzzyRule(
            premise=[
                ("distance", "close"),
                ("AND", "angle", "large")
            ],
            consequence=[
                ("linear_velocity", "slow"),
                ("angular_velocity", "fast")
            ]
        ),

        # --- FAR ---
        FuzzyRule(
            premise=[
                ("distance", "far"),
                ("AND", "angle", "super_small")
            ],
            consequence=[
                ("linear_velocity", "super_fast"),
                ("angular_velocity", "super_slow")
            ]
        ),
        FuzzyRule(
            premise=[
                ("distance", "far"),
                ("AND", "angle", "small")
            ],
            consequence=[
                ("linear_velocity", "super_fast"),
                ("angular_velocity", "slow")
            ]
        ),
        FuzzyRule(
            premise=[
                ("distance", "far"),
                ("AND", "angle", "medium")
            ],
            consequence=[
                ("linear_velocity", "super_fast"),
                ("angular_velocity", "medium")
            ]
        ),
        FuzzyRule(
            premise=[
                ("distance", "far"),
                ("AND", "angle", "large")
            ],
            consequence=[
                ("linear_velocity", "super_fast"),
                ("angular_velocity", "fast")
            ]
        ),
    ]
        self.inference_system = DecompositionalInference(
            and_operator="min",
            or_operator="max",
            implication_operator="Rc",
            composition_operator="max-min",
            production_link="max",
            defuzzification_operator="cog",
        )

   # #######################
    # ---- LINE CONTROLL ----
    # #######################
    
    def generate_linear_path(self, A, B):
        """
        Genera una trayectoria lineal entre dos puntos, dividiendo el segmento en puntos equidistantes.

        Args:
            A (tuple): Coordenadas del punto inicial (x1, y1).
            B (tuple): Coordenadas del punto final (x2, y2).

        Returns:
            list: Lista de puntos (x, y) que forman la trayectoria lineal desde A hasta B.

        Detalles:
        1. Calcula las coordenadas de los puntos intermedios dividiendo el segmento entre A y B en 
        `LINE_CHECKPOINTS + 1` partes.
        2. Añade el punto inicial (A) y el punto final (B) a la lista de resultados.
        3. Redondea las coordenadas de los puntos intermedios a 6 decimales para mejorar la precisión.
        """

        # Coordenadas del punto A
        x1, y1 = float(A[0]), float(A[1])
        
        # Coordenadas del punto B
        x2, y2 = float(B[0]), float(B[1])
        
        # Crear una lista vacía y agregar el punto A al inicio
        points = [A]
        
        # Calcular el paso entre puntos en los ejes x e y
        dx = (x2 - x1) / (self.LINE_CHECKPOINTS + 1)
        dy = (y2 - y1) / (self.LINE_CHECKPOINTS + 1)
        
        # Agregar puntos intermedios con precisión de float
        for i in range(1, self.LINE_CHECKPOINTS + 1):
            x = round(x1 + i * dx, 6)
            y = round(y1 + i * dy, 6)
            points.append((x, y))
        
        # Agregar el punto B al final de la lista
        points.append(B)
        
        return points
    
    # #######################
    # ---- CURVE CONTROLL ----
    # #######################
    
    def cubic_bezier(self, t, P0, P1, P2, P3):
        """
        Calcula un punto en una curva de Bézier cúbica para un valor dado de `t`.

        Args:
            t (float): Parámetro de la curva, en el rango [0, 1], donde:
                - 0 corresponde al inicio de la curva.
                - 1 corresponde al final de la curva.
            P0 (float): Coordenada inicial de la curva.
            P1 (float): Primer punto de control.
            P2 (float): Segundo punto de control.
            P3 (float): Coordenada final de la curva.

        Returns:
            float: Coordenada calculada en la curva de Bézier cúbica para el valor de `t`.

        Detalles:
        - Utiliza la fórmula estándar para una curva de Bézier cúbica:
        `(1 - t)^3 * P0 + 3 * (1 - t)^2 * t * P1 + 3 * (1 - t) * t^2 * P2 + t^3 * P3`.
        - La curva es definida por los puntos de control (P0, P1, P2, P3), proporcionando una interpolación suave entre P0 y P3.
        """
        return (1 - t)**3 * P0 + 3 * (1 - t)**2 * t * P1 + 3 * (1 - t) * t**2 * P2 + t**3 * P3
    
    def calcular_offset(self, A, B, C):
        """
        Calcula el offset de un punto respecto a una línea definida por dos puntos.

        Args:
            A (tuple): Coordenadas del punto A, que define el inicio de la línea (x1, y1).
            B (tuple): Coordenadas del punto B, que define el final de la línea (x2, y2).
            C (tuple): Coordenadas del punto C, desde donde se calcula el offset respecto a la línea AB.

        Returns:
            float: Offset calculado, limitado entre 0.5 y 5.

        Detalles:
        1. Calcula la longitud de la base (distancia entre A y B) usando la distancia euclidiana.
        2. Calcula la altura perpendicular desde el punto C hasta la línea AB mediante la fórmula de la distancia punto-línea.
        3. Normaliza el offset utilizando una fórmula específica: `1.5 + (0.5 * (altura / (longitud_base + 1)))`.
        4. Asegura que el offset esté dentro del rango especificado [0.5, 5].
        """

        # Calcula la longitud de la base AB
        longitud_base = math.dist(A, B)
        
        # Calcula la altura desde el punto C hasta la línea AB
        altura = abs((B[1] - A[1]) * C[0] - (B[0] - A[0]) * C[1] + B[0] * A[1] - B[1] * A[0]) / longitud_base
        
        # Calcula el offset, normalizando el valor
        offset = 1.5 + (0.5 * (altura / (longitud_base + 1)))
        
        # Limita el offset entre 0.5 y 1.5
        offset = max(0.5, min(offset, 5))

        print(offset)
        
        return offset

    def calculate_control_points(self, B, C, D):
        """
        Calcula los puntos de control para una curva basada en tres puntos dados.

        Args:
            B (tuple): Coordenadas del primer punto de la curva (x1, y1).
            C (tuple): Coordenadas del segundo punto de la curva (x2, y2).
            D (tuple): Coordenadas del tercer punto de la curva (x3, y3).

        Returns:
            tuple: 
                - CP1 (numpy.ndarray): Primer punto de control calculado.
                - CP2 (numpy.ndarray): Segundo punto de control calculado.

        Detalles:
        1. Calcula los vectores de dirección entre los puntos B → C y C → D.
        2. Normaliza estos vectores para obtener direcciones perpendiculares.
        3. Calcula un offset dinámico utilizando el método `calcular_offset`, que ajusta el desplazamiento de los puntos de control.
        4. Determina las posiciones de CP1 y CP2 sumando los desplazamientos proporcionales al vector de dirección y al offset.
        """

        # Vectores de dirección de B a C y de C a D
        vBC = np.array(C) - np.array(B)
        vCD = np.array(D) - np.array(C)

        # Normalización para obtener la dirección perpendicular
        norm_vBC = np.array([-vBC[1], vBC[0]]) / np.linalg.norm(vBC)
        norm_vCD = np.array([-vCD[1], vCD[0]]) / np.linalg.norm(vCD)
        
        offset = self.calcular_offset(B,D,C)

        # Desplazamiento de los puntos de control según el offset
        CP1 = np.array(B) + self.CONTROL_POINT_CONSTANT * vBC + offset * norm_vBC  # Desplazamiento de CP1
        CP2 = np.array(C) + self.CONTROL_POINT_CONSTANT * vCD + offset * norm_vCD  # Desplazamiento de CP2

        return CP1, CP2
    
    def generate_curved_path(self, B, C, D, CP1, CP2):
        """
        Genera una trayectoria curva utilizando dos secciones de curvas Bézier cúbicas.

        Args:
            B (tuple): Coordenadas del primer punto de la trayectoria (x1, y1).
            C (tuple): Coordenadas del punto intermedio de la trayectoria (x2, y2).
            D (tuple): Coordenadas del punto final de la trayectoria (x3, y3).
            CP1 (numpy.ndarray): Primer punto de control para la curva de B a C.
            CP2 (numpy.ndarray): Segundo punto de control para la curva de C a D.

        Returns:
            numpy.ndarray: Array de puntos que forman la trayectoria curva completa.

        Detalles:
        1. Divide la trayectoria en dos partes:
            - Primera curva Bézier cúbica desde B a C, usando `CP1` como punto de control.
            - Segunda curva Bézier cúbica desde C a D, usando `CP2` como punto de control.
        2. Calcula los puntos de la curva para cada sección utilizando el método `cubic_bezier`.
        3. Genera un conjunto de puntos equidistantes a lo largo de cada sección con `TRIANGLE_CHECKPOINTS`.
        4. Combina ambas secciones para formar la trayectoria completa.
        """

        trajectory = []

        # Primera parte: curva Bézier cúbica de B a C, con CP1 como punto de control
        for t in np.linspace(0, 1, self.TRIANGLE_CHECKPOINTS):
            point = self.cubic_bezier(t, np.array(B), np.array(CP1), np.array(CP1), np.array(C))
            trajectory.append(point)

        # Segunda parte: curva Bézier cúbica de C a D, con CP2 como punto de control
        for t in np.linspace(0, 1, self.TRIANGLE_CHECKPOINTS):
            point = self.cubic_bezier(t, np.array(C), np.array(CP2), np.array(CP2), np.array(D))
            trajectory.append(point)

        return np.array(trajectory)


    # ########################
    # ---- CODIGO GENERAL ----
    # ########################

    def setObjetivo(self, obj):
        self.objetivoAlcanzado = False
        self.segmentoObjetivo = obj

    def tomarDecision(self, poseRobot):
        """
        Determina las velocidades lineal y angular del robot basadas en la posición actual y el objetivo.

        Args:
            poseRobot (tuple): Pose actual del robot, que incluye las coordenadas (x, y), el ángulo actual y otros datos adicionales.

        Returns:
            tuple: Una tupla con las velocidades calculadas:
                - Velocidad lineal (V)
                - Velocidad angular (W)

        Raises:
            KeyError: Si faltan variables o reglas en el sistema de inferencia difusa.

        Este método realiza los siguientes pasos:
        1. Extrae las coordenadas actuales y el ángulo del robot.
        2. Calcula la distancia y el ángulo hacia el objetivo.
        3. Utiliza un sistema de inferencia difusa para determinar las velocidades óptimas.
        4. Verifica la proximidad al objetivo para detener el movimiento cuando sea necesario.
        """

        # Extraer las coordenadas y el ángulo actual del robot
        x_robot, y_robot, current_angle, _, _ = poseRobot
        
        # Obtener las coordenadas del objetivo
        x_target, y_target = self.obtener_coordenadas_objetivo()
        
        # Calcular la distancia entre el robot y el objetivo
        self.distance = math.sqrt((x_target - x_robot) ** 2 + (y_target - y_robot) ** 2)
        
        # Calcular el ángulo hacia el objetivo
        angle = self.calcular_angulo(x_target, y_target, x_robot, y_robot, current_angle)
        
        try:
            # Realizar la inferencia difusa para determinar las velocidades
            result, confidence = self.inference_system(
                variables=self.variables,
                rules=self.rules,
                distance=self.distance,
                angle=abs(angle)
            )
        except KeyError as e:
            # Manejo de errores en caso de que falten variables o reglas
            print(f"KeyError occurred: {e}. Verifique las definiciones de las variables y reglas.")
            raise
        
        # Obtener la velocidad lineal y angular del resultado de la inferencia
        V = result.get("linear_velocity", 0)  # Velocidad lineal
        W = result.get("angular_velocity", 0)  # Velocidad angular

        # Si el ángulo es negativo, invertir la velocidad angular
        if angle < 0:
            W = -W

        # Verificar la proximidad al objetivo para detenerse si es necesario
        self.verificar_proximidad_objetivo(self.distance)

        # Retornar las velocidades calculadas
        return (V, W)
    
    def obtener_coordenadas_objetivo(self):
        """
        Obtiene las coordenadas del objetivo basándose en el tipo y estado del segmento actual.

        Returns:
            tuple: Coordenadas del objetivo (x_target, y_target).

        Este método realiza las siguientes operaciones:
        1. Comprueba el tipo de segmento:
        - Si el segmento es lineal (tipo 1), genera una trayectoria lineal o utiliza puntos de inicio/fin.
        - Si el segmento es triangular, calcula los puntos de control y genera una trayectoria curva.
        2. Determina el punto objetivo basado en el progreso actual (check_point_segmento o check_point_triangulo).
        3. Gestiona la variable `FRENAR` dependiendo de la proximidad al objetivo o la trayectoria completada.

        Condiciones especiales:
        - Si está activado `VOLVER_AL_INICIO` y se alcanza el último segmento, se utiliza el punto inicial como objetivo.
        - Se guarda el punto inicial al inicio del primer segmento si está configurado `VOLVER_AL_INICIO`.

        Returns:
            tuple: Coordenadas calculadas del objetivo en formato (x_target, y_target).
        """

        if self.segmentoObjetivo.getType() == 1:
            # Guardar datos del primer punto del primer segmento
            if self.VOLVER_AL_INICIO and self.segment_number == self.FIRST_SEGMENT_INDEX:
                self.start_point = self.segmentoObjetivo.getInicio()

            # Obtener las coordenadas del objetivo
            if self.VOLVER_AL_INICIO and self.segment_number == self.TOTAL_SEGMENT_NUMBER:
                x_target, y_target = self.start_point
            else:
                if self.check_point_segmento == 0:
                    self.line_trayectory = self.generate_linear_path(self.segmentoObjetivo.getInicio(), self.segmentoObjetivo.getFin())
                    x_target, y_target = self.line_trayectory[self.check_point_segmento]
                    self.FRENAR = False
                elif self.check_point_segmento == len(self.line_trayectory)-1:
                    x_target, y_target = self.line_trayectory[self.check_point_segmento]
                    self.FRENAR = False
                else:
                    x_target, y_target = self.line_trayectory[self.check_point_segmento]
                    self.FRENAR = False
        else:
            # Obtener las coordenadas del objetivo
            if self.VOLVER_AL_INICIO and self.segment_number == self.TOTAL_SEGMENT_NUMBER:
                x_target, y_target = self.start_point
            else:
                if self.check_point_triangulo == 0:
                    CP1, CP2 = self.calculate_control_points(self.segmentoObjetivo.getInicio(), self.segmentoObjetivo.getMedio(), self.segmentoObjetivo.getFin())

                    self.triangle_trayectory = self.generate_curved_path(self.segmentoObjetivo.getInicio(), self.segmentoObjetivo.getMedio(), self.segmentoObjetivo.getFin(), CP1, CP2)
                    x_target, y_target = self.triangle_trayectory[self.check_point_triangulo]
                    self.FRENAR = False
                elif self.check_point_triangulo == (self.TRIANGLE_CHECKPOINTS*2)-1:
                    x_target, y_target = self.triangle_trayectory[self.check_point_triangulo]
                    if self.segment_number == self.TOTAL_SEGMENT_NUMBER-1:
                        self.FRENAR = False
                    else:
                        self.FRENAR = False
                else:
                    x_target, y_target = self.triangle_trayectory[self.check_point_triangulo]
                    self.FRENAR = False
        return x_target, y_target

    def calcular_angulo(self, x_target, y_target, x_robot, y_robot, current_angle):
        """
        Calcula el ángulo de giro necesario para que el robot apunte hacia un objetivo.

        Args:
            x_target (float): Coordenada X del objetivo.
            y_target (float): Coordenada Y del objetivo.
            x_robot (float): Coordenada X actual del robot.
            y_robot (float): Coordenada Y actual del robot.
            current_angle (float): Ángulo actual del robot en grados.

        Returns:
            float: Ángulo de giro en grados, normalizado dentro del rango [0°, 360°].

        Detalles:
        1. Calcula el ángulo objetivo en radianes utilizando `atan2` para determinar la dirección hacia el objetivo.
        2. Convierte el ángulo objetivo de radianes a grados para mayor interpretabilidad.
        3. Normaliza tanto el ángulo objetivo como el ángulo actual del robot para garantizar que estén dentro del rango de 0° a 360°.
        4. Calcula la diferencia angular (ángulo de giro necesario) entre el ángulo objetivo y el ángulo actual.
        5. Normaliza la diferencia angular para evitar valores fuera del rango esperado.
        """

        # Calcular el ángulo hacia el objetivo en radianes
        goal_angle_rad = math.atan2(y_target - y_robot, x_target - x_robot)
        # Convertir el ángulo a grados
        goal_angle_degrees = math.degrees(goal_angle_rad)

        # Normalizar los ángulos para evitar problemas con ángulos negativos o mayores a 360°
        goal_angle_degrees = self.normalize_angle(goal_angle_degrees)
        current_angle = self.normalize_angle(current_angle)

        # Calcular la diferencia angular entre el ángulo objetivo y el ángulo actual del robot
        turn_angle_deg = self.normalize_angle(goal_angle_degrees - current_angle)

        return turn_angle_deg

    def normalize_angle(self, angle):
        return (angle + 180) % 360 - 180
    
    def verificar_proximidad_objetivo(self, distance):
        """
        Verifica si el robot está lo suficientemente cerca del objetivo y actualiza los estados de proximidad y checkpoints.

        Args:
            distance (float): Distancia actual entre el robot y el objetivo.

        Detalles:
        1. Para segmentos lineales:
            - Si la distancia al objetivo es menor o igual a `STOP_DISTANCE` y el robot está en el último checkpoint del segmento:
                - Marca el objetivo como alcanzado (`objetivoAlcanzado`).
                - Incrementa el número de segmento (`segment_number`).
                - Reinicia el checkpoint (`check_point_segmento`) para preparar el próximo segmento.
                - Si está activada la opción `VOLVER_AL_INICIO`, ajusta la lógica para continuar el trayecto cíclicamente.
            - Si la distancia es menor a `CHECKPOINT_DISTANCE_ACTIVATOR` y aún hay checkpoints disponibles:
                - Avanza al siguiente checkpoint (`check_point_segmento`).

        2. Para segmentos curvos (triangulares):
            - Si la distancia al objetivo es menor o igual a 0.5 y el robot está en el último checkpoint del segmento:
                - Marca el objetivo como alcanzado.
                - Reinicia el checkpoint del segmento triangular.
                - Incrementa el número de segmento.
            - Si la distancia al siguiente checkpoint es menor o igual a `MINIMUM_DISTANCE_TRIANGLE_CP`:
                - Avanza al siguiente checkpoint del segmento triangular.
        """

        if self.segmentoObjetivo.getType() == 1:
            # Si la distancia al objetivo es menor o igual a STOP_DISTANCE, detenerse completamente
            if distance <= self.STOP_DISTANCE and self.check_point_segmento == len(self.line_trayectory)-1:
                self.objetivoAlcanzado = True  # Marcar el objetivo como alcanzado
                self.segment_number += 1        # Pasar al siguiente segmento

                # Control de lógica para volver al inicio
                self.objetivoAlcanzado = self.segment_number != self.TOTAL_SEGMENT_NUMBER if self.VOLVER_AL_INICIO else True

                # Reiniciar el checkpoint
                self.check_point_segmento = 0

            # --- ACTIVACIÓN DEL CHECKPOINT ---
            elif distance < self.CHECKPOINT_DISTANCE_ACTIVATOR and self.check_point_segmento < len(self.line_trayectory)-1:
                self.check_point_segmento += 1
        else:
            if distance <= 0.5 and self.check_point_triangulo == (self.TRIANGLE_CHECKPOINTS*2)-1:
                self.objetivoAlcanzado = True
                self.check_point_triangulo = 0
                self.segment_number += 1 

            elif distance <= self.MINIMUM_DISTANCE_TRIANGLE_CP and self.check_point_triangulo <= (self.TRIANGLE_CHECKPOINTS*2)-2:
                self.check_point_triangulo += 1

    def esObjetivoAlcanzado(self):
        return self.objetivoAlcanzado
    
    def hayParteOptativa(self):
        return True
