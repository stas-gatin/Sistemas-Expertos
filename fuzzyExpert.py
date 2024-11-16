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

        self.objetivoAlcanzado = False
        self.segmentoObjetivo = None

        self.VOLVER_AL_INICIO: bool = True             # Controla si el robot debe regresar al punto inicial del trayecto
        self.FRENAR: bool = None                       # Indica si el robot debe frenar

        # --- Velocidades y modo de movimiento ---
        self.velocidad: float = 0                      # Velocidad lineal inicial (en m/s)
        self.velocidad_angular: float = 0              # Velocidad angular inicial (en rad/s)
        self.reverse: bool = False                     # Modo de reversa inicial
        self.distance: float = 0.0

        # --- Parámetros de la trayectoria linear ---
        self.check_point_segmento: int = 0        # Estado del trayecto: punto inicial (False) o final (True)
        self.LINE_CHECKPOINTS: int = 20                 # Cantidad de puntos de control en trayecto linear
        self.line_trayectory = None                    # Lista de coordenadas del trayecto linear

        self.start_point: tuple = None                 # Punto de inicio del trayecto
        self.segment_number: int = 0                   # Número del segmento actual
        self.distance: int = 0                         # Distancia hasta el punto objetivo
        self.STOP_DISTANCE: float = 0.5                # Distancia para detener el robot al final del segmento
        self.CHECKPOINT_DISTANCE_ACTIVATOR: float = 5  # Distancia que activa el cambio de punto de control
        self.CONSTANTE_AUMENTAR_VELOCIDAD: float = 1.5     # Constante para aumentar velocidad en trayecto
        self.FIRST_SEGMENT_INDEX: int = 0                  # Índice del primer segmento del trayecto
        self.TOTAL_SEGMENT_NUMBER: int = len(objectiveSet) # Número total de segmentos en el trayecto

        # --- Parámetros de la trayectoria triangular ---
        self.check_point_triangulo: int = 0                # Contador de puntos de control en el trayecto triangular
        self.TRIANGLE_CHECKPOINTS: int = 5                 # Cantidad de puntos de control en trayecto triangular
        self.triangle_trayectory = None                    # Lista de coordenadas del trayecto triangular
        self.CONTROL_POINT_CONSTANT: float = 0.7           # Constante de ajuste de puntos de control en trayecto triangular
        self.MINIMUM_DISTANCE_TRIANGLE_CP: int = 4         # Distancia mínima para activar puntos de control

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

    def setObjetivo(self, obj):
        self.objetivoAlcanzado = False
        self.segmentoObjetivo = obj

    def tomarDecision(self, poseRobot):
        """
        Toma una decisión basada en la posición actual del robot y el objetivo.

        Args:
            poseRobot (tuple): Una tupla que contiene las coordenadas (x, y) del robot,
                            el ángulo actual del robot, y otros datos irrelevantes.

        Returns:
            tuple: Una tupla que contiene la velocidad lineal (V) y la velocidad angular (W)
                que el robot debe seguir para alcanzar el objetivo.
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
        Determina las coordenadas del objetivo actual en función del estado del robot y el segmento en el que se encuentra.
        También gestiona la lógica de volver al inicio y actualiza el estado de frenado.

        Returns:
            tuple: Una tupla con las coordenadas del objetivo (x_target, y_target).
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

                    self.triangle_trayectory = self.generate_trajectory(self.segmentoObjetivo.getInicio(), self.segmentoObjetivo.getMedio(), self.segmentoObjetivo.getFin(), CP1, CP2)
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
    
    def generate_linear_path(self, A, B):
        """
        Genera una trayectoria lineal entre dos puntos A y B.

        Args:
            A (tuple): Coordenadas del punto inicial (x1, y1).
            B (tuple): Coordenadas del punto final (x2, y2).

        Returns:
            list: Una lista de coordenadas que representan la trayectoria lineal entre A y B.
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
    
    def cubic_bezier(self, t, P0, P1, P2, P3):
        """
        Calcula una posición en una curva cúbica de Bézier.

        Parámetros:
        t (float): Un valor en el rango [0, 1] que indica la progresión a lo largo de la curva.
        P0, P1, P2, P3 (numpy.array): Los puntos de control de la curva.

        Retorna:
        numpy.array: El punto calculado en la curva de Bézier para el valor dado de t.
        """
        return (1 - t)**3 * P0 + 3 * (1 - t)**2 * t * P1 + 3 * (1 - t) * t**2 * P2 + t**3 * P3

    
    def calcular_offset(self, A, B, C):
        """
        Calcula el offset para una trayectoria curva que pasa por el triángulo ABC, 
        controlado por la longitud de la base AB y la altura del punto C desde esta base.
        
        Parámetros:
            A (tuple): Coordenadas de la primera esquina de la base (A).
            B (tuple): Coordenadas de la segunda esquina de la base (B).
            C (tuple): Coordenadas del vértice superior del triángulo (C).
        
        Retorna:
            float: El valor de offset, limitado entre 0.5 y 1.5, dependiendo de la forma del triángulo.
        
        La función calcula la longitud de la base y la altura desde C hasta AB, y ajusta el offset
        para permitir una curva más amplia si la base es corta y C está lejos de ella, o una curva más 
        cerrada si la base es larga y C está cerca.
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
        Calcula los puntos de control que definen las curvas fuera de la línea entre los puntos clave.

        Parámetros:
        B, C, D (numpy.array): Los puntos clave entre los cuales se construirán las curvas.
        offset (float): La distancia que define cuánto se desplazan los puntos de control desde la línea recta.

        Retorna:
        tuple: Los puntos de control CP1 y CP2 que generan las curvas suaves entre B-C y C-D.
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
    
    def generate_trajectory(self, B, C, D, CP1, CP2):
        """
        Genera una trayectoria usando dos curvas cúbicas de Bézier, que pasa por los puntos B, C y D,
        con puntos de control CP1 y CP2 para crear curvas suaves.

        Parámetros:
        B, C, D (numpy.array): Los puntos clave de la trayectoria.
        CP1, CP2 (numpy.array): Los puntos de control que definen el curvado entre B-C y C-D.
        num_points (int): El número de puntos a generar en cada segmento de la curva.

        Retorna:
        numpy.array: La trayectoria generada como una lista de puntos.
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

    def calcular_angulo(self, x_target, y_target, x_robot, y_robot, current_angle):
        """
        Calcula el ángulo hacia el objetivo, normaliza los ángulos y calcula la diferencia angular.

        Args:
            x_target (float): Coordenada X del objetivo.
            y_target (float): Coordenada Y del objetivo.
            x_robot (float): Coordenada X actual del robot.
            y_robot (float): Coordenada Y actual del robot.
            current_angle (float): Ángulo de orientación actual del robot en grados.

        Returns:
            tuple: Una tupla con los siguientes valores:
                - goal_angle_degrees (float): Ángulo hacia el objetivo en grados.
                - turn_angle_deg (float): Diferencia angular entre el ángulo objetivo y el ángulo actual en grados.
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
    
    def verificar_proximidad_objetivo(self, distance):
        """
        Verifica la proximidad del robot al objetivo y gestiona la lógica de parada, activación de checkpoints,
        y reducción suave de la velocidad.

        Args:
            distance (float): La distancia actual entre el robot y el objetivo.
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

    def normalize_angle(self, angle):
        return (angle + 180) % 360 - 180
    
    def esObjetivoAlcanzado(self):
        return self.objetivoAlcanzado
    
    def hayParteOptativa(self):
        return True
