'''
 Sistema Experto para el guiado de un robot
 Esta clase contendrá el código creado por los alumnos de RyRDC para el control 
 y guiado de un robot móvil sobre un plano cartesiano pasando por un punto inicial
 y siguiendo una línea recta hasta un punto final

 Creado por: Stanislav Gatin

'''

from segmento import *
import math
from robot import WACC, WMAX, VACC, VMAX
import numpy as np

class ExpertSystem:
    
    def __init__(self) -> None:
        from P1Launcher import objectiveSet  # Importación de los objetivos del trayecto

        # --- Flags y estados del trayecto ---
        self.objetivoAlcanzado: bool = False           # Indica si el robot ha alcanzado su objetivo final
        self.segmentoObjetivo: object = None           # Segmento actual del trayecto
        self.VOLVER_AL_INICIO: bool = True             # Controla si el robot debe regresar al punto inicial
        self.GO_AROUND_TRIANGLE: bool = False          # Indica si el robot debe rodear un obstáculo triangular
        self.FRENAR: bool = None                       # Indica si el robot debe frenar

        # --- Velocidades y modos de movimiento ---
        self.velocidad: float = 0                      # Velocidad lineal inicial (en m/s)
        self.velocidad_angular: float = 0              # Velocidad angular inicial (en rad/s)
        self.reverse: bool = False                     # Modo reversa activado/desactivado

        # --- Parámetros de la trayectoria lineal ---
        self.check_point_segmento: int = 0             # Estado del trayecto: punto inicial o final
        self.LINE_CHECKPOINTS: int = 20                # Cantidad de puntos de control en trayecto lineal
        self.line_trayectory = None                    # Lista de coordenadas del trayecto lineal
        self.start_point: tuple = None                 # Punto de inicio del trayecto
        self.segment_number: int = 0                   # Número del segmento actual
        self.distance: int = 0                         # Distancia al punto objetivo
        self.STOP_DISTANCE: float = 0.2                # Distancia para detener el robot
        self.CHECKPOINT_DISTANCE_ACTIVATOR: float = 0.5 # Distancia que activa cambio de punto de control
        self.CONSTANTE_AUMENTAR_VELOCIDAD: float = 1.5  # Constante para aumentar velocidad
        self.FIRST_SEGMENT_INDEX: int = 0              # Índice del primer segmento
        self.TOTAL_SEGMENT_NUMBER: int = len(objectiveSet) # Número total de segmentos
        self.LINE_EXPANSION_FACTOR: float = 0.03       # Factor para extender punto inicial de la trayectoria lineal

        # --- Parámetros de la trayectoria triangular ---
        self.check_point_triangulo: int = 0            # Contador de puntos de control en trayecto triangular
        self.MAX_TRIANGLE_CHECKPOINTS: int = 4         # Cantidad máxima de puntos en trayecto triangular
        self.CURRENT_TRIANGLE_CHECKPOINTS: int = 0     # Puntos actuales en la trayectoria triangular
        self.triangle_trayectory = None                # Lista de coordenadas del trayecto triangular
        self.CONTROL_POINT_CONSTANT: float = 0.5       # Constante para ajustar puntos de control
        self.TRIANGLE_SPEED: float = 1                 # Velocidad para movimiento triangular
        self.MINIMUM_DISTANCE_TRIANGLE_CP: float = 1.5 # Distancia mínima para activar puntos de control
        self.CURVE_EXPANSION_FACTOR: float = 0.25      # Factor para expansión de curvas
        self.CURVE_CONTROLL_POINTS_OFFSET: float = 0.5 # Desplazamiento de puntos de control en curvas

        # --- Ángulos y control de giros ---
        self.turn_angle_rad: float = 0.0               # Ángulo de giro en radianes
        self.turn_angle_deg: float = 0.0               # Ángulo de giro en grados
        self.REVERSE_THRESHOLD: int = 90               # Umbral en grados para activar marcha atrás
        self.MAXIMUM_ANGLE_DEG: int = 10               # Ángulo máximo antes de desviarse
        self.DISTANCE_TURN_CONSTANT: float = 4.5       # Constante para ajustar ángulo según distancia
        self.VELOCIDAD_ANGULAR_CONSTANT: int = 2       # Constante para ajustar velocidad angular

    # función setObjetivo
    #   Especifica un segmento como objetivo para el recorrido del robot
    #   Este método NO debería ser modificado
    def setObjetivo(self, segmento):
        self.objetivoAlcanzado = False
        self.segmentoObjetivo = segmento

    # #######################
    # ---- CURVE CONTROLL ----
    # #######################

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
        offset = self.CURVE_CONTROLL_POINTS_OFFSET + (2 * (altura / (longitud_base + 1)))
        
        # Limita el offset entre 0.5 y 1.5
        offset = max(0.5, min(offset, 1.5))
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
    
    def move_point_C_perpendicular(self, B, C, D):
        """
        Desplaza el punto C hacia arriba en relación con el segmento B-D a una distancia dada.

        Parámetros:
        B, C, D (numpy.array): Puntos B, C y D.
        distancia (float): Distancia para desplazar el punto C.

        Devuelve:
        numpy.array: Nuevo punto C desplazado perpendicularmente al segmento B-D.
        """
        B = np.array(B)
        C = np.array(C)
        D = np.array(D)

        # Vector de B a D
        vector_BD = D - B

        # Vector perpendicular a BD (en 2D se pueden intercambiar los componentes e invertir el signo de uno de ellos)
        vector_perpendicular = np.array([-vector_BD[1], vector_BD[0]])  # Vector perpendicular a BD

        # Normalizamos el vector perpendicular
        vector_perpendicular_unitario = vector_perpendicular / np.linalg.norm(vector_perpendicular)

        # Desplazamos el punto C a lo largo del vector perpendicular
        nuevo_C = C + vector_perpendicular_unitario * self.CURVE_EXPANSION_FACTOR

        return nuevo_C
    
    def find_circumcenter(self, B, C, D):
        """
        Encuentra el circuncentro del triángulo definido por los puntos B, C y D.

        Parámetros:
        B, C, D (numpy.array): Puntos que definen los vértices del triángulo.

        Devuelve:
        numpy.array: Coordenadas del circuncentro del triángulo.

        Lanza:
        ValueError: Si las líneas perpendiculares no se intersectan en un único punto.
        """
        A = np.array(B)
        B = np.array(C)
        C = np.array(D)

        # Calculamos los vectores AB, BC, CA
        AB = B - A
        BC = C - B
        CA = A - C

        # Encontramos los puntos medios de los vectores
        punto_medio_AB = A + 0.5 * AB
        punto_medio_BC = B + 0.5 * BC
        punto_medio_CA = C + 0.5 * CA

        # Calculamos las direcciones de las líneas perpendiculares
        perp_AB = np.array([-AB[1], AB[0]])
        perp_BC = np.array([-BC[1], BC[0]])
        perp_CA = np.array([-CA[1], CA[0]])

        # Ecuaciones de las líneas en la forma Ax + By = C
        def ecuacion_linea(p1, p2):
            """
            Calcula la ecuación de una línea en la forma Ax + By = C.

            Parámetros:
            p1, p2 (numpy.array): Puntos por los que pasa la línea.

            Devuelve:
            tuple: Coeficientes A, B y C de la ecuación de la línea.
            """
            A = p2[1] - p1[1]
            B = p1[0] - p2[0]
            C = A * p1[0] + B * p1[1]
            return A, B, C

        # Ecuaciones de las líneas perpendiculares en los puntos medios
        A1, B1, C1 = ecuacion_linea(punto_medio_AB, punto_medio_AB + perp_AB)
        A2, B2, C2 = ecuacion_linea(punto_medio_BC, punto_medio_BC + perp_BC)
        A3, B3, C3 = ecuacion_linea(punto_medio_CA, punto_medio_CA + perp_CA)

        # Encontramos el punto de intersección de las dos primeras líneas
        def interseccion(A1, B1, C1, A2, B2, C2):
            """
            Calcula el punto de intersección de dos líneas en la forma Ax + By = C.

            Parámetros:
            A1, B1, C1 (float): Coeficientes de la primera línea.
            A2, B2, C2 (float): Coeficientes de la segunda línea.

            Devuelve:
            numpy.array: Coordenadas del punto de intersección.

            Lanza:
            ValueError: Si las líneas son paralelas o no se intersectan.
            """
            det = A1 * B2 - A2 * B1
            if det == 0:
                raise ValueError("Las líneas no se intersectan o son paralelas")
            x = (B2 * C1 - B1 * C2) / det
            y = (A1 * C2 - A2 * C1) / det
            return np.array([x, y])

        # Calculamos el punto de intersección de las dos primeras líneas
        P = interseccion(A1, B1, C1, A2, B2, C2)

        # Verificamos que el punto P está en la tercera línea
        if np.isclose(A3 * P[0] + B3 * P[1], C3):
            return P
        else:
            raise ValueError("Las líneas no se intersectan en un único punto")
    
    def add_point_above_D(self, B, C, D):
        """
        Crea tres puntos por encima del punto D que son perpendiculares al segmento B-D.

        Parámetros:
        B, C, D (numpy.array): Coordenadas de los puntos B, C y D.

        Devuelve:
        tuple: Tres nuevos puntos desplazados perpendicularmente al segmento B-D.
        """
        # Encuentra el punto de referencia como el circuncentro de B, C y D
        punto_referencia = self.find_circumcenter(B, C, D)
        D = np.array(D)

        # Vector desde el punto de referencia hacia D
        vector_BD = D - punto_referencia

        # Vector perpendicular al segmento BD
        vector_perpendicular = np.array([-vector_BD[1], vector_BD[0]])

        # Normaliza el vector perpendicular
        vector_perpendicular_unitario = vector_perpendicular / np.linalg.norm(vector_perpendicular)

        # Desplaza el punto D para generar nuevos puntos
        nuevo_punto1 = D + vector_perpendicular_unitario * 4
        nuevo_punto2 = D + vector_perpendicular_unitario * 2.5
        nuevo_punto3 = D + vector_perpendicular_unitario * 1.5

        return nuevo_punto1, nuevo_punto2, nuevo_punto3
    
    def generate_curved_path(self, B, C, D):
        """
        Genera una trayectoria utilizando dos curvas de Bézier cúbicas que pasan por los puntos B, C y D,
        con puntos de control CP1 y CP2 para crear curvas suaves.

        Parámetros:
        B, C, D (numpy.array): Puntos clave de la trayectoria.
        shift_distance (float): Distancia para ajustar el punto C de manera perpendicular.

        Devuelve:
        numpy.array: Trayectoria generada como una lista de puntos.
        """
        # Verifica si debe cerrar la trayectoria en el segmento final
        if self.VOLVER_AL_INICIO and self.segment_number == self.TOTAL_SEGMENT_NUMBER - 1:
            trayectoria = []
            trayectoria.append(B)
            trayectoria.append(C)
            trayectoria.append(D)
            self.CURRENT_TRIANGLE_CHECKPOINTS = len(trayectoria)
            return np.array(trayectoria)
        else:
            # Genera un punto único perpendicular al segmento B-D y por encima de D
            D_nuevo, punto_encima_D2, punto_encima_D3 = self.add_point_above_D(B, C, D)
            nuevo_C = self.move_point_C_perpendicular(B, C, D)
            CP1, CP2 = self.calculate_control_points(self.segmentoObjetivo.getInicio(), nuevo_C, D_nuevo)
            trayectoria = []

            # Primera parte: curva de Bézier cúbica de B a C con CP1 como punto de control
            for t in np.linspace(0, 1, self.MAX_TRIANGLE_CHECKPOINTS):
                punto = self.cubic_bezier(t, np.array(B), np.array(CP1), np.array(CP1), np.array(nuevo_C))
                trayectoria.append(punto)

            # Segunda parte: curva de Bézier cúbica de C a D con CP2 como punto de control
            for t in np.linspace(0, 1, self.MAX_TRIANGLE_CHECKPOINTS):
                punto = self.cubic_bezier(t, np.array(nuevo_C), np.array(CP2), np.array(CP2), np.array(D_nuevo))
                trayectoria.append(punto)

            # Añade puntos adicionales por encima de D
            trayectoria.append(punto_encima_D2)
            trayectoria.append(punto_encima_D3)
            trayectoria.append(D)

            self.CURRENT_TRIANGLE_CHECKPOINTS = len(trayectoria)

            return np.array(trayectoria)

    # #######################
    # ---- LINE CONTROLL ----
    # #######################
    def generate_point_on_extension(self, A, B):
        """
        Genera un punto en la extensión de la línea definida por los puntos A y B.

        Parámetros:
        A, B (tuple): Coordenadas de los puntos A y B.
        k (float, opcional): Factor de extensión. Por defecto es 0.05.

        Devuelve:
        tuple: Coordenadas del nuevo punto generado en la extensión.
        """
        # Coordenadas de los puntos A y B
        x1, y1 = A
        x2, y2 = B

        # Cálculo de las coordenadas del punto C
        x = x1 - self.LINE_EXPANSION_FACTOR * (x2 - x1)
        y = y1 - self.LINE_EXPANSION_FACTOR * (y2 - y1)

        return (x, y)

    def generate_linear_path(self, A, B):
        """
        Genera una trayectoria lineal entre los puntos A y B, incluyendo puntos intermedios
        con precisión flotante y, opcionalmente, un punto en la extensión de la línea.

        Parámetros:
        A, B (tuple): Coordenadas de los puntos inicial (A) y final (B).

        Devuelve:
        list: Lista de puntos (tuplas) que forman la trayectoria lineal, incluyendo puntos intermedios.
        """
        # Coordenadas del punto A
        x1, y1 = float(A[0]), float(A[1])
        # Coordenadas del punto B
        x2, y2 = float(B[0]), float(B[1])

        puntos = []

        # Si el número de segmento es 0, añade un punto en la extensión de la línea
        if self.segment_number == 0:
            puntos.append(self.generate_point_on_extension(A, B))

        # Añadimos el punto A al inicio
        puntos.append(A)

        # Calculamos los incrementos entre puntos en los ejes x e y
        dx = (x2 - x1) / (self.LINE_CHECKPOINTS + 1)
        dy = (y2 - y1) / (self.LINE_CHECKPOINTS + 1)

        # Añadimos puntos intermedios con precisión flotante
        for i in range(1, self.LINE_CHECKPOINTS + 1):
            x = round(x1 + i * dx, 6)
            y = round(y1 + i * dy, 6)
            puntos.append((x, y))

        # Añadimos el punto B al final
        puntos.append(B)
        return puntos

    # ########################
    # ---- CODIGO GENERAL ----
    # ########################
    
    def normalize_angle(self, angle):
        """
        Normaliza un ángulo al rango [-180, 180) grados.
        """
        return (angle + 180) % 360 - 180

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
    
    def decidir_modo_movimiento(self, turn_angle_deg):
        """
        Decide el modo de movimiento del robot.
        """
        self.reverse = abs(turn_angle_deg) > self.REVERSE_THRESHOLD

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

            # --- REDUCCIÓN SUAVE DE VELOCIDAD ---
            elif self.FRENAR == True:
                # Calcular la distancia necesaria para detenerse suavemente
                stop_distance = (self.velocidad ** 2) / (2 * VACC)
                if distance <= stop_distance:
                    # Reducir la velocidad proporcionalmente a la distancia restante
                    self.velocidad *= (distance / stop_distance)
        else:
            if distance <= 0.5 and self.check_point_triangulo == self.CURRENT_TRIANGLE_CHECKPOINTS-1:
                self.objetivoAlcanzado = True
                self.check_point_triangulo = 0
                self.segment_number += 1 

            elif distance <= self.MINIMUM_DISTANCE_TRIANGLE_CP and self.check_point_triangulo <= self.CURRENT_TRIANGLE_CHECKPOINTS-2:
                self.check_point_triangulo += 1

            # --- REDUCCIÓN SUAVE DE VELOCIDAD ---
            elif self.FRENAR == True:
                # Calcular la distancia necesaria para detenerse suavemente
                stop_distance = (self.velocidad ** 2) / (2 * VACC)
                if distance <= stop_distance:
                    # Reducir la velocidad proporcionalmente a la distancia restante
                    self.velocidad *= (distance / stop_distance)

    def obtener_coordenadas_objetivo(self):
        """
        Determina las coordenadas del objetivo actual en función del estado del robot y el segmento en el que se encuentra.
        También gestiona la lógica de volver al inicio y actualiza el estado de frenado.

        Returns:
            tuple: Una tupla con las coordenadas del objetivo (x_target, y_target).
        """
        if self.segmentoObjetivo.getType() == 1:
            if self.check_point_segmento == 0:
                self.line_trayectory = self.generate_linear_path(self.segmentoObjetivo.getInicio(), self.segmentoObjetivo.getFin())
                if self.segment_number == 0:
                    self.FRENAR = True
                    self.CHECKPOINT_DISTANCE_ACTIVATOR = 2
            else:
                self.FRENAR = False
                self.CHECKPOINT_DISTANCE_ACTIVATOR = 0.5
            x_target, y_target = self.line_trayectory[self.check_point_segmento]
        else:
            if self.check_point_triangulo == 0:
                self.triangle_trayectory = self.generate_curved_path(self.segmentoObjetivo.getInicio(), self.segmentoObjetivo.getMedio(), self.segmentoObjetivo.getFin())
            x_target, y_target = self.triangle_trayectory[self.check_point_triangulo]
            self.FRENAR = False
        return x_target, y_target
    
    def calcular_distancia_objetivo(self, x_target, y_target, x_robot, y_robot):
        """
        Calcula la distancia euclidiana entre el robot y el objetivo.

        Args:
            x_target (float): Coordenada X del objetivo.
            y_target (float): Coordenada Y del objetivo.
            x_robot (float): Coordenada X actual del robot.
            y_robot (float): Coordenada Y actual del robot.

        Returns:
            float: Distancia euclidiana entre el robot y el objetivo.
        """
        return math.sqrt((x_target - x_robot) ** 2 + (y_target - y_robot) ** 2)
    
    def calcular_velocidad_lineal(self, turn_angle_rad, distance):
        """
        Calcula y actualiza la velocidad lineal del robot en función del ángulo de giro y la distancia al objetivo.

        Args:
            turn_angle_rad (float): Ángulo de giro en radianes.
            distance (float): Distancia euclidiana entre el robot y el objetivo.
        """
        if self.segmentoObjetivo.getType() == 1:
            if (self.check_point_segmento == 0 and abs(turn_angle_rad) < math.radians(distance * self.DISTANCE_TURN_CONSTANT)) or \
            (self.check_point_segmento == 0 and abs(turn_angle_rad) < math.radians(self.MAXIMUM_ANGLE_DEG)):
                self.velocidad = min(VMAX, distance * VACC * self.CONSTANTE_AUMENTAR_VELOCIDAD)  # VMAX es la velocidad máxima, VACC es el coeficiente de aceleración lineal
            elif self.check_point_segmento != 0 and abs(turn_angle_rad) < math.radians(self.MAXIMUM_ANGLE_DEG):
                self.velocidad = 3  
            else:
                self.velocidad = 0  # Si no cumple con las condiciones, la velocidad es 0
        else:
            if self.segment_number == self.TOTAL_SEGMENT_NUMBER-1:
                velocidad_angular_factor = max(0, 1 - abs(turn_angle_rad) / math.radians(90))
                self.velocidad = min(VMAX, distance * VACC * self.TRIANGLE_SPEED * velocidad_angular_factor * 1.5) 
            else:
                velocidad_angular_factor = max(0, 1 - abs(turn_angle_rad) / math.radians(90))
                self.velocidad = min(VMAX, distance * VACC * self.TRIANGLE_SPEED * velocidad_angular_factor * 1.25)

    def calcular_velocidad_angular(self, turn_angle_rad):
        """
        Calcula y actualiza la velocidad angular del robot en función del ángulo de giro.

        Args:
            turn_angle_rad (float): Ángulo de giro en radianes.
        """
        # Calcular la velocidad angular proporcional al ángulo de giro necesario
        self.velocidad_angular = turn_angle_rad * WACC * self.VELOCIDAD_ANGULAR_CONSTANT  # WACC es el coeficiente de aceleración angular
        
        # Limitar la velocidad angular a los valores máximos permitidos
        self.velocidad_angular = max(-WMAX, min(WMAX, self.velocidad_angular))  # WMAX es la velocidad angular máxima

        if self.segmentoObjetivo.getType() == 2:
            self.velocidad_angular = max(-WMAX, min(WMAX, self.velocidad_angular*1.5))

    def tomarDecision(self, poseRobot):
        """
        Toma una decisión de movimiento para el robot basado en su posición actual
        y la posición del objetivo en el segmento.

        Este método calcula las velocidades lineal y angular necesarias para mover
        al robot hacia el objetivo de manera eficiente, ajustando su trayectoria 
        en función del ángulo de giro y la distancia al objetivo. También considera 
        la posibilidad de moverse en reversa para optimizar el movimiento en casos 
        donde el ángulo de giro es mayor al umbral definido.

        Además, se maneja la lógica de activación de checkpoints y el retorno al 
        inicio de la trayectoria si es necesario.

        Args:
            poseRobot (tuple): Una tupla que contiene la posición actual del robot y su
                            ángulo de orientación en grados, en el formato (x_robot, y_robot, current_angle).

        Returns:
            tuple: Una tupla con las velocidades lineal y angular calculadas
                (velocidad, velocidad_angular), donde:
                - velocidad (float): Velocidad lineal en m/s.
                - velocidad_angular (float): Velocidad angular en rad/s.

        Comportamiento:
            - Si el ángulo de giro es menor que un valor calculado (dependiendo de la 
            distancia o un ángulo máximo fijo), el robot avanza hacia el objetivo.
            - Si la distancia al objetivo es menor o igual a un umbral de parada, el 
            robot se detiene completamente.
            - Si el ángulo de giro excede un umbral definido, el robot puede activar 
            el modo reversa para minimizar el giro.
            - Controla la lógica de volver al inicio cuando el robot ha alcanzado el 
            último segmento de la trayectoria.
        """

        # --- RECOPILAR DATOS ---
        x_target, y_target = self.obtener_coordenadas_objetivo()
        # Obtener las coordenadas actuales del robot
        x_robot, y_robot, current_angle, _, _ = poseRobot

        # --- CÁLCULO DE ANGULO ---
        self.turn_angle_deg = self.calcular_angulo(x_target, y_target, x_robot, y_robot, current_angle)

        # --- DECIDIR MODO DE MOVIMIENTO: ADELANTE O REVERSA ---
        self.decidir_modo_movimiento(self.turn_angle_deg)

        if self.reverse:
            self.turn_angle_deg = self.normalize_angle(self.turn_angle_deg - 180)  # Ajustar el ángulo para reversa

        # Convertir el ángulo de giro a radianes para cálculos posteriores
        self.turn_angle_rad = math.radians(self.turn_angle_deg)

        # --- CÁLCULO DE LA VELOCIDAD ANGULAR ---
        self.calcular_velocidad_angular(self.turn_angle_rad)

        # --- CÁLCULO DE LA DISTANCIA AL OBJETIVO ---
        self.distance = self.calcular_distancia_objetivo(x_target, y_target, x_robot, y_robot)

        # --- CÁLCULO DE LA VELOCIDAD LINEAL ---
        self.calcular_velocidad_lineal(self.turn_angle_rad, self.distance)

        # --- VERIFICACIÓN DE PROXIMIDAD AL OBJETIVO PARA DETENERSE ---
        self.verificar_proximidad_objetivo(self.distance)

        # --- AJUSTE DE LA VELOCIDAD PARA MOVIMIENTO REVERSO ---
        # Si se ha activado el modo de reversa, invertir la velocidad lineal
        if self.reverse:
            self.velocidad = -self.velocidad

        # --- DEVOLVER LAS VELOCIDADES CALCULADAS ---
        return self.velocidad, self.velocidad_angular
    
    # función esObjetivoAlcanzado 
    #   Devuelve True cuando el punto final del objetivo ha sido alcanzado. 
    #   Es responsabilidad de la alumna o alumno cambiar el valor de la 
    #   variable objetivoAlcanzado cuando se detecte que el robot ha llegado 
    #   a su objetivo. Esto se llevará a cabo en el método tomarDecision
    #   Este método NO debería ser modificado
    def esObjetivoAlcanzado(self):
        return self.objetivoAlcanzado
    
    def hayParteOptativa(self):
        return True
