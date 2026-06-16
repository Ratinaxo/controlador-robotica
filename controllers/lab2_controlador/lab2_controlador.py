from controller import Robot
import math
import csv
import os

# --- CONSTANTES FÍSICAS Y CONFIGURACIÓN ---
RADIO_RUEDA = 0.0205      # Metros (20.5 mm)
DISTANCIA_EJES = 0.052    # Metros (52 mm)
VEL_MAXIMA = 6.28         # Radianes por segundo

# --- PARÁMETROS DE FILTRADO Y KALMAN ---
ALPHA = 0.2               # Factor de suavizado para el promedio exponencial
Q_RUIDO_PROCESO = 0.001   # Confianza en el modelo/encoders (q)
R_RUIDO_MEDICION = 0.05   # Confianza en los sensores infrarrojos (R)

# Ajuste de sensibilidad para prevenir colisiones diagonales
UMBRAL_PELIGRO = 300.0       # Umbral unificado más sensible para activar evasión
UMBRAL_ZONA_DESPEJADA = 125.0    # Umbral bajo para asegurar que el entorno esté limpio

# --- INICIALIZACIÓN DE DISPOSITIVOS ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())
T_s = timestep / 1000.0   
f_s = 1.0 / T_s           

motor_izq = robot.getDevice('left wheel motor')
motor_der = robot.getDevice('right wheel motor')
motor_izq.setPosition(float('inf'))
motor_der.setPosition(float('inf'))
motor_izq.setVelocity(0.0)
motor_der.setVelocity(0.0)

enc_izq = robot.getDevice('left wheel sensor')
enc_der = robot.getDevice('right wheel sensor')
enc_izq.enable(timestep)
enc_der.enable(timestep)

nombres_sensores = {
    'f_izq': 'ps7', 'f_der': 'ps0', 
    'lat_izq': 'ps5', 'lat_der': 'ps2'
}
sensores = {}
for clave, nombre in nombres_sensores.items():
    sensores[clave] = robot.getDevice(nombre)
    sensores[clave].enable(timestep)

# --- DETECCIÓN DE MUNDO Y CONFIGURACIÓN DE POSES (FASE 1) ---
world_file = os.environ.get('WEBOTS_WORLD', '')
if 'MundoHard' in world_file:
    nombre_mundo = "MundoHard"
    # Pose inicial y meta para MundoHard (Complejo)
    x_ini = 0.04
    y_ini = 0.23
    phi_ini = -1.5708  # -90 grados (orientado al sur en radianes)
    x_meta = 0.35
    y_meta = -0.35
else:
    nombre_mundo = "MundoEz"
    # Pose inicial y meta para MundoEz (Simple)
    x_ini = -0.93
    y_ini = 0.0
    phi_ini = 0.0  # 0 grados (orientado al este en radianes)
    x_meta = 0.8
    y_meta = 0.0

print(f"[Sistema] Mundo detectado: {nombre_mundo}")
print(f"[Sistema] Pose inicial: ({x_ini}, {y_ini}) | Orientación: {phi_ini} rad ({round(math.degrees(phi_ini), 1)}°)")
print(f"[Sistema] Meta establecida en: ({x_meta}, {y_meta})")

# --- VARIABLES DE ESTADO GLOBAL (FASE 2) ---
pos_izq_ant = 0.0
pos_der_ant = 0.0
primera_iteracion = True

# Estado de la odometría acumulada
x_k = x_ini
y_k = y_ini
phi_k = phi_ini

# Historial del filtro EMA individual para cada sensor
filtrados_ant = {'f_izq': 0.0, 'f_der': 0.0, 'lat_izq': 0.0, 'lat_der': 0.0}

# Estado independiente para los 3 Filtros de Kalman
kalman_estados = {
    'frontal': {'d': 75.0, 'P': 1.0},
    'lat_izq': {'d': 75.0, 'P': 1.0},
    'lat_der': {'d': 75.0, 'P': 1.0}
}

ultimo_tiempo_impresion = 0.0

# --- FUNCIONES DE MOVIMIENTO ESTANDARIZADAS ---

def aplicar_velocidades(v_izq, v_der):
    v_izq_limitada = max(min(v_izq, VEL_MAXIMA), -VEL_MAXIMA)
    v_der_limitada = max(min(v_der, VEL_MAXIMA), -VEL_MAXIMA)
    motor_izq.setVelocity(v_izq_limitada)
    motor_der.setVelocity(v_der_limitada)

def Avanzar(velocidad_lineal=0.04):
    w_rueda = velocidad_lineal / RADIO_RUEDA
    aplicar_velocidades(w_rueda, w_rueda)

def GirarEnSitioDer(velocidad_giro=1.2):
    aplicar_velocidades(velocidad_giro, -velocidad_giro)

def GirarEnSitioIzq(velocidad_giro=1.2):
    aplicar_velocidades(-velocidad_giro, velocidad_giro)

# --- PIPELINE DE PROCESAMIENTO DE DATOS ---

def etapa_muestreo_y_crudos():
    global pos_izq_ant, pos_der_ant, primera_iteracion
    crudos = {clave: sens.getValue() for clave, sens in sensores.items()}
    
    pos_izq_actual = enc_izq.getValue()
    pos_der_actual = enc_der.getValue()
    
    # Evitar lecturas iniciales de tipo NaN
    if math.isnan(pos_izq_actual): pos_izq_actual = 0.0
    if math.isnan(pos_der_actual): pos_der_actual = 0.0
    
    if primera_iteracion:
        pos_izq_ant = pos_izq_actual
        pos_der_ant = pos_der_actual
        primera_iteracion = False
    
    # Ecuación de Odometría (Fórmulas 1 y 2)
    delta_izq = pos_izq_actual - pos_izq_ant
    delta_der = pos_der_actual - pos_der_ant
    
    pos_izq_ant = pos_izq_actual
    pos_der_ant = pos_der_actual
    
    # Desplazamientos lineales por rueda (Fórmulas 3 y 4)
    s_izq = RADIO_RUEDA * delta_izq
    s_der = RADIO_RUEDA * delta_der
    
    # Avance lineal instantáneo (Fórmula 5)
    avance_inst = (s_izq + s_der) / 2.0
    velocidad_lineal = avance_inst / T_s
    
    # Variación angular instantánea (Fórmula 6)
    delta_phi = (s_der - s_izq) / DISTANCIA_EJES
    
    return crudos, avance_inst, velocidad_lineal, delta_phi

def etapa_filtrado(crudos):
    global filtrados_ant
    
    filtrados_actuales = {}
    for clave in crudos.keys():
        filtrados_actuales[clave] = (ALPHA * crudos[clave]) + ((1.0 - ALPHA) * filtrados_ant[clave])
        filtrados_ant[clave] = filtrados_actuales[clave]
        
    return filtrados_actuales

def etapa_estimacion_kalman(z_k, avance_inst, tipo_kalman):
    global kalman_estados
    estado = kalman_estados[tipo_kalman]
    
    # Predicción
    if tipo_kalman == 'frontal':
        d_prediccion = estado['d'] + (avance_inst * 1500.0) 
    else:
        d_prediccion = estado['d']
        
    P_prediccion = estado['P'] + Q_RUIDO_PROCESO
    
    # Corrección
    K_k = P_prediccion / (P_prediccion + R_RUIDO_MEDICION)
    estado['d'] = d_prediccion + K_k * (z_k - d_prediccion)
    estado['P'] = (1.0 - K_k) * P_prediccion
    
    return estado['d']

# --- PREPARACIÓN DE ARCHIVO CSV ---
datos_csv = open('../../data_sensores/datos_sensores.csv', mode='w', newline='')
escritor_csv = csv.writer(datos_csv)
escritor_csv.writerow([
    'Tiempo (s)', 'Avance Inst (m)', 'Velocidad (m/s)',
    'Crudo Frontal Izq (ps7)', 'Crudo Frontal Der (ps0)', 'Promedio Frontal Crudo',
    'Crudo Lateral Izq (ps5)', 'Crudo Lateral Der (ps2)',
    'Filtro EMA ps7', 'Filtro EMA ps0', 'Filtro EMA ps5', 'Filtro EMA ps2',
    'Kalman Frontal', 'Kalman Lateral Izq', 'Kalman Lateral Der',
    'Pose X (m)', 'Pose Y (m)', 'Orientacion Phi (rad)'
])

# --- BUCLE PRINCIPAL ---
estado_navegacion = "AVANZAR"
direccion_giro = "DERECHA"

while robot.step(timestep) != -1:
    tiempo_actual = robot.getTime()
    
    # 1 y 2. Muestreo, Datos Crudos y Odometría (Fórmulas 1 a 6)
    datos_crudos, avance_lineal, vel_lineal, delta_phi = etapa_muestreo_y_crudos()
    promedio_frontal_crudo = (datos_crudos['f_izq'] + datos_crudos['f_der']) / 2.0
    
    # Ecuaciones Cinemáticas de Pose Global (Fórmula 7)
    phi_previo = phi_k
    phi_k = phi_previo + delta_phi
    
    # Normalizar orientación en el rango [-pi, pi]
    phi_k = (phi_k + math.pi) % (2 * math.pi) - math.pi
    
    # Integración global usando el promedio de ángulo (Runge-Kutta 2)
    x_k += avance_lineal * math.cos(phi_previo + delta_phi / 2.0)
    y_k += avance_lineal * math.sin(phi_previo + delta_phi / 2.0)
    
    # 3. Filtrado EMA Individual
    datos_filtrados = etapa_filtrado(datos_crudos)
    
    # 5. Estimación (Filtro de Kalman múltiple)
    kalman_frontal = etapa_estimacion_kalman(promedio_frontal_crudo, avance_lineal, 'frontal')
    kalman_lat_izq = etapa_estimacion_kalman(datos_crudos['lat_izq'], avance_lineal, 'lat_izq')
    kalman_lat_der = etapa_estimacion_kalman(datos_crudos['lat_der'], avance_lineal, 'lat_der')
    
    # Fusión Lateral para toma de decisiones
    max_lateral_kalman = max(kalman_lat_izq, kalman_lat_der)
    
    # Registro Completo en CSV
    escritor_csv.writerow([
        round(tiempo_actual, 3), round(avance_lineal, 6), round(vel_lineal, 4),
        round(datos_crudos['f_izq'], 2), round(datos_crudos['f_der'], 2), round(promedio_frontal_crudo, 2),
        round(datos_crudos['lat_izq'], 2), round(datos_crudos['lat_der'], 2),
        round(datos_filtrados['f_izq'], 2), round(datos_filtrados['f_der'], 2),
        round(datos_filtrados['lat_izq'], 2), round(datos_filtrados['lat_der'], 2),
        round(kalman_frontal, 2), round(kalman_lat_izq, 2), round(kalman_lat_der, 2),
        round(x_k, 5), round(y_k, 5), round(phi_k, 5)
    ])
    datos_csv.flush()
    
    # Telemetría en consola cada 2 segundos (Validación de Telemetría)
    if tiempo_actual - ultimo_tiempo_impresion >= 2.0:
        phi_grados = math.degrees(phi_k)
        print(f"[Telemetría {round(tiempo_actual, 1)}s] "
              f"Pose:({round(x_k, 3)}, {round(y_k, 3)}) | Ori:{round(phi_k, 3)} rad ({round(phi_grados, 1)}°) | "
              f"Kalman Frente:{round(kalman_frontal, 1)}")
        ultimo_tiempo_impresion = tiempo_actual
    
    # 6. Toma de Decisiones (Navegación Reactiva)
    if estado_navegacion == "AVANZAR":
        # Ahora evaluamos colisiones utilizando los 3 filtros de Kalman
        if (kalman_frontal > UMBRAL_PELIGRO) or (max_lateral_kalman > UMBRAL_PELIGRO):
            estado_navegacion = "ROTAR_EVASION"
            
            # Forzamos temporalmente el estado para evitar oscilaciones
            kalman_estados['frontal']['d'] = max(kalman_frontal, UMBRAL_PELIGRO + 100.0)
            
            if kalman_lat_izq > kalman_lat_der:
                direccion_giro = "DERECHA"
            else:
                direccion_giro = "IZQUIERDA"
        else:
            Avanzar(0.04)
            
    elif estado_navegacion == "ROTAR_EVASION":
        if (kalman_frontal < UMBRAL_ZONA_DESPEJADA) and (max_lateral_kalman < UMBRAL_ZONA_DESPEJADA):
            aplicar_velocidades(0.0, 0.0)
            estado_navegacion = "AVANZAR"
        else:
            if direccion_giro == "DERECHA":
                GirarEnSitioDer(1.2)
            else:
                GirarEnSitioIzq(1.2)

datos_csv.close()