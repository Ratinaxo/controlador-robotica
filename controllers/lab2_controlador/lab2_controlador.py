from controller import Robot
import math
import csv

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

# --- VARIABLES DE ESTADO GLOBAL ---
pos_izq_ant = 0.0
pos_der_ant = 0.0

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
    global pos_izq_ant, pos_der_ant
    crudos = {clave: sens.getValue() for clave, sens in sensores.items()}
    
    pos_izq_actual = enc_izq.getValue()
    pos_der_actual = enc_der.getValue()
    
    delta_izq = pos_izq_actual - pos_izq_ant
    delta_der = pos_der_actual - pos_der_ant
    
    pos_izq_ant = pos_izq_actual
    pos_der_ant = pos_der_actual
    
    avance_inst = (RADIO_RUEDA * (delta_izq + delta_der)) / 2.0
    velocidad_lineal = avance_inst / T_s
    return crudos, avance_inst, velocidad_lineal

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
    'Kalman Frontal', 'Kalman Lateral Izq', 'Kalman Lateral Der'
])

# --- BUCLE PRINCIPAL ---
estado_navegacion = "AVANZAR"
direccion_giro = "DERECHA"

while robot.step(timestep) != -1:
    tiempo_actual = robot.getTime()
    
    # 1 y 2. Muestreo y Datos Crudos
    datos_crudos, avance_lineal, vel_lineal = etapa_muestreo_y_crudos()
    promedio_frontal_crudo = (datos_crudos['f_izq'] + datos_crudos['f_der']) / 2.0
    
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
        round(kalman_frontal, 2), round(kalman_lat_izq, 2), round(kalman_lat_der, 2)
    ])
    datos_csv.flush()
    
    # Telemetría en consola cada 2 segundos
    if tiempo_actual - ultimo_tiempo_impresion >= 2.0:
        print(f"[Telemetría {round(tiempo_actual, 1)}s] "
              f"Front_Crudo(ps7:{round(datos_crudos['f_izq'], 1)}, ps0:{round(datos_crudos['f_der'], 1)}, Prom:{round(promedio_frontal_crudo, 1)}) | "
              f"Lat_Crudo(ps5:{round(datos_crudos['lat_izq'], 1)}, ps2:{round(datos_crudos['lat_der'], 1)}) | "
              f"Kalman(Frente:{round(kalman_frontal, 1)}, Lat_I:{round(kalman_lat_izq, 1)}, Lat_D:{round(kalman_lat_der, 1)})")
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