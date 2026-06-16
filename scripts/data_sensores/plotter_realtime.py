import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os
import warnings
from pathlib import Path

# Desactivar advertencias visuales de Matplotlib
warnings.filterwarnings("ignore")

DATA_DIR = Path(__file__).resolve().parent
SENSORS_CSV = DATA_DIR / "datos_sensores.csv"

# 1. Configuración inicial de la figura con 3 subgráficas
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 14), sharex=True)
fig.canvas.manager.set_window_title('Telemetría Webots en Tiempo Real')

def actualizar_grafica(frame):
    nombre_archivo = SENSORS_CSV
    
    if not os.path.exists(nombre_archivo):
        print(f"Buscando el archivo '{nombre_archivo}'... Asegúrate de iniciar la simulación en Webots.")
        return

    try:
        df = pd.read_csv(nombre_archivo)
        df.columns = df.columns.str.strip()
        
        if df.shape[1] < 15:
            print(f"Esperando estructura completa del CSV. Columnas detectadas: {df.shape[1]}/15")
            return
            
        if len(df) < 2:
            return

        # --- LÓGICA DE VENTANA DE TIEMPO (30 SEGUNDOS) ---
        # 1. Obtenemos el tiempo más reciente de la simulación
        tiempo_actual = df.iloc[-1, 0]
        
        # 2. Calculamos el límite inferior (0 si no han pasado 30s, o (t - 30) si ya pasaron)
        limite_inferior = max(0.0, tiempo_actual - 30.0)
        
        # 3. Filtramos el DataFrame para retener solo los datos dentro de esa ventana
        df_ventana = df[df.iloc[:, 0] >= limite_inferior]
        
        # Extracción segura de datos por índice de columna de la ventana filtrada
        tiempo = df_ventana.iloc[:, 0]

        # Módulo Frontal
        crudo_f_izq = df_ventana.iloc[:, 3]
        crudo_f_der = df_ventana.iloc[:, 4]
        max_f_crudo = df_ventana.iloc[:, 5]
        kalman_f    = df_ventana.iloc[:, 12]

        # Módulo Lateral Izquierdo
        crudo_lat_izq = df_ventana.iloc[:, 6]
        kalman_lat_izq = df_ventana.iloc[:, 13]

        # Módulo Lateral Derecho
        crudo_lat_der = df_ventana.iloc[:, 7]
        kalman_lat_der = df_ventana.iloc[:, 14]

        # Limpieza de ejes
        ax1.clear()
        ax2.clear()
        ax3.clear()

        # --- SUBGRÁFICA 1: ANÁLISIS FRONTAL ---
        ax1.plot(tiempo, crudo_f_izq, label='ps7 Crudo (Front Izq)', alpha=0.3, color='orange')
        ax1.plot(tiempo, crudo_f_der, label='ps0 Crudo (Front Der)', alpha=0.3, color='cyan')
        ax1.plot(tiempo, max_f_crudo, label='Fusión Máxima Cruda', alpha=0.6, color='blue', linestyle='-.')
        ax1.plot(tiempo, kalman_f, label=r'Estimación Kalman Frontal ($\hat{d}_k$)', color='green', linewidth=2.5)
        ax1.axhline(y=120.0, color='gray', linestyle=':', label='Zona Despejada (120)')
        ax1.axhline(y=300.0, color='red', linestyle='--', label='Umbral Peligro (300)')
        ax1.set_ylim(0, 600)
        ax1.set_ylabel('Intensidad Infrarroja')
        ax1.set_title('1. Módulo Frontal: Fusión de Máximos y Filtro de Kalman')
        ax1.legend(loc='upper right', fontsize='small', ncol=3)
        ax1.grid(True, linestyle='--', alpha=0.5)

        # --- SUBGRÁFICA 2: ANÁLISIS LATERAL IZQUIERDO ---
        ax2.plot(tiempo, crudo_lat_izq, label='ps5 Crudo (Lat Izq)', alpha=0.4, color='magenta')
        ax2.plot(tiempo, kalman_lat_izq, label=r'Kalman Lateral Izq', color='indigo', linewidth=2.5)
        ax2.axhline(y=120.0, color='gray', linestyle=':')
        ax2.axhline(y=300.0, color='red', linestyle='--')
        ax2.set_ylim(0, 600)
        ax2.set_ylabel('Intensidad Infrarroja')
        ax2.set_title('2. Flanco Izquierdo: Sensor ps5')
        ax2.legend(loc='upper right', fontsize='small')
        ax2.grid(True, linestyle='--', alpha=0.5)

        # --- SUBGRÁFICA 3: ANÁLISIS LATERAL DERECHO ---
        ax3.plot(tiempo, crudo_lat_der, label='ps2 Crudo (Lat Der)', alpha=0.4, color='limegreen')
        ax3.plot(tiempo, kalman_lat_der, label=r'Kalman Lateral Der', color='darkgreen', linewidth=2.5)
        ax3.axhline(y=120.0, color='gray', linestyle=':')
        ax3.axhline(y=300.0, color='red', linestyle='--')
        ax3.set_ylim(0, 600)
        ax3.set_xlabel('Tiempo de Simulación (s)')
        ax3.set_ylabel('Intensidad Infrarroja')
        ax3.set_title('3. Flanco Derecho: Sensor ps2')
        ax3.legend(loc='upper right', fontsize='small')
        ax3.grid(True, linestyle='--', alpha=0.5)
        
        # --- AJUSTE DINÁMICO DEL EJE X ---
        # Forzamos los límites de visualización temporal para todas las gráficas (sharex=True)
        ax1.set_xlim(limite_inferior, max(5.0, tiempo_actual))
        
        plt.tight_layout()

    except Exception as e:
        print(f"Error de sincronización en tiempo real: {e}")

# Configuración del bucle de animación
ani = FuncAnimation(fig, actualizar_grafica, interval=500, cache_frame_data=False)

plt.show()
