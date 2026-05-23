import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

# Definir el nombre del archivo
archivo_csv = 'data_sensores/datos_sensores.csv'

# Verificar si el archivo existe antes de intentar graficar
if not os.path.exists(archivo_csv):
    print(f"Error: No se encontro el archivo '{archivo_csv}'.")
    print("Asegurate de ejecutar la simulacion en Webots primero para generarlo.")
    sys.exit(1)

# 1. Carga de datos completos
df = pd.read_csv(archivo_csv)

# Limpieza de espacios en los nombres de las cabeceras
df.columns = df.columns.str.strip()

# Validacion de estructura para evitar errores de indice
if df.shape[1] < 15:
    print(f"Error: El archivo CSV tiene {df.shape[1]} columnas, pero se esperaban 15.")
    print("Asegurate de usar la ultima version del controlador de Webots.")
    sys.exit(1)

# 2. Extraccion segura de datos por posicion de columna (iloc)
tiempo = df.iloc[:, 0]

# Frente
crudo_f_izq = df.iloc[:, 3]
crudo_f_der = df.iloc[:, 4]
max_f_crudo = df.iloc[:, 5]
kalman_f = df.iloc[:, 12]

# Lateral Izquierdo
crudo_lat_izq = df.iloc[:, 6]
ema_lat_izq = df.iloc[:, 10]
kalman_lat_izq = df.iloc[:, 13]

# Lateral Derecho
crudo_lat_der = df.iloc[:, 7]
ema_lat_der = df.iloc[:, 11]
kalman_lat_der = df.iloc[:, 14]

# 3. Configuracion de la figura con 3 subgraficas verticales
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 14), sharex=True)
fig.canvas.manager.set_window_title('Analisis Estatico de Sensores e-puck')

# --- SUBGRAFICA 1: ANALISIS FRONTAL ---
ax1.plot(tiempo, crudo_f_izq, label='ps7 Crudo (Front Izq)', alpha=0.3, color='orange')
ax1.plot(tiempo, crudo_f_der, label='ps0 Crudo (Front Der)', alpha=0.3, color='cyan')
ax1.plot(tiempo, max_f_crudo, label='Maximo Crudo Central', alpha=0.5, color='blue', linestyle='-.')
ax1.plot(tiempo, kalman_f, label=r'Estimacion Kalman Frontal ($\hat{d}_k$)', color='green', linewidth=2.0)

ax1.axhline(y=120.0, color='gray', linestyle=':', label='Zona Despejada (120)')
ax1.axhline(y=300.0, color='red', linestyle='--', label='Umbral Peligro (300)')
ax1.set_ylim(0, 600)
ax1.set_xlim(tiempo.iloc[0], tiempo.iloc[-1])
ax1.set_ylabel('Intensidad Infrarroja')
ax1.set_title('1. Modulo Frontal: Fusion de Maximos y Filtro de Kalman')
ax1.legend(loc='upper right', fontsize='small', ncol=3)
ax1.grid(True, linestyle='--', alpha=0.5)

# --- SUBGRAFICA 2: ANALISIS LATERAL IZQUIERDO ---
ax2.plot(tiempo, crudo_lat_izq, label='ps5 Crudo (Lat Izq)', alpha=0.4, color='magenta')
ax2.plot(tiempo, ema_lat_izq, label='Filtro EMA ps5', alpha=0.7, color='purple', linestyle='--')
ax2.plot(tiempo, kalman_lat_izq, label=r'Kalman Lateral Izq', color='indigo', linewidth=2.0)

ax2.axhline(y=120.0, color='gray', linestyle=':')
ax2.axhline(y=300.0, color='red', linestyle='--')
ax2.set_ylim(0, 600)
ax2.set_ylabel('Intensidad Infrarroja')
ax2.set_title('2. Flanco Izquierdo: Sensor ps5')
ax2.legend(loc='upper right', fontsize='small')
ax2.grid(True, linestyle='--', alpha=0.5)

# --- SUBGRAFICA 3: ANALISIS LATERAL DERECHO ---
ax3.plot(tiempo, crudo_lat_der, label='ps2 Crudo (Lat Der)', alpha=0.4, color='limegreen')
ax3.plot(tiempo, ema_lat_der, label='Filtro EMA ps2', alpha=0.7, color='olive', linestyle='--')
ax3.plot(tiempo, kalman_lat_der, label=r'Kalman Lateral Der', color='darkgreen', linewidth=2.0)

ax3.axhline(y=120.0, color='gray', linestyle=':')
ax3.axhline(y=300.0, color='red', linestyle='--')
ax3.set_ylim(0, 600)
ax3.set_xlabel('Tiempo de Simulacion (s)')
ax3.set_ylabel('Intensidad Infrarroja')
ax3.set_title('3. Flanco Derecho: Sensor ps2')
ax3.legend(loc='upper right', fontsize='small')
ax3.grid(True, linestyle='--', alpha=0.5)

# 4. Renderizado Final y Exportacion
plt.tight_layout()
plt.savefig('data_sensores/reporte_final_sensores.png', dpi=300, bbox_inches='tight')
print("Grafica generada con exito y guardada como 'reporte_final_sensores.png'")
plt.show()
