# Laboratorio 2: Navegación Reactiva con Filtrado y Fusión de Sensores en Webots

## Integrantes
* Maura Gonzalez
* Guadalupe Marin
* Rodrigo Rojas
* Ricardo Toro
---

## Requisitos Previos
* [Webots](https://cyberbotics.com/) (versión R2025a).
* [Python](https://www.python.org/) (versión 3.14)
* [Git](https://git-scm.com/) (opcional)


## Instrucciones para ejecutar la simulación

### Pasos

1. Clonar el repositorio:
   ```bash
   git clone https://github.com/Ratinaxo/controlador-robotica.git
   cd controlador-robotica
   ```

2. Abrir Webots y cargar el mundo:
    - `File > Open World` y seleccionar uno de los mapas `.wbt` dentro del directorio `worlds/` del repositorio.
        -   MundoEz.wbt es un mundo "fácil" donde el entorno son únicamente cajas.
        -   MundoHard.wbt es un mundo "dificil" donde el entorno es un laberinto .
        
3. Asignar el controlador al robot:
   - Seleccionar el e-puck en la escena.
   - En el panel de propiedades, verificar que el controlador seleccionado sea `lab2_controlador.py`.

4. Ejecutar la simulación con el botón `play` de Webots.

    -    Antes de ejecutar la simulación, podemos ir al directorio `data_sensores/` y correr el script de python `plotter_realtime.py`, con esto podremos visualizar en tiempo real los datos crudos y filtrados de los sensores del e-puck.

5. Durante la ejecución, el archivo `datos_sensores.csv` se irá llenando con datos de los sensores se será almacenado en en el directorio `data_sensores/`.

6. Una vez finalizada la simulación podemos, podemos ir al directorio `data_sensores/` y visualizar los datos de los sensores con el script de python `plotter_estatico.py`.

## Objetivo

Implementar un sistema básico de navegación reactiva en Webots para un robot móvil diferencial, utilizando sensores de distancia y encoders de rueda, aplicando filtrado sobre las mediciones y empleando un filtro de Kalman para estimar la distancia frontal a obstáculos y mejorar la toma de decisiones.

---

## Robot y sensores utilizados

Se utilizó el robot **e-puck** de Webots, un robot diferencial con dos ruedas motrices independientes.

| Componente | Dispositivo Webots | Descripción |
|---|---|---|
| Motor izquierdo | `left wheel motor` | Rueda motriz izquierda |
| Motor derecho | `right wheel motor` | Rueda motriz derecha |
| Sensor frontal izq. | `ps7` | Sensor de proximidad frontal izquierdo |
| Sensor frontal der. | `ps0` | Sensor de proximidad frontal derecho |
| Sensor lateral izq. | `ps5` | Sensor de proximidad lateral izquierdo |
| Sensor lateral der. | `ps2` | Sensor de proximidad lateral derecho |
| Encoder izquierdo | `left wheel sensor` | Posición angular rueda izq. (rad) |
| Encoder derecho | `right wheel sensor` | Posición angular rueda der. (rad) |

**Radio de rueda:** `r = 0.0205 m`

Los sensores de proximidad del e-puck devuelven valores en un rango aproximado de `[0, 4095]`, donde **mayor valor indica mayor cercanía al obstáculo**. Se normalizan a `[0, 1]` para mantener unidades consistentes con el filtro de Kalman.

---

## Frecuencia de muestreo

El paso de simulación se obtiene directamente de Webots mediante `robot.getBasicTimeStep()`.  
Para el e-puck, el valor típico es `64 ms`, lo que da:

```
Ts = 0.064 s
fs = 1 / Ts ≈ 15.625 Hz
```

La frecuencia exacta se imprime al inicio de la simulación y depende de la configuración del mundo en Webots.

---

## Descripción de la solución implementada

### 1. Lectura de encoders y estimación de avance

Los encoders proporcionan la posición angular acumulada de cada rueda en radianes. En cada paso de simulación se calcula el desplazamiento diferencial:

```
Δθ_izq = θ_izq(k) - θ_izq(k-1)
Δθ_der = θ_der(k) - θ_der(k-1)
```

El desplazamiento lineal de cada rueda se obtiene con `s = r·θ`, y el avance estimado del robot es el promedio de ambas:

```
Δd_k = (s_izq + s_der) / 2
```

### 2. Lectura y normalización de sensores frontales

Se promedian las lecturas crudas de los dos sensores frontales y se normaliza el resultado:

```
z_k = ((ps7 + ps0) / 2) / 4095
```

Esto produce un valor en `[0, 1]` donde `1` representa un obstáculo a máxima proximidad.

### 3. Filtro simple (EMA — Promedio Móvil Exponencial)

Se aplica un filtro exponencial sobre `z_k` para reducir el ruido de alta frecuencia:

```
z_k_filtrado = α · z_k + (1 - α) · z_k_filtrado_anterior
```

Con `α = 0.2`, el filtro es relativamente suave: da mayor peso al historial pasado y reacciona lentamente a cambios bruscos, lo que reduce lecturas falsas.

### 4. Filtro de Kalman para estimación de distancia frontal

El filtro de Kalman combina la información de movimiento (encoders) con la medición del entorno (sensores frontales) para obtener una estimación más robusta de la proximidad frontal `d_k`.

#### Etapa de Predicción

A partir del avance estimado con encoders, se actualiza la estimación de proximidad (al avanzar, el robot se acerca al obstáculo):

```
d̂⁻_k = d̂_{k-1} + Δd_k_norm
P⁻_k  = P_{k-1} + Q
```

Donde `Q = 0.001` es el ruido del proceso (confianza en los encoders) y `P` es la covarianza de la estimación.

#### Etapa de Corrección

La predicción se corrige usando la medición del sensor frontal `z_k`:

```
K_k  = P⁻_k / (P⁻_k + R)
d̂_k  = d̂⁻_k + K_k · (z_k - d̂⁻_k)
P_k  = (1 - K_k) · P⁻_k
```

Donde `R = 0.05` es el ruido de medición (confianza en los sensores).

**Interpretación de la ganancia K_k:**
- Si `R` es grande (sensor ruidoso) → `K_k` pequeño → el filtro confía más en la predicción.
- Si `P⁻_k` es grande (predicción incierta) → `K_k` grande → el filtro confía más en la medición.

### 5. Lógica de navegación reactiva

La decisión de movimiento se toma comparando `d̂_k` (estimación Kalman normalizada) con un umbral de seguridad `UMBRAL_OBSTACULO = 0.30`:

```
SI d̂_k > UMBRAL_OBSTACULO:
    SI lateral_izq >= lateral_der:
        → GIRAR A LA DERECHA  (obstáculo más cerca por la izquierda)
    SINO:
        → GIRAR A LA IZQUIERDA (obstáculo más cerca por la derecha)
SINO:
    → AVANZAR RECTO
```

Los sensores laterales (`ps5` y `ps2`) determinan la dirección del giro cuando hay un obstáculo al frente, favoreciendo el lado con más espacio libre.

---

## Registro de datos

Todos los datos se almacenan en `datos_sensores.csv` con las siguientes columnas:

| Columna | Descripción |
|---|---|
| Tiempo (s) | Tiempo de simulación en segundos |
| Avance Estimado (m) | Δd_k calculado con encoders |
| Frontal Crudo z_k (norm) | Promedio frontal normalizado [0,1] |
| Frontal Filtrado (norm) | Señal EMA aplicada sobre z_k |
| Kalman d_k (norm) | Estimación fusionada Kalman [0,1] |
| Lateral Izq (raw) | Lectura cruda sensor ps5 |
| Lateral Der (raw) | Lectura cruda sensor ps2 |
| Accion | AVANCE / GIRO_DER / GIRO_IZQ |

---

## Gráficos de señales

*(Incluir aquí los gráficos generados a partir del CSV: señal cruda vs. filtrada vs. Kalman, avance de encoders, y acción tomada en el tiempo)*

Ejemplo de análisis esperado:
- La señal cruda `z_k` presenta picos y ruido ante obstáculos.
- La señal filtrada EMA es más suave pero tiene retardo.
- La estimación Kalman converge más rápido y es más estable que la EMA ante cambios reales.

---

## Escenarios de prueba

### Escenario 1 — Entorno simple (pocos obstáculos)
- Una o dos cajas en el camino del robot.
- Se analiza si el robot detecta el obstáculo a tiempo, gira correctamente y retoma el avance.

### Escenario 2 — Entorno complejo (pasillo o laberinto)
- Múltiples obstáculos o paredes laterales cercanas.
- Se analiza la cantidad de giros, la estabilidad del movimiento y si el robot colisiona.

---

## Análisis y conclusiones

*(Completar con los resultados observados en la simulación)*

**Aspectos a analizar:**
- Comparación entre medición cruda, filtrada y estimación Kalman ante diferentes situaciones.
- Efectividad del umbral elegido: ¿reacciona a tiempo? ¿gira demasiado pronto?
- Comportamiento del filtro EMA vs. Kalman: el EMA reacciona más lento por su naturaleza promediadora, mientras que Kalman incorpora el modelo de movimiento y converge más rápido.
- Sensibilidad al parámetro `R`: valores más bajos hacen que Kalman confíe más en los sensores (más reactivo pero más ruidoso); valores más altos lo hacen confiar más en los encoders (más estable pero más lento).

---
