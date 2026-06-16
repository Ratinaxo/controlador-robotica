# Proyecto Final: Navegación Autónoma y Planificación de Trayectorias (Línea A)

## Integrantes
* Maura Gonzalez
* Guadalupe Marin
* Rodrigo Rojas
* Ricardo Toro

---

## 📋 Estado del Proyecto (Checklist de Avance)

### ✅ Fase 1: Configuración de Entornos en Webots
- [x] **Diseñar el Escenario 1 (Simple):** Construir un entorno reproducible en Webots con baja densidad de obstáculos fijos y una ruta directa hacia la meta (`worlds/MundoEz.wbt`).
- [x] **Diseñar el Escenario 2 (Complejo):** Crear un entorno tipo laberinto o pasillo con múltiples obstáculos, curvas cerradas, pasillos estrechos o zonas de bloqueo (`worlds/MundoHard.wbt`).
- [x] **Definir la Pose Inicial y la Meta:** Establecer y registrar las coordenadas (x,y) formales para el punto de partida y el punto de destino final en ambos mapas.

### ✅ 📐 Fase 2: Localización 2D (Extensión de Odometría)
- [x] **Implementar las Ecuaciones Cinemáticas:** Integrar en el bucle principal las ecuaciones de odometría (fórmulas 1 a 7 del enunciado) para estimar de forma continua el desplazamiento lineal instantáneo ($\Delta s$), la variación angular ($\Delta \phi$), la posición global ($x_k, y_k$) y la orientación acumulada ($\phi_k$).
- [x] **Sincronizar Encoders:** Reutilizar el bloque de lectura de los sensores de las ruedas (`left wheel sensor` y `right wheel sensor`) para alimentar la cinemática de pose global.
- [x] **Validar Telemetría:** Verificar en la consola de Webots que los valores de posición cambien coherentemente cuando el robot avance o rote.

### 🧠 Fase 3: Mapeo Discreto y Algoritmo Global
- [ ] **Representar la Grilla de Ocupación:** Traducir el mapa continuo de Webots a una matriz 2D o grafo discreto en Python, donde las celdas libres se representen con 0 y las celdas bloqueadas por obstáculos se marquen con 1.
- [ ] **Programar el Algoritmo de Planificación:** Implementar un script con el algoritmo elegido (A∗ o Dijkstra) que tome como entradas la celda inicial, la celda meta y la grilla de ocupación.
- [ ] **Generar la Ruta Óptima:** Lograr que el algoritmo retorne la secuencia ordenada de celdas libres para ir desde el inicio hasta la meta.
- [ ] **Conversión a Waypoints:** Traducir los índices discretos de la ruta calculada de vuelta a coordenadas métricas reales del mundo de Webots, generando una lista de puntos intermedios (waypoints).

### 🕹️ Fase 4: Navegación Local y Control de Movimiento
- [ ] **Controlador de Seguimiento:** Programar un algoritmo de control (por ejemplo, un control proporcional de ángulo) que ajuste de forma autónoma las velocidades de los motores para orientar y desplazar el robot consecutivamente hacia cada waypoint.
- [ ] **Fusión de Seguridad Reactiva:** Integrar los sensores de distancia frontales (`ps7`, `ps0`) y laterales (`ps5`, `ps2`) junto al Filtro de Kalman y el filtro EMA heredado del Laboratorio 2.
- [ ] **Interrupción por Obstáculos:** Configurar el sistema para que, si el robot detecta un obstáculo inesperado o sufre una desviación física, la navegación reactiva local tome el control temporal para evitar la colisión y luego reanude el seguimiento de la ruta global.
- [ ] **Rutina de Llegada:** Asegurar que el robot detecte cuando ha alcanzado el último waypoint (meta) para desacelerar y aplicar una detención absoluta y segura de los motores.

### 📊 Fase 5: Evaluación Experimental y Métricas
- [ ] **Adaptar el Archivo CSV:** Configurar el registro de datos para almacenar el tiempo de simulación y las nuevas métricas requeridas para el análisis global.
- [ ] **Medir Indicadores Cuantitativos:** Ejecutar la simulación en ambos escenarios y registrar obligatoriamente: tiempo total hasta la meta, longitud de la ruta planificada, longitud de la trayectoria real ejecutada, número de colisiones y cantidad de giros innecesarios.
- [ ] **Análisis Odométrico y Tasa de Éxito:** Estimar la acumulación del error de posición por odometría y calcular el porcentaje de ejecuciones exitosas tras realizar múltiples pruebas repetidas.

### 📝 Fase 6: Entregables del Repositorio de GitHub
- [ ] **Grabar Video Demostrativo:** Producir un video corto y enlazarlo en el repositorio.
- [ ] **Redactar el README.md (Informe Oficial):** Estructurar este archivo como el informe técnico definitivo del proyecto.

---

## 🗺️ Fase 1: Configuración de Entornos, Poses Iniciales y Metas

Los entornos han sido modelados y guardados en la carpeta `worlds/`. El robot e-puck se autogestiona con base en las coordenadas del mundo cargado:

| Escenario | Nombre del Archivo | Pose Inicial $(x_{ini}, y_{ini}, \phi_{ini})$ | Coordenada Meta $(x_{meta}, y_{meta})$ | Descripción |
| :--- | :--- | :---: | :---: | :--- |
| **Escenario 1 (Simple)** | `MundoEz.wbt` | $(-0.93\text{ m},\ 0.0\text{ m},\ 0.0\text{ rad})$ | $(0.8\text{ m},\ 0.0\text{ m})$ | Entorno abierto de $2\times 1$ metros con 13 cajas como obstáculos y paso directo. |
| **Escenario 2 (Complejo)** | `MundoHard.wbt` | $(0.04\text{ m},\ 0.23\text{ m},\ -1.5708\text{ rad})$ | $(0.35\text{ m},\ -0.35\text{ m})$ | Laberinto de pasillos estrechos estructurado mediante nodos `Wall`. |

---

## 📐 Fase 2: Localización 2D (Ecuaciones de Odometría)

Para estimar de forma continua la pose del robot diferencial ($x_k, y_k, \phi_k$), se implementó el modelo cinemático directo basado en los encoders integrados en las ruedas (`left wheel sensor` y `right wheel sensor`).

### Formulación Matemática (Fórmulas 1 a 7)

1. **Variación angular de la rueda izquierda ($\Delta\theta_{izq}$):**
   $$\Delta\theta_{izq} = \theta_{izq, k} - \theta_{izq, k-1} \tag{1}$$

2. **Variación angular de la rueda derecha ($\Delta\theta_{der}$):**
   $$\Delta\theta_{der} = \theta_{der, k} - \theta_{der, k-1} \tag{2}$$

3. **Desplazamiento lineal de la rueda izquierda ($\Delta s_{izq}$):**
   $$\Delta s_{izq} = r \cdot \Delta\theta_{izq} \tag{3}$$

4. **Desplazamiento lineal de la rueda derecha ($\Delta s_{der}$):**
   $$\Delta s_{der} = r \cdot \Delta\theta_{der} \tag{4}$$

5. **Desplazamiento lineal instantáneo del centro del robot ($\Delta s$):**
   $$\Delta s = \frac{\Delta s_{izq} + \Delta s_{der}}{2} \tag{5}$$

6. **Variación angular del robot ($\Delta\phi$):**
   $$\Delta\phi = \frac{\Delta s_{der} - \Delta s_{izq}}{L} \tag{6}$$

7. **Actualización de la pose global en 2D ($x_k, y_k, \phi_k$):**
   Usamos la aproximación Runge-Kutta de segundo orden (ángulo medio de desplazamiento) para mayor estabilidad ante curvas y rotaciones continuas:
   $$\phi_k = \phi_{k-1} + \Delta\phi \tag{7a}$$
   $$x_k = x_{k-1} + \Delta s \cdot \cos\left(\phi_{k-1} + \frac{\Delta\phi}{2}\right) \tag{7b}$$
   $$y_k = y_{k-1} + \Delta s \cdot \sin\left(\phi_{k-1} + \frac{\Delta\phi}{2}\right) \tag{7c}$$

*Donde:*
*   $r = 0.0205\text{ m}$ (Radio de la rueda).
*   $L = 0.052\text{ m}$ (Distancia entre ejes / baseline del e-puck).
*   $\phi_k$ se normaliza cíclicamente en el rango de $[-\pi, \pi]\text{ rad}$.

### Validación de Telemetría e Historial (CSV)

El bucle del controlador realiza los cálculos de odometría en cada iteración y actualiza tanto la consola como el archivo de datos:
*   **Consola de Webots:** Imprime la posición en tiempo real cada 2 segundos, por ejemplo:
    `[Telemetría 4.0s] Pose:(-0.93, 0.0) | Ori:0.0 rad (0.0°) | Kalman Frente:75.0`
*   **Archivo CSV (`data_sensores/datos_sensores.csv`):** Se adaptó para incluir tres nuevas columnas de pose calculada: `Pose X (m)`, `Pose Y (m)` y `Orientacion Phi (rad)`.

---

## 🛠️ Requisitos Previos y Ejecución
*   **Webots:** Versión R2025a.
*   **Python:** Versión 3.10 o superior.

### Instrucciones:
1. Abra Webots y cargue uno de los mapas en la carpeta `worlds/` (`MundoEz.wbt` o `MundoHard.wbt`).
2. Verifique en el panel del robot que el controlador asignado sea `lab2_controlador`.
3. Inicie la simulación. La consola mostrará la inicialización del mundo autodetectado y la telemetría dinámica de odometría en tiempo real.

---

## 🧬 Herencia del Laboratorio 2: Evasión Reactiva y Filtrado

Para asegurar la supervivencia física del robot, se mantienen los siguientes desarrollos previos integrados en la navegación local actual:
1.  **Filtro EMA (Exponential Moving Average):** Aplica suavizado en las lecturas de los sensores infrarrojos para atenuar ruido blanco y picos erráticos:
    $$z_{EMA, k} = \alpha \cdot z_k + (1-\alpha) \cdot z_{EMA, k-1} \quad (\alpha = 0.2)$$
2.  **Filtro de Kalman:** Combina la odometría de encoders con las lecturas analógicas de proximidad para predecir y corregir la distancia a obstáculos en el frente, lateral izquierdo y lateral derecho:
    *   **Predicción:** $d^-_{k} = d_{k-1} + \Delta s \cdot 1500.0$
    *   **Corrección:** $d_{k} = d^-_{k} + K_k \cdot (z_k - d^-_{k})$
3.  **Toma de Decisiones Reactiva:** Si el estimador de Kalman de proximidad frontal o lateral excede `UMBRAL_PELIGRO = 300.0`, el robot interrumpe la navegación para rotar en el sitio en sentido opuesto al obstáculo más cercano hasta que el área quede despejada (`UMBRAL_ZONA_DESPEJADA = 125.0`).
