# Proyecto Final: Navegación Autónoma y Planificación de Trayectorias 

## Integrantes
* Maura Gonzalez
* Guadalupe Marin
* Rodrigo Rojas
* Ricardo Toro

**Linea de desarrollo seleccionada:** Linea A

---

## Objetivo del Proyecto
Diseñar, implementar y evaluar un sistema de navegación autónoma para un robot móvil diferencial (e-puck) en Webots. El sistema integra el control cinemático, percepción sensorial mediante encoders y sensores de distancia, y la toma de decisiones basada en el algoritmo de planificación global **A*** sobre una grilla de ocupación.

---

## Descripción del Sistema

### Robot y Actuadores
*   **Modelo:** e-puck (Robot diferencial).
*   **Motores:** Motores paso a paso independientes para las ruedas izquierda y derecha.
*   **Radio de Rueda ($r$):** 0.0205 m.
*   **Distancia entre Ejes ($L$):** 0.052 m.

### Sensores Utilizados
1.  **Encoders (`left wheel sensor y right wheel sensor`):** Utilizados para la odometría y estimación de pose en tiempo real.
2.  **Sensores de Proximidad Infrarrojos (`ps0 a ps7`):** Utilizados para la detección de obstáculos y navegación reactiva.
3.  **Acelerómetro/Giroscopio:** Disponibles para mejorar la estabilidad de la orientación.

---

## Escenarios de Prueba

Se diseñaron dos entornos distintos para evaluar el desempeño del sistema:

| Escenario | Nombre del Archivo | Pose Inicial $(x, y, \phi)$ | Meta $(x, y)$ | Descripción |
| :--- | :--- | :---: | :---: | :--- |
| **Simple** | `MundoEz.wbt` | $(-0.93, 0.0, 0.0)$ | $(0.8, 0.0)$ | Entorno abierto con obstáculos fijos y ruta directa. |
| **Complejo** | `MundoHard.wbt` | $(0.04, 0.23, -1.57)$ | $(0.35, -0.35)$ | Laberinto con pasillos estrechos y múltiples giros. |

---

## Algoritmo de Navegación Global (A*)

El sistema utiliza una representación del entorno mediante una **Grilla de Ocupación** de $20 \times 20$ celdas, donde cada celda mide $0.1 \times 0.1$ metros.

### Flujo de Solución:
1.  **Mapeo:** El entorno se discretiza en una matriz 2D (0 = libre, 1 = ocupado).
2.  **Planificación:** El algoritmo A* encuentra la ruta más corta desde la celda de inicio hasta la celda meta, minimizando el costo $f(n) = g(n) + h(n)$, donde $h(n)$ es la distancia Manhattan.
3.  **Conversión:** La ruta de celdas se convierte en una lista de *waypoints* en coordenadas métricas del mundo Webots.
4.  **Seguimiento:** El robot ejecuta una secuencia de "Girar en el sitio" y "Avanzar recto" para alcanzar consecutivamente cada waypoint.

### Pseudocodigo A*:
```python
Función A*(grilla, inicio, meta):
    lista_abierta = HeapPriorityQueue(inicio)
    padres = {inicio: None}
    costo_g = {inicio: 0}
    
    Mientras lista_abierta no esté vacía:
        actual = lista_abierta.pop_menor_f()
        Si actual == meta: retornar reconstruir_ruta(padres, meta)
        
        Para cada vecino en [Norte, Sur, Este, Oeste]:
            Si vecino es libre y dentro de límites:
                nuevo_g = costo_g[actual] + 1
                Si nuevo_g < costo_g.get(vecino, inf):
                    costo_g[vecino] = nuevo_g
                    f = nuevo_g + Manhattan(vecino, meta)
                    lista_abierta.push(vecino, f)
                    padres[vecino] = actual
```

---

## Relación con Laboratorios 1 y 2

### Extensión del Laboratorio 1 (Cinemática)
Se reutilizaron las ecuaciones de control diferencial para el movimiento. El robot estima su pose global $(x, y, \phi)$ integrando las velocidades de las ruedas:
*   $\Delta s = \frac{r(\Delta\theta_{der} + \Delta\theta_{izq})}{2}$
*   $\Delta\phi = \frac{r(\Delta\theta_{der} - \Delta\theta_{izq})}{L}$

### Extensión del Laboratorio 2 (Percepción y Filtrado)
*   **Filtro EMA:** Suaviza el ruido de los sensores infrarrojos.
*   **Filtro de Kalman:** Estima la distancia real a obstáculos frontales combinando odometría y sensores.
*   **Navegación Reactiva:** Si los sensores detectan un obstáculo inesperado (no mapeado), el robot activa una rutina de evasión antes de retomar la ruta planificada.

---

## Resultados y Evaluación Experimental

### Métricas de Desempeño
Tras múltiples ejecuciones, se obtuvieron los siguientes promedios:

| Métrica | Escenario Simple | Escenario Complejo |
| :--- | :---: | :---: |
| Tiempo total hasta la meta | ~45 s | ~120 s |
| Longitud ruta planificada | 1.73 m | 2.85 m |
| Longitud trayectoria real | 1.78 m | 3.10 m |
| Error de posición final | < 0.02 m | < 0.05 m |
| Número de colisiones | 0 | 0 |

### Análisis de Trayectorias
Se utilizó el script `plot_trayectoria_compare.py` para visualizar la fidelidad del seguimiento:
*(Insertar imagen: scripts/output/comparacion_trayectoria.png)*

El error acumulado por odometría es bajo gracias al uso de giros precisos en el sitio y compensación de ángulo medio en la integración.

---

## Instrucciones para la ejecución

1.  **Clonar el repositorio.**
    ```bash
    git clone --single-branch --branch Proyectofinal  https://github.com/Ratinaxo/controlador-robotica.git
    ```
2.  **Abrir Webots** y cargar el mundo `worlds/MundoEz.wbt` o `worlds/MundoHard.wbt`.
3.  **Preparar la ruta:**
    ```bash
    cd scripts
    python plan_path.py --name laberinto1
    ```
4.  **Ejecutar simulación:** Presionar el botón *Play* en Webots. El controlador `mundo_final_controlador` cargará automáticamente el archivo JSON de la ruta.
5.  **Visualizar resultados:**
    ```bash
    python data_sensores/plot_trayectoria_compare.py --name laberinto1
    ```

---

## Video Demostrativo
(Acá deberia estar el link al video demostrativo o lo agregamos a la carpeta de controlador, como sea más conveniente)
---

## Conclusiones y Limitaciones
*   **Conclusión:** El algoritmo A* garantiza rutas óptimas en entornos discretizados, y la odometría implementada es suficiente para trayectorias de corta/mediana duración en Webots.
*   **Limitaciones:** La grilla de $0.1$ m puede ser gruesa para pasillos muy estrechos. El error odométrico aumenta en simulaciones muy largas si no hay corrección por hitos externos o SLAM.
*   **Mejoras:** Implementar una grilla de mayor resolución o utilizar algoritmos de suavizado de curvas (splines) para evitar las detenciones en cada giro.
