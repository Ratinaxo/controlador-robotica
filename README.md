# Laboratorio 1: Simulación de un Robot Móvil Diferencial en Webots

## Descripción del Laboratorio
El repositorio contiene el código fuente correspondiente al controlador de un robot móvil de tracción diferencial (modelo e-puck v2 de Gctronic) operado dentro del entorno de simulación Webots. El objetivo principal de este laboratorio es implementar y analizar modelos cinemáticos bidimensionales mediante la programación de actuadores y la lectura de sensores integrados al e-puck.

## Requisitos Previos
* [Webots](https://cyberbotics.com/) (versión R2025a).
* [Python](https://www.python.org/) (versión 3.14)
* [Git](https://git-scm.com/) (opcional)
## Cómo ejecutar la simulación en Webots
Para replicar los experimentos y visualizar las trayectorias del robot:

1.  **Clonar o descargar el repositorio:**
    Descarga el repositorio en tu máquina utilizando el git o el método de preferencia:
    `git clone https://github.com/Ratinaxo/controlador-robotica.git`

2.  **Crear el terreno:**
    Abre Webots, dirigete a `File` -> `New` -> `New World File...` y sigue el wizard de creación de mundo, ingresa un nombre para el archivo y agrega un área rectangular para que el robot tenga suelo.

3. **Agregar el robot:**
    Haz click derecho en el tablero, selecciona `Add New` y busca el robot e-puck, `PROTO nodes (Webots Projects)` -> `robots` -> `gctronic` -> `e-puck` -> `Add`, se creará el robot `E-Puck "e-puck"` dentro del árbol de escena (Scene Tree) de Webots.

4.  **Crear un nuevo controlador:**
    Dirigete a `File` -> `New` -> `New Robot Controller...` y sigue el wizard para crear un controlador en python.
    Mueve el contenido de nuestro controlador `controlador_python.py` dentro del controlador recién creado.

5.  **Vincular el controlador:**
    En el árbol de escena, busca el robot `E-Puck "e-puck"` y expande el nodo, busca el campo `controller`, haz clic izquierdo en él y selecciona `Select...` para buscar nuestro controlador personalizado. Elije el archivo de nuestro script.

7.  **Ejecutar:**
    Presiona el botón de reproducción en la interfaz de Webots. Dentro de la consola del simulador se imprimirá la telemetría en tiempo real (distancias, estados y coordenadas).

## Resultados Obtenidos
La ejecución del controlador demostró que a partir de las matemáticas de la cinemática diferencial aplicada nuestro entorno, arrojando los siguientes resultados comprobables:

* **Validación de Ecuaciones Cinemáticas:** 
    - Se confirmó que al igualar las velocidades de las ruedas (v<sub>r</sub> = v<sub>l</sub>) el radio de giro tiende a infinito (movimiento recto).
    - Al establecer velocidades opuestas (v<sub>r</sub> = -v<sub>l</sub>), el centro instantáneo de rotación se posiciona exactamente en el centro del eje del robot, permitiendo giros sobre su propio eje vertical sin desplazamiento traslacional nulo.
    - Las diferencias de velocidad (v<sub>r</sub> != v<sub>l</sub>) generaron radios de curvatura predecibles y proporcionales.
