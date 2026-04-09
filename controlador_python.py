from controller import Robot
import math
import random

RADIO_RUEDA = 0.02
DISTANCIA_RUEDAS = 0.0555
def aplicar_velocidades(motor_izq, motor_der, v_izq, v_der):
    vel_maxima = 6.28
    motor_izq.setVelocity(max(min(v_izq, vel_maxima), -vel_maxima))
    motor_der.setVelocity(max(min(v_der, vel_maxima), -vel_maxima))

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    motor_izq = robot.getDevice('left wheel motor')
    motor_der = robot.getDevice('right wheel motor')
    motor_izq.setPosition(float('inf'))
    motor_der.setPosition(float('inf'))

    izq_sensor = robot.getDevice('left wheel sensor')
    der_sensor = robot.getDevice('right wheel sensor')
    izq_sensor.enable(timestep)
    der_sensor.enable(timestep)

    # Odometría y Estados
    pose_x, pose_y, pose_theta = 0.0, 0.0, 0.0
    robot.step(timestep)
    enc_izq_prev, enc_der_prev = izq_sensor.getValue(), der_sensor.getValue()

    paso_actual = 1
    dist_acum = 0.0
    giro_acum = 0.0
    ruido = random.uniform(-0.0005, 0.0005)

    # Contador para las 4 caras del cuadrado
    lados_cuadrado = 0

    while robot.step(timestep) != -1:
        t = robot.getTime()

        # 1. Actualización de Odometría + ruido
        izq_act, der_act = izq_sensor.getValue(), der_sensor.getValue()
        ds = ((der_act - enc_der_prev) * (1 + ruido) + (izq_act - enc_izq_prev) * (1 + ruido)) * RADIO_RUEDA / 2.0
        dth = ((der_act - enc_der_prev) * (1 + ruido) - (izq_act - enc_izq_prev) * (1 + ruido)) * RADIO_RUEDA / DISTANCIA_RUEDAS

        pose_x += ds * math.cos(pose_theta + dth/2.0)
        pose_y += ds * math.sin(pose_theta + dth/2.0)
        pose_theta += dth
        dist_acum += abs(ds)
        giro_acum += abs(dth)

        enc_izq_prev, enc_der_prev = izq_act, der_act

        # 2. Secuencia de Movimientos
        if paso_actual == 1: # Linea Recta vr = vl
            estado = "1. Linea Recta (0.4m)"
            if dist_acum < 0.4: aplicar_velocidades(motor_izq, motor_der, 4, 4)
            else: paso_actual, dist_acum, giro_acum = 2, 0, 0

        elif paso_actual == 2: # Giro 180 vr = -vl
            estado = "2. Giro 180 (Media Vuelta)"
            if giro_acum < math.pi: aplicar_velocidades(motor_izq, motor_der, -2, 2)
            else: paso_actual, dist_acum, giro_acum = 3, 0, 0

        elif paso_actual == 3: # Linea Recta (Regreso) vr = vl
            estado = "3. Linea Recta (0.4m)"
            if dist_acum < 0.4: aplicar_velocidades(motor_izq, motor_der, 4, 4)
            else: paso_actual, dist_acum, giro_acum = 4, 0, 0

        elif paso_actual == 4: # Giro 180 vr = -vl
            estado = "4. Giro 180 (Orientacion Original)"
            if giro_acum < math.pi: aplicar_velocidades(motor_izq, motor_der, -2, 2)
            else: paso_actual, dist_acum, giro_acum = 5, 0, 0

        elif paso_actual == 5: # Movimientos Curvos (Círculo) vr != vl
            estado = "5. Circulo Completo"
            if giro_acum < 2 * math.pi: aplicar_velocidades(motor_izq, motor_der, 2, 5)
            else: paso_actual, dist_acum, giro_acum = 6, 0, 0

        elif paso_actual == 6: # Cuadrado: Lado
            estado = f"6. Cuadrado: Lado {lados_cuadrado + 1}"
            if dist_acum < 0.3: aplicar_velocidades(motor_izq, motor_der, 4, 4)
            else: paso_actual, dist_acum, giro_acum = 7, 0, 0

        elif paso_actual == 7: # Cuadrado: Giro 90
            estado = f"7. Cuadrado: Giro Esquina {lados_cuadrado + 1}"
            if giro_acum < math.pi/2: aplicar_velocidades(motor_izq, motor_der, -2, 2)
            else:
                lados_cuadrado += 1
                dist_acum, giro_acum = 0, 0
                paso_actual = 6 if lados_cuadrado < 4 else 8

        else:
            estado = "Secuencia Completada"
            aplicar_velocidades(motor_izq, motor_der, 0, 0)

        # Telemetría
        if int(t * 100) % 10 == 0:
            print(f"[{t:.1f}s] {estado} | X:{pose_x:.5f} Y:{pose_y:.5f} Th:{pose_theta:.5f}")

if __name__ == "__main__":
    main()
