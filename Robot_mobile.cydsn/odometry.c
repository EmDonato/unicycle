#include "project.h"
#include "odometry.h"
#include "globals.h"
#include "math.h"

// Función para calcular la odometría del robot
void Odometry() {
    // Actualiza la posición en el eje X utilizando la velocidad promedio de las ruedas y la orientación actual
    x = x + ((v_L + v_R) / 2) * cos(theta_act) * dt;
    
    // Actualiza la posición en el eje Y utilizando la velocidad promedio de las ruedas y la orientación actual
    y = y + ((v_L + v_R) / 2) * sin(theta_act) * dt;
    
    // Actualiza el ángulo de orientación actual utilizando la diferencia de velocidad entre las ruedas
    theta_act = theta_act + ((v_R - v_L) / d) * dt;
    
    // Calcula el error como la distancia entre la posición deseada y la posición actual
    error = sqrt(pow((x_des - x), 2) + pow((y_des - y), 2));
    
    // Si el error es menor o igual a un umbral, deshabilita el control
    if (error <= 0.01) {
        enable = 0;
    }
}
