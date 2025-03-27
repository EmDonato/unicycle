#include "project.h"
#include "control.h"
#include "globals.h"
#include "math.h"

// Función para establecer la dirección de la velocidad de las ruedas
void _setSpeed_direction(float speed_c_L, float speed_c_R) {
    // Configura la dirección de la rueda izquierda
    if (speed_c_L >= 0) {
        // Mueve hacia adelante
        Cy_GPIO_Write(forward_L_PORT, forward_L_NUM, 1);
        Cy_GPIO_Write(back_L_PORT, back_L_NUM, 0);
    } else {
        // Mueve hacia atrás
        Cy_GPIO_Write(forward_L_PORT, forward_L_NUM, 0);
        Cy_GPIO_Write(back_L_PORT, back_L_NUM, 1);
    }

    // Configura la dirección de la rueda derecha
    if (speed_c_R >= 0) {
        // Mueve hacia adelante
        Cy_GPIO_Write(forward_R_PORT, forward_R_NUM, 1);
        Cy_GPIO_Write(back_R_PORT, back_R_NUM, 0);
    } else {
        // Mueve hacia atrás
        Cy_GPIO_Write(forward_R_PORT, forward_R_NUM, 0);
        Cy_GPIO_Write(back_R_PORT, back_R_NUM, 1);
    }
}

// Función para calcular la velocidad en base a la velocidad deseada
int _calcola_velocita(float velocita) {
    int sign = 0;
    int valore;

    // Determina el signo de la velocidad
    if (velocita >= 0) {
        sign = 1;
    } else {
        sign = -1;
        velocita *= sign;
    }

    // Calcula el valor de la velocidad basado en rangos predefinidos
    if (velocita <= 0.1068) {
        valore = 0;
    } else if (velocita <= 0.2136) {
        valore = (int)ceil((velocita + 0.1068) / 0.01068);
    } else if (velocita <= 0.32046) {
        valore = (int)ceil((velocita + 0.42756) / 0.021372);
    } else if (velocita <= 0.42726) {
        valore = (int)ceil((velocita - 0.22701) / 0.00267);
    } else if (velocita <= 0.48066) {
        valore = (int)ceil((velocita - 0.26706) / 0.002136);
        valore = (valore > 100) ? 100 : valore;
    } else {
        valore = 100;
    }

    return valore;
}

// Función hash para PWM
uint16_t _hash_function_PWM(int x) {
    uint16_t hash_value;

    // Asigna valores de PWM basado en la entrada x
    if (x <= 20) {
        hash_value = 0;
    } else if (x > 100) {
        hash_value = 551;
    } else {
        hash_value = (uint16_t)ceil((551.0 * x) / 100.0);
    }

    return hash_value;
}

// Función para establecer la velocidad de las ruedas
void set_speed(float v_R_des, float v_L_des) {
    int level_R = _calcola_velocita(v_R_des);
    int level_L = _calcola_velocita(v_L_des);

    uint32_t status = Cy_SysLib_EnterCriticalSection();
    _setSpeed_direction(v_L_des, v_R_des);
    PWM_L_SetCompare0(_hash_function_PWM(level_L));
    PWM_R_SetCompare0(_hash_function_PWM(level_R));
    Cy_SysLib_ExitCriticalSection(status);
}

// Función de control proporcional
void Control_P() {
    if (enable == 1) {
        // Calcula la velocidad deseada basada en la posición y orientación actuales
        v_1_des = K_1 * ((x_des - x) * cos(theta_act) + (y_des - y) * sin(theta_act));
        theta_des = atan2(y_des - y, x_des - x) + nGiri * 2 * M_PI;
        Last_theta_des = theta_des;

        // Ajusta theta_des para mantener la continuidad angular
        if (fabs(theta_des + 2 * M_PI - theta_act) < fabs(theta_des - theta_act)) {
            theta_des = theta_des + 2 * M_PI;
            nGiri += 1;
        } else if (fabs(theta_des - 2 * M_PI - theta_act) < fabs(theta_des - theta_act)) {
            theta_des = theta_des - 2 * M_PI;
            nGiri -= 1;
        }

        // Si el robot está cerca de la posición deseada, mantener el último theta_des
        if (fabs(y_des - y) <= 0.05 && fabs(x_des - x) <= 0.05) {
            theta_des = Last_theta_des;
        }

        // Calcula la velocidad angular deseada
        v_2_des = K_2 * (theta_des - theta_act);

        // Calcula las velocidades deseadas para las ruedas izquierda y derecha
        v_L_des = v_1_des - (d / 2) * v_2_des;
        v_R_des = v_1_des + (d / 2) * v_2_des;
    } else {
        // Si el control no está habilitado, establece las velocidades deseadas a 0
        v_L_des = 0;
        v_R_des = 0;
    }
}
