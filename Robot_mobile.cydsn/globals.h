#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

// Buffers para la comunicación UART
extern char buffer[125];

// Variables para los encoders
extern volatile int tick_L; // Contador de ticks del encoder izquierdo
extern volatile int tick_R; // Contador de ticks del encoder derecho
extern int Tick_max; // Valor máximo de ticks

// Velocidades angulares
extern volatile float u_R; // Velocidad angular actual de la rueda derecha
extern volatile float v_R; // Velocidad lineal actual de la rueda derecha
extern volatile float u_L; // Velocidad angular actual de la rueda izquierda
extern volatile float v_L; // Velocidad lineal actual de la rueda izquierda
extern volatile float v_R_des; // Velocidad lineal deseada para la rueda derecha
extern volatile float v_L_des; // Velocidad lineal deseada para la rueda izquierda
extern volatile float theta_des; // Ángulo deseado
extern volatile float Last_theta_des; // Último ángulo deseado

// Variables de posición
extern volatile float x; // Posición actual en el eje X
extern volatile float y; // Posición actual en el eje Y
extern volatile float theta_act; // Ángulo actual
extern float dt; // Intervalo de tiempo

// Variables de control
extern volatile int notify_level; // Nivel de notificación
extern volatile int enable; // Bandera para habilitar el control
extern volatile int enable_ultrasonic_sensor; // Bandera para habilitar el sensor ultrasónico
extern volatile float error; // Error actual en el control
extern int nGiri; // Número de giros

// Constantes de control proporcional
extern float K_1; // Constante proporcional 1
extern float K_2; // Constante proporcional 2

// Velocidades deseadas
extern volatile float v_1_des; // Velocidad deseada 1
extern volatile float v_2_des; // Velocidad deseada 2

// Posiciones deseadas
extern volatile float x_des; // Posición deseada en el eje X
extern volatile float y_des; // Posición deseada en el eje Y

// Dimensiones del robot
extern float R; // Radio de la rueda
extern float d; // Distancia entre las ruedas

// Frecuencia de muestreo
extern float f; // Frecuencia de muestreo

// Variables de objetivo
extern int n_target; // Número de objetivo
extern float x_goal; // Posición objetivo en el eje X
extern float y_goal; // Posición objetivo en el eje Y

// Variables para el sensor ultrasónico HC-SR04
extern int value_counter; // Valor del contador
extern volatile float distancia; // Distancia medida por el sensor
extern char imp[30]; // Buffer para imprimir mensajes
extern int value_echo; // Valor del eco
extern float barriera; // Umbral de distancia para la barrera
extern volatile int var_aus_trigger; // Variable auxiliar para el trigger
extern volatile int first_echo_detected; // Variable para rastrear el primer eco
extern volatile float filtered_distance; // Distancia filtrada
extern float alpha; // Factor de suavizado para el filtro
extern volatile uint8_t process_data_flag; // Bandera para procesar datos

#endif // GLOBALS_H
