#include "project.h"
#include "encoder.h"
#include "globals.h"
#include "math.h"

// Variables para el filtro de media móvil exponencial
float filtered_u_L = 0.0;  // Velocidad angular filtrada de la rueda izquierda
float filtered_u_R = 0.0;  // Velocidad angular filtrada de la rueda derecha
const float encoder_alpha = 0.97;  // Factor de suavizado para el filtro

// Función para calcular la velocidad de los encoders
void encoder_speed() {
    uint32_t status = Cy_SysLib_EnterCriticalSection();
    
    // Calcula la velocidad angular cruda basada en los ticks del encoder
    float raw_u_L = (tick_L / (float)Tick_max) * (1 / f);
    float raw_u_R = (tick_R / (float)Tick_max) * (1 / f);
    
    // Aplica el filtro de media móvil exponencial para suavizar las velocidades angulares
    filtered_u_L = encoder_alpha * raw_u_L + (1 - encoder_alpha) * filtered_u_L;
    filtered_u_R = encoder_alpha * raw_u_R + (1 - encoder_alpha) * filtered_u_R;
    
    // Calcula la velocidad lineal de las ruedas utilizando las velocidades angulares filtradas
    v_L = (filtered_u_L * R * 2 * M_PI);
    v_R = (filtered_u_R * R * 2 * M_PI);
    
    // Reinicia los contadores de ticks para el próximo intervalo de muestreo
    tick_L = 0;
    tick_R = 0;
    
    // Ajusta la dirección de la velocidad de las ruedas según la velocidad deseada
    if (v_L_des < 0) {
        v_L = -v_L;
    }
    if (v_R_des < 0) {
        v_R = -v_R;
    }
    
    Cy_SysLib_ExitCriticalSection(status);
}

// Manejador de interrupción para el encoder de la rueda izquierda
void encoder_L_IRQ_Handler(void) {
    // Limpia la interrupción pendiente
    NVIC_ClearPendingIRQ(encoder_L_int_cfg.intrSrc);
    // Incrementa el contador de ticks del encoder izquierdo
    ++tick_L;
}

// Manejador de interrupción para el encoder de la rueda derecha
void encoder_R_IRQ_Handler(void) {
    // Limpia la interrupción pendiente
    NVIC_ClearPendingIRQ(encoder_R_int_cfg.intrSrc);
    // Incrementa el contador de ticks del encoder derecho
    ++tick_R;
}
