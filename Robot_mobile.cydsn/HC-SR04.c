#include "HC-SR04.h"
#include "project.h"
#include "stdio.h"
#include "math.h"
#include "control.h"
#include "globals.h"

// Variable global para rastrear la llamada a echo_bajo_Handler
volatile bool echo_bajo_called = true;

// Manejador de interrupción para el pulso bajo del eco
void echo_sub_Handler() {
    // Limpia la interrupción pendiente
    NVIC_ClearPendingIRQ(SysInt_1_cfg.intrSrc);
    
    // Entra en una sección crítica
    uint32_t status = Cy_SysLib_EnterCriticalSection();
    
    // Indica que echo_bajo_Handler no ha sido llamado aún
    echo_bajo_called = false;
    
    // Configura el contador a su valor máximo
    Counter_4_SetCounter(65535u);
    
    // Sale de la sección crítica
    Cy_SysLib_ExitCriticalSection(status);
}

// Manejador de interrupción para el pulso bajo del eco
void echo_bajo_Handler() {
    // Limpia la interrupción pendiente
    NVIC_ClearPendingIRQ(SysInt_2_cfg.intrSrc);
    
    // Entra en una sección crítica
    uint32_t status = Cy_SysLib_EnterCriticalSection();
    
    // Calcula el valor del contador
    value_counter = 65535u - Counter_4_GetCounter();
    
    // Calcula la distancia utilizando la fórmula adecuada para el sensor HC-SR04
    distancia = value_counter / 58.0;
    
    // Aplica un filtro de suavizado para reducir el ruido en la medición
    filtered_distance = alpha * distancia + (1 - alpha) * filtered_distance;

    // Salida de depuración para la distancia sin filtrar
    //sprintf(imp, "Raw Distance: %f cm\n", distancia);
    //Cy_SCB_UART_PutString(UART_1_HW, imp);
    
    // Salida de depuración para la distancia filtrada
    //sprintf(imp, "Filtered Distance: %f cm\n", filtered_distance);
    //Cy_SCB_UART_PutString(UART_1_HW, imp);
    
    // Si la distancia filtrada es menor que la barrera y mayor que 3 cm
    if (filtered_distance <= barriera && filtered_distance > 3.0) {
        // Indica que el objeto está demasiado cerca
        Cy_SCB_UART_PutString(UART_1_HW, "\n\nTROPPO VICINO\n\n");
        // Detiene el sensor ultrasónico
        enable_ultrasonic_sensor = 0;
        // Salida de depuración para la distancia filtrada
        //sprintf(imp, "%f cm\n", filtered_distance);
        //Cy_SCB_UART_PutString(UART_1_HW, imp);
    } else {
        // Imprime la distancia filtrada
        sprintf(imp, "%f cm\n", filtered_distance);
        Cy_SCB_UART_PutString(UART_1_HW, imp);
        // Habilita el sensor ultrasónico
        enable_ultrasonic_sensor = 1;
    }

    // Reinicia el contador 3 para la próxima medición
    Counter_3_Start();

    // Indica que echo_bajo_Handler ha sido llamado
    echo_bajo_called = true;
    
    // Sale de la sección crítica
    Cy_SysLib_ExitCriticalSection(status);
}

// Manejador de la interrupción del sistema para el contador 3
void SysInt_3_IRQHandler(void) {
    // Limpia la interrupción del contador 3
    Counter_3_ClearInterrupt(Counter_3_config.interruptSources);
    
    // Entra en una sección crítica
    uint32_t status = Cy_SysLib_EnterCriticalSection();
    
    // Verifica el estado de la señal de eco
    if (Cy_GPIO_Read(echo_PORT, echo_NUM) == 0) {
        // Asegúrate de que el trigger se envíe solo después de que echo_bajo_Handler haya sido llamado
        //if (echo_bajo_called) {
        //    if (!first_echo_detected) {
                // Envía el trigger hasta que se detecte el primer eco
        //        Cy_GPIO_Write(trigger_PORT, trigger_NUM, 1);
        //        CyDelay(10u); // Pulso de 10 microsegundos
        //        Cy_GPIO_Write(trigger_PORT, trigger_NUM, 0);
        //        Counter_3_SetCounter(1u);
        //    } else {
                // Alterna el trigger para el sensor ultrasónico
                if (var_aus_trigger == 1) {
                    Cy_GPIO_Write(trigger_PORT, trigger_NUM, 1);
                    var_aus_trigger = 0;
                    Counter_3_SetCounter(1u);
                } else {
                    Cy_GPIO_Write(trigger_PORT, trigger_NUM, 0);
                    var_aus_trigger = 1;
                    Counter_3_SetCounter(19u);
                }
        //    }
        //}
    }
    // Sale de la sección crítica
    Cy_SysLib_ExitCriticalSection(status);
    
    //else {
    //    first_echo_detected = 1; // Establece la bandera cuando se detecta el primer eco
    //}
}
