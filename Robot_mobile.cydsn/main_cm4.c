#include "light_sensor.h"
#include "project.h"
#include "control.h"
#include "encoder.h"
#include "odometry.h"
#include "globals.h"
#include "stdio.h"
#include "HC-SR04.h"

// Buffer para la comunicación UART
char buffer[125];

// Variables para los encoders
volatile int tick_L = 0;
volatile int tick_R = 0;
int Tick_max = 20;

// Velocidades angulares
volatile float u_R = 0.0;
volatile float v_R = 0.0;
volatile float u_L = 0.0;
volatile float v_L = 0.0;
volatile float v_R_des = 0.0;
volatile float v_L_des = 0.0;
volatile float theta_des = 0.0;
volatile float Last_theta_des = 0.0;

// Variables de posición
volatile float x = 0.0;
volatile float y = 0.0;
volatile float theta_act = 0.0;
float dt = 0.2;  // Intervalo de tiempo

// Variables de control
int volatile enable = 1;
int volatile enable_ultrasonic_sensor = 1;
volatile float error = 0.0;
int nGiri = 0;  // Número de giros

// Constantes de control proporcional
float K_1 = 3.2;
float K_2 = 10;

// Velocidades deseadas
volatile float v_1_des = 0.0;
volatile float v_2_des = 0.0;

// Posiciones deseadas
volatile float x_des = 0.0;
volatile float y_des = 0.0;

// Dimensiones del robot
float R = 0.034;  // Radio de la rueda
float d = 0.135;  // Distancia entre ruedas

// Frecuencia de muestreo
float f = 0.2;

// Bandera de control
volatile bool control_flag = false;

// Variables para el sensor HC-SR04
int value_counter = 0;
float volatile distancia = 0.0;
char imp[30];
int value_echo = 0;
float barriera = 15.0;  // Distancia de barrera para el sensor
volatile int var_aus_trigger = 1;
volatile int first_echo_detected = 0;  // Variable para rastrear el primer eco
volatile float filtered_distance = 0.0;
float alpha = 0.05;  // Factor de suavizado para el filtro
volatile uint8_t process_data_flag = 0;

volatile int test_selector = 0;  // Variable para seleccionar el test

// Manejador de la interrupción del temporizador del sistema
void SysTime_out_IRQHandler(void) {
    // Limpiar la interrupción del contador 1
    Counter_1_ClearInterrupt(Counter_1_config.interruptSources);
    control_flag = true;  // Establecer la bandera para indicar que es tiempo de ejecutar el control
    // Invertir el estado de los LEDs verde y rojo
    Cy_GPIO_Inv(GREEN_LED_PORT, GREEN_LED_NUM);
    Cy_GPIO_Inv(RED_LED_PORT, RED_LED_NUM);
}

// Manejador de la interrupción del botón
void BtnIsr_handler(void) {
    uint32_t status = Cy_SysLib_EnterCriticalSection();
    // Reiniciar variables de posición y velocidad
    x = 0.0;
    y = 0.0;
    v_R = 0.0;
    v_L = 0.0;
    v_R_des = 0.0;
    v_L_des = 0.0;
    theta_des = 0.0;
    Last_theta_des = 0.0;
    theta_act = 0.0;
    error = 0.0;
    nGiri = 0;
    v_1_des = 0.0;
    v_2_des = 0.0;
    
    // Selección del test basado en el valor de test_selector
    if (test_selector == 0) {
        CyDelay(10000u);  // Esperar 10 segundos
        x_des = 1.0;
        y_des = 0.0;
        test_selector = 1;
    } else if (test_selector == 1) {
        CyDelay(10000u);  // Esperar 10 segundos
        x_des = 0.0;
        y_des = -1.0;
        test_selector = 2;
    } else if (test_selector == 2) {
        CyDelay(10000u);  // Esperar 10 segundos
        x_des = 0.0;
        y_des = 1.0;       
        test_selector = 3;
    } else if (test_selector == 3) {
        CyDelay(10000u);  // Esperar 10 segundos
        x_des = 1.0;
        y_des = -1.0;
        test_selector = 4;
    } else if (test_selector == 4) {
        CyDelay(10000u);  // Esperar 10 segundos
        x_des = 2.0;
        y_des = 0.0;
        test_selector = 0;
    }
    
    enable = 1;  // Habilitar el control
    Cy_SysLib_ExitCriticalSection(status);
}

int main(void) {
    __enable_irq(); /* Habilitar interrupciones globales. */

    // Inicializar la UART y las interrupciones
    UART_1_Start();
    Cy_SysInt_Init(&time_out_int_cfg, SysTime_out_IRQHandler);
    Cy_SysInt_Init(&encoder_L_int_cfg, encoder_L_IRQ_Handler);
    Cy_SysInt_Init(&encoder_R_int_cfg, encoder_R_IRQ_Handler);

    NVIC_ClearPendingIRQ(time_out_int_cfg.intrSrc);
    NVIC_EnableIRQ(time_out_int_cfg.intrSrc);

    NVIC_ClearPendingIRQ(encoder_L_int_cfg.intrSrc);
    NVIC_EnableIRQ(encoder_L_int_cfg.intrSrc);

    NVIC_ClearPendingIRQ(encoder_R_int_cfg.intrSrc);
    NVIC_EnableIRQ(encoder_R_int_cfg.intrSrc);

    Cy_SysInt_Init(&BtnIsr_cfg, BtnIsr_handler);
    NVIC_ClearPendingIRQ(BtnIsr_cfg.intrSrc);
    NVIC_EnableIRQ(BtnIsr_cfg.intrSrc);

    PWM_L_SetCompare0(0);  // Configurar PWM inicial para el motor izquierdo
    PWM_R_SetCompare0(0);  // Configurar PWM inicial para el motor derecho

    LightSensor_Init();  // Inicializar el sensor de luz
    Counter_2_Start();  // Iniciar el contador 2

    // Código comentado para el sensor ultrasónico
   /*
    Cy_SysInt_Init(&SysInt_1_cfg, echo_sub_Handler);
    Cy_SysInt_Init(&SysInt_2_cfg, echo_bajo_Handler);

    NVIC_ClearPendingIRQ(SysInt_1_cfg.intrSrc);
    NVIC_EnableIRQ(SysInt_1_cfg.intrSrc);

    NVIC_ClearPendingIRQ(SysInt_2_cfg.intrSrc);
    NVIC_EnableIRQ(SysInt_2_cfg.intrSrc);

    Counter_4_Enable();
    Counter_4_Start();

    Cy_SysInt_Init(&SysInt_3_cfg, SysInt_3_IRQHandler);
    NVIC_ClearPendingIRQ(SysInt_3_cfg.intrSrc);
    NVIC_EnableIRQ(SysInt_3_cfg.intrSrc);
    Counter_3_Start();
    */

    Counter_1_Start();  // Iniciar el contador 1
    PWM_L_Start();  // Iniciar el PWM para el motor izquierdo
    PWM_R_Start();  // Iniciar el PWM para el motor derecho

    for (;;) {
        // Ejecutar el control si la bandera está activa
        if (control_flag) {
            control_flag = false;  // Reiniciar la bandera
            if (enable_ultrasonic_sensor == 1) {
                uint32_t status = Cy_SysLib_EnterCriticalSection();
                encoder_speed();  // Calcular la velocidad de los encoders
                Cy_SysLib_ExitCriticalSection(status);

                // Enviar los datos de velocidad y ticks a través de UART
                snprintf(buffer, sizeof(buffer), "I TICK SONO:  %d\n la velocita destra:  %f m/s \n la velocita sinistra:  %f m/s\n", tick_L, v_R, v_L);
                Cy_SCB_UART_PutString(UART_1_HW, buffer);
                Cy_SCB_UART_PutString(UART_1_HW, "\n");

                status = Cy_SysLib_EnterCriticalSection();
                Odometry();  // Calcular la odometría
                Cy_SysLib_ExitCriticalSection(status);

                // Enviar los datos de odometría y error a través de UART
                snprintf(buffer, sizeof(buffer), "GOAL:  (%f,%f)\n X_act: %f  \n Y_act:  %f \n\n ERROR = %f \n\n", x_des, y_des, x, y, error);
                Cy_SCB_UART_PutString(UART_1_HW, buffer);
                Cy_SCB_UART_PutString(UART_1_HW, "\n");

                status = Cy_SysLib_EnterCriticalSection();
                Control_P();  // Aplicar el control proporcional
                Cy_SysLib_ExitCriticalSection(status);

                // Enviar los datos de control a través de UART
                snprintf(buffer, sizeof(buffer), "V_R_DES: %f  \n\n V_L_DES:  %f \n\n THETA_DES: %f  \n\n THETA_ACT:  %f \n\n", v_R_des, v_L_des, theta_des, theta_act);
                Cy_SCB_UART_PutString(UART_1_HW, buffer);
                Cy_SCB_UART_PutString(UART_1_HW, "\n");

                status = Cy_SysLib_EnterCriticalSection();
                set_speed(v_R_des, v_L_des);  // Establecer la velocidad deseada
                Cy_SysLib_ExitCriticalSection(status);
            }
        }

        // Código comentado para el sensor ultrasónico
        /*
        while (Cy_GPIO_Read(echo_PORT, echo_NUM) == 0) {
            Cy_GPIO_Write(trigger_PORT, trigger_NUM, 1);
            CyDelay(10u);
            Cy_GPIO_Write(trigger_PORT, trigger_NUM, 0);
            CyDelay(1);
            value_echo = Cy_GPIO_Read(echo_PORT, echo_NUM);
        }
        */
    }
}
