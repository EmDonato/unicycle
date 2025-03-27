#include "light_sensor.h"
#include "globals.h"

// Definición de la variable global
volatile int32_t sample_uV; // Variable global para almacenar el valor de la muestra en microvoltios

// Función de inicialización del sensor de luz
void LightSensor_Init(void) {
    // Inicia el ADC y configura la interrupción correspondiente
    ADC_1_StartEx(ADC_1_my_ISR);
    // Configura la máscara de fin de secuencia para el ADC
    ADC_1_SetEosMask(0);
    // Configura la máscara de límite para el ADC
    ADC_1_SetLimitMask(1);
}

// Función de interrupción para el ADC
void ADC_1_my_ISR(void) {
    // Limpia la interrupción pendiente
    //NVIC_ClearPendingIRQ(ADC_1_IRQ_cfg.intrSrc);
    
    // Inicializa el valor de la muestra a 0
    sample_uV = 0;
    
    // Obtiene el resultado crudo del ADC
    int16_t raw_counts = ADC_1_GetResult16(0);
    
    // Convierte el resultado crudo a cuentas
    int16_t counts = Cy_SAR_RawCounts2Counts(ADC_1_SAR__HW, 0, raw_counts);
    
    // Convierte las cuentas a microvoltios
    sample_uV = ADC_1_CountsTo_uVolts(0, counts);
    
    // Desactiva la habilitación (enable)
    enable = 0;
    
    // Obtiene el estado de la interrupción
    uint32_t intr_status = Cy_SAR_GetInterruptStatus(ADC_1_SAR__HW);
    
    // Limpia la interrupción de rango
    Cy_SAR_ClearRangeInterrupt(ADC_1_SAR__HW, intr_status);
}
