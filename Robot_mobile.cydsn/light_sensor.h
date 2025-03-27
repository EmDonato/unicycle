#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include "project.h"

// Dichiarazione della variabile globale per il campione in microvolt

// Funzione di inizializzazione del sensore di luce
void LightSensor_Init(void);

// Dichiarazione dell'interrupt service routine (ISR)
void ADC_1_my_ISR(void);


#endif // LIGHT_SENSOR_H