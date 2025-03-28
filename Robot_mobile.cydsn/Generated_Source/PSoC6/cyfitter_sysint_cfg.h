/*******************************************************************************
* File Name: cyfitter_sysint_cfg.h
* 
* PSoC Creator  4.4
*
* Description:
* 
* This file is automatically generated by PSoC Creator.
*
********************************************************************************
* Copyright (c) 2007-2020 Cypress Semiconductor.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#ifndef INCLUDED_CYFITTER_SYSINT_CFG_H
#define INCLUDED_CYFITTER_SYSINT_CFG_H

#include "sysint/cy_sysint.h"

/* ARM CM4 */
#if (((__CORTEX_M == 4) && (CY_CORE_ID == 0)))
    #define UART_1_SCB_IRQ__INTC_ASSIGNED 1u
    extern const cy_stc_sysint_t UART_1_SCB_IRQ_cfg;

    #define SysInt_3__INTC_ASSIGNED 1u
    extern const cy_stc_sysint_t SysInt_3_cfg;

    #define time_out_int__INTC_ASSIGNED 1u
    extern const cy_stc_sysint_t time_out_int_cfg;

    #define BtnIsr__INTC_ASSIGNED 1u
    extern const cy_stc_sysint_t BtnIsr_cfg;

    #define SysInt_1__INTC_ASSIGNED 1u
    extern const cy_stc_sysint_t SysInt_1_cfg;

    #define SysInt_2__INTC_ASSIGNED 1u
    extern const cy_stc_sysint_t SysInt_2_cfg;

    #define encoder_L_int__INTC_ASSIGNED 1u
    extern const cy_stc_sysint_t encoder_L_int_cfg;

    #define encoder_R_int__INTC_ASSIGNED 1u
    extern const cy_stc_sysint_t encoder_R_int_cfg;

    #define ADC_1_IRQ__INTC_ASSIGNED 1u
    extern const cy_stc_sysint_t ADC_1_IRQ_cfg;

#endif /* ((__CORTEX_M == 4) && (CY_CORE_ID == 0)) */


#endif /* INCLUDED_CYFITTER_SYSINT_CFG_H */
