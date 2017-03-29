/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/ 
 *  
 *  
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 * 
 * 	* Redistributions of source code must retain the above copyright 
 * 	  notice, this list of conditions and the following disclaimer.
 * 
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	  notice, this list of conditions and the following disclaimer in the 
 * 	  documentation and/or other materials provided with the   
 * 	  distribution.
 * 
 * 	* Neither the name of Texas Instruments Incorporated nor the names of
 * 	  its contributors may be used to endorse or promote products derived
 * 	  from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 //  Tweaks to the PID parameters were implemented by Gregory Raven.

#include <stdint.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include "resource_table_empty.h"

/* Registros GPIO de las PRU */
// Se define el registro R30 interno de la PRU de tipo volatile.
// Escribir en el registro R30 controla los pines GPO.
// Este registro es usado para los pines GPO de las PRUs y es usado para el parpadeo del LED.
volatile register uint32_t __R30;

/* Estructura de datos del PID compartida - asegurar que ambas estructuras coinciden con PRU 1  */
struct pid_data {
    /* Sintoniación PID */
    int Kp_f, Ki_f, Kd_f;

    /* Controles PID */
    int setpoint;
    int int_err;
    int input, output, last_output;
    int min_output, max_output;
	int min, med, max, sum;
};												// Estas estructuras son compartidas entre las PRUs,
												// y así se forma la comunicación entre ambas.
/* Estructura del bloque de memoria compartida */
struct shared_mem {
    volatile char init_flag;
    volatile unsigned int pwm_out;
    volatile int enc_rpm;
    volatile struct pid_data pid;
};

/* Conversión "Float" */
#define SHIFT    0x0E							// Se desplaza hacia la derecha 0x0E posiciones
                                                // (14 decimales) para convertir el dato en punto flotante.

/* Declaración de funciones, prototipo */
void update_pid(volatile struct pid_data* pid);    // Función de actualización del PID.
void init_pid(volatile struct pid_data* pid);      // Función de inicialización del PID.

/* Establecimiento de la memoria compartida */
#pragma DATA_SECTION(share_buff, ".share_buff")    // Asigna el espacio de memoria de la
                                                   // sección ".share_buff" al símbolo share_buff
volatile far struct shared_mem share_buff;         // Se define el símbolo shared_buff como una instancia de la estructura
                                                   // shared_mem, tipo volatile y far ( 16 bits superiores de la memoria)

#define LOOPS (*((volatile unsigned int *)0x4A301000))   // Defino la variable LOOP en el espacio de memoria de datos PRU RAM0.
                                                         // De esta manera la otra PRU también tendrá esta variable sin necesidad
														 // de definirla en su código.

/*
 * main.c
 */
void main(void) {
    LOOPS = 0;
    //  Parpadea el LED para mostrar que la PRU0 está activa.
    //  Se usa Universal UIO para cambiar un GPIO a pruout
    //  para habilitarlo.
    __R30 = 0x0000;                    // Todos los pines GPO a nivel bajo. (Se configurará el que esté habilitado "pruout")
    __delay_cycles(10000000);
    __R30 = 0xFFFF;                    // Todos los pines GPO a nivel alto.
    __delay_cycles(10000000);
    __R30 = 0x0000;
    __delay_cycles(10000000);

    /* Permiso de PRU 1 para inicializarse */
    while (!share_buff.init_flag == 1);
    __R30 = 0xFFFF;  //  Encender el LED si la inicialización empieza.

    /* Inicializar PID */
    init_pid(&share_buff.pid);

    /* Se establecen los parámetros PID por defecto */
    share_buff.pid.Kp_f    = 500;
    share_buff.pid.Ki_f    = 200;
    share_buff.pid.Kd_f    = 200;

    share_buff.pid.max_output = 0x1000; // Decimal 4096.  Es el periodo del módulo eCAP para convertir en PWM. (Máximo ciclo de trabajo)
    share_buff.pid.min_output = 0;

    share_buff.pid.setpoint = 3000;

    /* Bucle principal */
	while(1) {
	    update_pid(&share_buff.pid);
	    LOOPS += 1;                       // Conteo del número de ciclos producidos.
	}
}

/*
 * update_pid
 */
void update_pid(volatile struct pid_data* pid) {
    unsigned int p_f, d_f;
    int output_f, output, cycles;
	cycles = 0;
	
	/* Inicio del conteo de ciclo del segmento de código */
	PRU0_CTRL.CTRL_bit.CTRL_EN = 0;               // Desactivo el contador y lo limpio.
	PRU0_CTRL.CYCLE_bit.CYCLECOUNT = 0xFFFFFFFF;
	PRU0_CTRL.CTRL_bit.CTRL_EN = 1;               // Inicio del conteo.
	
    /* Cálculo del error */											// (->) Selección de elemento con puntero.
    int error = (pid->input - pid->setpoint);

    /* Cálculo de la parte Proporcional */
    p_f = pid->Kp_f * error;

    /* Cálculo de la parte Integral */
    pid->int_err += (pid->Ki_f * error) >> SHIFT;

    /* Cálculo de la parte Derivativa */
    d_f = pid->Kd_f * (pid->output - pid->last_output);

    /* Suma total de la salida PID */
    output_f = p_f + pid->int_err + d_f;
    output = output_f >> SHIFT;

    /* Establecimieto de la salida PID, comprobación min/max de la salida */
    if (output < pid->min_output) output = pid->min_output;
    if (output > pid->max_output) output = pid->max_output;

    pid->last_output = pid->output;
    pid->output = pid->max_output - output;
	
	/* Fin del conteo */
	PRU0_CTRL.CTRL_bit.CTRL_EN = 0;                // Se detiene el contador.
	cycles = PRU0_CTRL.CYCLE_bit.CYCLECOUNT;       // Copio el número de ciclos.
	
	pid->sum += cycles;
	
	pid->med = pid->sum / LOOPS;
	
	if (cycles > pid->max) pid->max = cycles;
	if (cycles < pid->min) pid->min = cycles;	

}

/*
 * init_pid
 */
void init_pid(volatile struct pid_data* pid) {
    pid->input = 0;
    pid->output = 0;

    pid->setpoint = 0;
    pid->int_err = 0;
	
	pid->sum = 0;
	pid->med = 0;
	pid->max = 0;
	pid->min = 65536;     // número grande aleatorio, que no se cree que se supere.
}
