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

struct pid_data {
    /* Sintoniación PID */
    int Kp_f1, Ki_f1, Kd_f1;
	int Kp_f2, Ki_f2, Kd_f2;

    /* Controles PID */
    int setpoint1;
	int setpoint2;
    int int_err1;
	int int_err2;
    int input1, output1, last_output1;
	int input2, output2, last_output2;
    int min_output, max_output;
	
	/* Datos bucle PID */
	unsigned int min1, med1, max1, sum1;
	unsigned int min2, med2, max2, sum2;
	unsigned int loops;
};												// Estas estructuras son compartidas entre las PRUs,
												// y así se forma la comunicación entre ambas.
/* Estructura del bloque de memoria compartida */
struct shared_mem {
    volatile char init_flag;
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
/*
#define LOOPS (*((volatile unsigned int *)0x4A301000))   // Defino la variable LOOP en el espacio de memoria de datos PRU RAM0.
                                                         // De esta manera la otra PRU también tendrá esta variable sin necesidad
														 // de definirla en su código.
*/ // Lo hago con la variable loops de la estructura
														 
/*
 * main.c
*/
void main(void) {
    share_buff.pid.loops = 0;

    /* Permiso de PRU 1 para inicializarse */
	
	/* Inicializar PID */
	init_pid(&share_buff.pid);
		
	while (!share_buff.init_flag == 1);		// Permiso de PRU 1 para empezar el control PID
	
    /* Bucle principal */
	while(1) {    // Modificar salida del bucle con algún comando
	
	    update_pid(&share_buff.pid);
		share_buff.pid.loops += 1;                       // Conteo del número de ciclos producidos.

	}
}

/*
 * update_pid
 */
void update_pid(volatile struct pid_data* pid) {
    unsigned int p_f, d_f, cycles;
    int output_f, output, error;
	cycles = 0;
	
	// PID 1
	/* Inicio del conteo de ciclo del segmento de código */
	PRU0_CTRL.CTRL_bit.CTR_EN = 0;               // Desactivo el contador y lo limpio.
	PRU0_CTRL.CYCLE_bit.CYCLECOUNT = 0xFFFFFFFF;
	PRU0_CTRL.CTRL_bit.CTR_EN = 1;               // Inicio del conteo.

	/* Cálculo del error */											// (->) Selección de elemento con puntero.
	error = (pid->input1 - pid->setpoint1);

	/* Cálculo de la parte Proporcional */
	p_f = pid->Kp_f1 * error;

	/* Cálculo de la parte Integral */
	pid->int_err1 += (pid->Ki_f1 * error) >> SHIFT;

	/* Cálculo de la parte Derivativa */
	d_f = pid->Kd_f1 * (pid->output1 - pid->last_output1);

	/* Suma total de la salida PID */
	output_f = p_f + pid->int_err1 + d_f;
	output = output_f >> SHIFT;

	/* Establecimieto de la salida PID, comprobación min/max de la salida */
	if (output < pid->min_output) output = pid->min_output;
	if (output > pid->max_output) output = pid->max_output;

	pid->last_output1 = pid->output1;
	pid->output1 = pid->max_output - output;

	/* Fin del conteo */
	PRU0_CTRL.CTRL_bit.CTR_EN = 0;                // Se detiene el contador.
	cycles = PRU0_CTRL.CYCLE_bit.CYCLECOUNT;       // Copio el número de ciclos.

	if (pid->sum1 <= 65300)            // Evita el desbordamiento del dato sum (unsigned int).
	{
	pid->sum1 += cycles;
	pid->med1 = pid->sum1 / (pid->loops + 1 );			// Le sumo 1 porque shared_buff.loops se actualiza después al final del bucle.
	};

	if (cycles > pid->max1) pid->max1 = cycles;
	if (cycles < pid->min1) pid->min1 = cycles;

	// PID 2
	cycles = 0;
	/* Inicio del conteo de ciclo del segmento de código */
	PRU0_CTRL.CTRL_bit.CTR_EN = 0;               // Desactivo el contador y lo limpio.
	PRU0_CTRL.CYCLE_bit.CYCLECOUNT = 0xFFFFFFFF;
	PRU0_CTRL.CTRL_bit.CTR_EN = 1;               // Inicio del conteo.

	/* Cálculo del error */											// (->) Selección de elemento con puntero.
	error = (pid->input2 - pid->setpoint2);

	/* Cálculo de la parte Proporcional */
	p_f = pid->Kp_f2 * error;

	/* Cálculo de la parte Integral */
	pid->int_err2 += (pid->Ki_f2 * error) >> SHIFT;

	/* Cálculo de la parte Derivativa */
	d_f = pid->Kd_f2 * (pid->output2 - pid->last_output2);

	/* Suma total de la salida PID */
	output_f = p_f + pid->int_err2 + d_f;
	output = output_f >> SHIFT;

	/* Establecimieto de la salida PID, comprobación min/max de la salida */
	if (output < pid->min_output2) output = pid->min_output2;
	if (output > pid->max_output2) output = pid->max_output2;

	pid->last_output2 = pid->output2;
	pid->output2 = pid->max_output2 - output;

	/* Fin del conteo */
	PRU0_CTRL.CTRL_bit.CTR_EN = 0;                // Se detiene el contador.
	cycles = PRU0_CTRL.CYCLE_bit.CYCLECOUNT;       // Copio el número de ciclos.

	if (pid->sum2 <= 65300)            // Evita el desbordamiento del dato sum (unsigned int).
	{
	pid->sum2 += cycles;
	pid->med2 = pid->sum2 / (pid->loops + 1 );			// Le sumo 1 porque shared_buff.loops se actualiza después al final del bucle.
	};

	if (cycles > pid->max2) pid->max2 = cycles;
	if (cycles < pid->min2) pid->min2 = cycles;

}

/*
 * init_pid
 */
void init_pid(volatile struct pid_data* pid1, volatile struct pid_data* pid2) {

	pid->max_output = 0x1000; // Decimal 4096. Es el periodo del módulo eCAP para convertir en PWM. (Máximo ciclo de trabajo)
	pid->min_output = 0;
	
	// PID 1
	/* Se establecen los parámetros PID por defecto */
	pid->Kp_f1 = 500;
	pid->Ki_f1 = 200;
	pid->Kd_f1 = 200;

	pid->setpoint1 = 3000;

	pid->input1 = 0;
	pid->output1 = 0;

	pid->setpoint1 = 0;
	pid->int_err1 = 0;
	
	pid->sum1 = 0;
	pid->med1 = 0;
	pid->max1 = 0;
	pid->min1 = 65535;     // máximo valor unsigned int.
	
	// PID 2
	/* Se establecen los parámetros PID por defecto */
	pid->Kp_f2 = 500;
	pid->Ki_f2 = 200;
	pid->Kd_f2 = 200;

	pid->setpoint2 = 3000;


	pid->input2 = 0;
	pid->output2 = 0;

	pid->setpoint2 = 0;
	pid->int_err2 = 0;
	
	pid->sum2 = 0;
	pid->med2 = 0;
	pid->max2 = 0;
	pid->min2 = 65535;     // máximo valor unsigned int.

}
