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

#define nPIDs	2								// Defino el número de PIDs.

/* Conversión "Float" */
#define SHIFT    0x0E							// Se desplaza hacia la derecha 0x0E posiciones
                                                // (14 decimales) para convertir el dato en punto flotante.

/* Registros GPIO de las PRU */
// Se define el registro R30 interno de la PRU de tipo volatile.
// Escribir en el registro R30 controla los pines GPO.
// Este registro es usado para los pines GPO de las PRUs y es usado para el parpadeo del LED.
volatile register uint32_t __R30;

struct pid_data {
    /* Sintoniación PID */
    int Kp_f, Ki_f, Kd_f;

    /* Controles PID */
    int setpoint;
    int int_err;
    int input, output, last_output;
    int min_output, max_output;
};												// Estas estructuras son compartidas entre las PRUs,
												// y así se forma la comunicación entre ambas.

struct pid_cycles {
	unsigned int min, med, max, sum, loops;
};
												
/* Estructura del bloque de memoria compartida */
struct shared_mem {
    volatile char init_flag;
	volatile struct pid_cycles c_pid;
    volatile struct pid_data pid[nPIDs];
};

/* Declaración de funciones, prototipo */
void update_pid(volatile struct pid_data pid[], volatile struct pid_cycles* c_pid);    // Función de actualización del PID.
void init_pid(volatile struct pid_data pid[]);      // Función de inicialización del PID.

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
    share_buff.c_pid.loops = 0;

    /* Permiso de PRU 1 para inicializarse */
	
	/* Inicializar PID */
	init_pid(share_buff.pid);
		
	while (!share_buff.init_flag == 1);		// Permiso de PRU 1 para empezar el control PID
	
    /* Bucle principal */
	while(1) {    // Modificar salida del bucle con algún comando
	
	    update_pid(share_buff.pid, &share_buff.c_pid);
		share_buff.c_pid.loops += 1;                       // Conteo del número de ciclos producidos.

	}
}

/*
 * update_pid
 */
void update_pid(volatile struct pid_data pid[], volatile struct pid_cycles* c_pid) {
    unsigned int p_f, d_f, cycles;
    int output_f, output, i;
	cycles = 0;
	
	/* Inicio del conteo de ciclo del segmento de código */
	PRU0_CTRL.CTRL_bit.CTR_EN = 0;               // Desactivo el contador y lo limpio.
	PRU0_CTRL.CYCLE_bit.CYCLECOUNT = 0xFFFFFFFF;
	PRU0_CTRL.CTRL_bit.CTR_EN = 1;               // Inicio del conteo.
	
	for (i = 0; i < nPIDs; i++)
	{
		/* Cálculo del error */											// (->) Selección de elemento con puntero.
		int error = (pid[i].input - pid[i].setpoint);

		/* Cálculo de la parte Proporcional */
		p_f = pid[i].Kp_f * error;

		/* Cálculo de la parte Integral */
		pid[i].int_err += (pid[i].Ki_f * error) >> SHIFT;

		/* Cálculo de la parte Derivativa */
		d_f = pid[i].Kd_f * (pid[i].output - pid[i].last_output);

		/* Suma total de la salida PID */
		output_f = p_f + pid[i].int_err + d_f;
		output = output_f >> SHIFT;

		/* Establecimieto de la salida PID, comprobación min/max de la salida */
		if (output < pid[i].min_output) output = pid[i].min_output;
		if (output > pid[i].max_output) output = pid[i].max_output;

		pid[i].last_output = pid[i].output;
		pid[i].output = pid[i].max_output - output;	
	}
	
	/* Fin del conteo */
	PRU0_CTRL.CTRL_bit.CTR_EN = 0;                // Se detiene el contador.
	cycles = PRU0_CTRL.CYCLE_bit.CYCLECOUNT;       // Copio el número de ciclos.

	if (c_pid->sum <= 65300)            // Evita el desbordamiento del dato sum (unsigned int).
	{
	c_pid->sum += cycles;
	c_pid->med = c_pid->sum / (c_pid->loops + 1 );			// Le sumo 1 porque shared_buff.loops se actualiza después al final del bucle.
	};

	if (cycles > c_pid->max) c_pid->max = cycles;
	if (cycles < c_pid->min) c_pid->min = cycles;
	
}

/*
 * init_pid
 */
void init_pid(volatile struct pid_data pid[]) {
    int i;
	for (i = 0; i < nPIDs; i++)
	{
		/* Se establecen los parámetros PID por defecto */
		pid[i].Kp_f = 500;
		pid[i].Ki_f = 200;
		pid[i].Kd_f = 200;

		pid[i].max_output = 0x1000; // Decimal 4096. Es el periodo del módulo eCAP para convertir en PWM. (Máximo ciclo de trabajo)
		pid[i].min_output = 0;

		pid[i].setpoint = 3000;


		pid[i].input = 0;
		pid[i].output = 0;

		pid[i].int_err = 0;

	}
}
