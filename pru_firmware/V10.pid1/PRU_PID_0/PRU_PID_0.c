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

struct cycles_data {
	unsigned int min, med, max, sum, loops;
};
												
/* Estructura del bloque de memoria compartida */
struct shared_mem {
    volatile char init_flag;
	volatile struct cycles_data cycles;
    volatile struct pid_data pid1;
};

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
	
	while (!(share_buff.init_flag == 1));		// Permiso de PRU 1 para empezar el control PID
	
    share_buff.cycles.loops = 0;
	share_buff.cycles.min = 65535;
	share_buff.cycles.med = 0;
	share_buff.cycles.max = 0;
	share_buff.cycles.sum = 0;
	
	share_buff.pid1.Kp_f = 500;
	share_buff.pid1.Ki_f = 200;
	share_buff.pid1.Kd_f = 200;
	share_buff.pid1.int_err = 0;
	
	share_buff.pid1.max_output = 0x1000; // Decimal 4096. Es el periodo del módulo eCAP para convertir en PWM. (Máximo ciclo de trabajo)
	share_buff.pid1.min_output = 0;
	
	share_buff.pid1.setpoint = 3000;
	
	share_buff.pid1.input = 0;
	share_buff.pid1.output = 0;
	
    /* Bucle principal */
	while(1) {    // Modificar salida del bucle con algún comando
		
		unsigned int p_f, d_f, ncycles;
		int output_f, output, error;
		ncycles = 0;
		
		/* Inicio del conteo de ciclo del segmento de código */
		PRU0_CTRL.CTRL_bit.CTR_EN = 0;               // Desactivo el contador y lo limpio.
		PRU0_CTRL.CYCLE_bit.CYCLECOUNT = 0xFFFFFFFF;
		PRU0_CTRL.CTRL_bit.CTR_EN = 1;               // Inicio del conteo.
		
		/* PID 1 */
		/* Cálculo del error */											// (->) Selección de elemento con puntero.
		error = (share_buff.pid1.input - share_buff.pid1.setpoint);
		
		/* Cálculo de la parte Proporcional */
		p_f = share_buff.pid1.Kp_f * error;
		
		/* Cálculo de la parte Integral */
		share_buff.pid1.int_err += (share_buff.pid1.Ki_f * error) >> SHIFT;
		
		/* Cálculo de la parte Derivativa */
		d_f = share_buff.pid1.Kd_f * (share_buff.pid1.output - share_buff.pid1.last_output);
		
		/* Suma total de la salida PID */
		output_f = p_f + share_buff.pid1.int_err + d_f;
		output = output_f >> SHIFT;
		
		/* Establecimieto de la salida PID, comprobación min/max de la salida */
		if (output < share_buff.pid1.min_output) output = share_buff.pid1.min_output;
		if (output > share_buff.pid1.max_output) output = share_buff.pid1.max_output;
		
		share_buff.pid1.last_output = share_buff.pid1.output;
		share_buff.pid1.output = share_buff.pid1.max_output - output;
		
		/* Fin del conteo */
		PRU0_CTRL.CTRL_bit.CTR_EN = 0;                // Se detiene el contador.
		ncycles = PRU0_CTRL.CYCLE_bit.CYCLECOUNT;       // Copio el número de ciclos.
		
		if (share_buff.cycles.sum <= 65300)            // Evita el desbordamiento del dato sum (unsigned int).
		{
		share_buff.cycles.sum += ncycles;
		share_buff.cycles.med = share_buff.cycles.sum / (share_buff.cycles.loops + 1 );			// Le sumo 1 porque shared_buff.loops se actualiza después al final del bucle.
		};
		
		if (ncycles > share_buff.cycles.max) share_buff.cycles.max = ncycles;
		if (ncycles < share_buff.cycles.min) share_buff.cycles.min = ncycles;

		share_buff.cycles.loops += 1;                       // Conteo del número de ciclos producidos.
	}
}
