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
 //  This version was revised by José R. García.

#include <stdint.h>
#include <limits.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <sys_pwmss.h>
#include "resource_table_empty.h"


// Número de PIDs
#define NPID	2			// actualmente 2 PIDs.

struct pid_data {
	// Sintoniación PID.
	short Kp, Ki;
	
	// Controles PID.
	int setpoint;
	float input;
	float int_err;
	short output;
	short min_output, max_output;
};						// Estas estructuras son compartidas entre las PRUs,
						// y así se forma la comunicación entre ambas.

struct cycles_data {
	float sumX, sumX2, loops;
	short min, med, max, var;
};
												
// Estructura del bloque de memoria compartida.
struct shared_mem {
	volatile char init_flag;
	volatile int control;				// 1 -> automático; 0 -> manual.
	volatile int numc;				// numero de ciclos constantes PID.
	volatile float tc;					// tiempo de ciclo constante PID.
	volatile struct cycles_data cycles;
	volatile struct pid_data pid[NPID];
};


/* Establecimiento de la memoria compartida */
#pragma DATA_SECTION(share_buff, ".share_buff")    // Asigna el espacio de memoria de la
                                                   // sección ".share_buff" al símbolo share_buff
volatile far struct shared_mem share_buff;         // Se define el símbolo shared_buff como una instancia de la estructura
                                                   // shared_mem, tipo volatile y far ( 16 bits superiores de la memoria)

// Parámetros del Encoder de Cuadratura.
#define TICKS_PER_REV       1336 // 4x334
#define SAMPLES_PER_SEC     250	// Antiguo valor: 12
#define SEC_PER_MIN         60


/* Declaración de funciones, prototipo */
void fdelay(short cycles, short numc);		// Función de espera.
void update_pid(volatile struct pid_data pid[], volatile float tc);    // Función de actualización del PID.
void write_output(volatile struct pid_data pid[]);	// Función de escritura de PWMs.
void init_pid();      // Función de inicialización del PID.
float get_enc_rpm1();		// Obtención de la velocidad por el encoder.
float get_enc_rpm2();

/*
 * main.c
*/
void main(void) {
	
	short cycles;
	
	
	while (!(share_buff.init_flag == 1));		// Permiso de PRU 1 para empezar el PID
	
	init_pid();
	share_buff.control = 1;
	
	PRU0_CTRL.CTRL_bit.CTR_EN = 0;               // Desactivo el contador (por seguridad) y lo limpio.
	PRU0_CTRL.CYCLE_bit.CYCLECOUNT = 0x0;
	
	// Bucle principal.
	while(1) {
		
		// Inicio del conteo de ciclo del segmento de código.

		PRU0_CTRL.CTRL_bit.CTR_EN = 1;               // Inicio del conteo.
		cycles = 0;
		
		// Lee Velocidad.
		share_buff.pid[0].input = get_enc_rpm1();
		share_buff.pid[1].input = get_enc_rpm2();
		
		// Comrprueba control automático para realizar el control.
		if (share_buff.control == 1) {
			
			// Actualiza PID.
			update_pid(share_buff.pid, share_buff.tc);
			
		}
		
		// Establece la velocidad PWM.
		write_output(share_buff.pid);
		
		// Fin del conteo.
		PRU0_CTRL.CTRL_bit.CTR_EN = 0;                // Se detiene el contador.
		cycles = PRU0_CTRL.CYCLE_bit.CYCLECOUNT;      // Copio el número de ciclos.
		PRU0_CTRL.CYCLE_bit.CYCLECOUNT = 0x0;		// Borro registro.
		

		if (share_buff.cycles.sumX < 3e+38)            // Evita el desbordamiento del dato sumX (float).
		{
			share_buff.cycles.sumX += cycles;
			share_buff.cycles.loops += 1;
			share_buff.cycles.med = share_buff.cycles.sumX / share_buff.cycles.loops;
			
			if (share_buff.cycles.sumX2 < 3e+38)            // Evita el desbordamiento del dato sumX2 (float), ocurrirá antes que el de sumX.
			{
				share_buff.cycles.sumX2 += cycles * cycles;
				share_buff.cycles.var = (share_buff.cycles.sumX2 / share_buff.cycles.loops) - ((float)share_buff.cycles.med * share_buff.cycles.med);
			}
			
		}
		
		
		
		if (cycles > share_buff.cycles.max) share_buff.cycles.max = cycles;
		if (cycles < share_buff.cycles.min) share_buff.cycles.min = cycles;

		fdelay(cycles,share_buff.numc); // Tiempo de espera para mantener el tempo de ciclo PID constante.
	}
}

/*
 * fdelay
 */
void fdelay(short cycles, short numc){
	short i, delay;
	delay = numc - cycles;
	for (i = 0; i < delay; i++){
		
		__delay_cycles(1);
		
	}
	
}

/*
 * update_pid
 */
void update_pid(volatile struct pid_data pid[], volatile float tc) {
	float error, p, output;
	short i;
	
	for (i = 0; i < NPID; i++) {
		
		// Cálculo del error.
		error = (pid[i].setpoint - pid[i].input);

		// Cálculo de la parte Proporcional.
		p = pid[i].Kp * error;

		// Cálculo de la parte Integral.
		pid[i].int_err += (pid[i].Ki * error * tc);
		
		if (pid[i].int_err < pid[i].min_output) {
			pid[i].int_err = pid[i].min_output;
		} else if (pid[i].int_err > pid[i].max_output) {
			pid[i].int_err = pid[i].max_output;
		}

		// Suma total de la salida PID.
		output = p + pid[i].int_err;
		
		// Establecimieto de la salida PID, comprobación min/max de la salida.
		if ((short)output < pid[i].min_output) {
			pid[i].output = pid[i].min_output;
		} else if ((short)output > pid[i].max_output) {
			pid[i].output = pid[i].max_output;
		} else {
			pid[i].output = (short)output;
		}
	}
}

/*
 * write_output
 */
void write_output(volatile struct pid_data pid[]) {
	
	short output0, output1;
	output0 = pid[0].output;
	output1 = pid[1].output;
	
	// Comprobacion rango de la salida.
	if (output0 < pid[0].min_output) {
		output0 = pid[0].min_output;
	} else if (output0 > pid[0].max_output) {
		output0 = pid[0].max_output;
	}
	
	if (output1 < pid[1].min_output) {
		output1 = pid[1].min_output;
	} else if (output1 > pid[1].max_output) {
		output1 = pid[1].max_output;
	}
	
	// Establecimiento de los sentidos de giro.
	if (output0 > 0) {
		PWMSS1.EPWM_CMPA = output0;		// Hacida adelante
		PWMSS1.EPWM_CMPB = 0;
	} else if (output0 < 0){
		PWMSS1.EPWM_CMPA = 0;		// Hacia atrás
		PWMSS1.EPWM_CMPB = - output0;
	} else if (output0 == 0) {
		PWMSS1.EPWM_CMPA = 0;			// Parado
		PWMSS1.EPWM_CMPB = 0;
	}
	
	if (output1 > 0) {
		PWMSS2.EPWM_CMPA = output1;		// Hacida adelante
		PWMSS2.EPWM_CMPB = 0;
	} else if (output1 < 0){
		PWMSS2.EPWM_CMPA = 0;		// Hacia atrás
		PWMSS2.EPWM_CMPB = - output1;
	} else if (output1 == 0) {
		PWMSS2.EPWM_CMPA = 0;			// Parado
		PWMSS2.EPWM_CMPB = 0;
	}
}

/*
 * init_pid
 */
void init_pid() {
	
	short i;
	// PID.
	
	for (i = 0; i < NPID; i++) {
		
		share_buff.pid[i].output = 0;
		share_buff.pid[i].input = 0;
		
		share_buff.pid[i].int_err = 0;
		share_buff.pid[i].max_output = 4095; // Decimal 4095. Es el periodo del módulo ePWM ( a 100MHz) para convertir en PWM. (Máximo ciclo de trabajo)
		share_buff.pid[i].min_output = -4095;
		
		share_buff.pid[i].setpoint = 0;
	}
	
	// Fuera del bucle porque pueden ser diferentes.
	share_buff.pid[0].Kp = 5;
	share_buff.pid[0].Ki = 10;
	
	share_buff.pid[1].Kp = 5;
	share_buff.pid[1].Ki = 10;
	
	// Periodo.
	share_buff.numc = 2000000;
	share_buff.tc = (float)share_buff.numc * 5e-9;
	
	// Ciclos.
	share_buff.cycles.loops = 0;
	share_buff.cycles.min = SHRT_MAX;
	share_buff.cycles.med = 0;
	share_buff.cycles.var = 0;
	share_buff.cycles.max = 0;
	share_buff.cycles.sumX = 0;
	share_buff.cycles.sumX2 = 0;
}

/*
 * get_enc_rpm()
 */
float get_enc_rpm1() {
	
	float rpm;
	
	// Comprobación de errores de desbordamiento, encender el LED y reestablecer a 0 el RPM.
	if (PWMSS1.EQEP_QEPSTS &= 0x0C) {
		PWMSS1.EQEP_QEPSTS |= 0x0C;
		rpm = 0;
	} else {

		rpm = (int)PWMSS1.EQEP_QPOSLAT * SAMPLES_PER_SEC * SEC_PER_MIN / (float)TICKS_PER_REV;
	}
	
	return rpm;
}

float get_enc_rpm2() {
	
	float rpm;
	
	// Comprobación de errores de desbordamiento, encender el LED y reestablecer a 0 el RPM.
	if (PWMSS2.EQEP_QEPSTS &= 0x0C) {
		PWMSS2.EQEP_QEPSTS |= 0x0C;
		rpm = 0;
	} else {
		rpm = (int)PWMSS2.EQEP_QPOSLAT * SAMPLES_PER_SEC * SEC_PER_MIN / (float)TICKS_PER_REV;
	}
	
	return rpm;
}

