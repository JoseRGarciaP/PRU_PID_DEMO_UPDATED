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
 //  Tweaks to the PID parameters were implemented by Gregory Raven.

#include <stdint.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <sys_pwmss.h>
#include "resource_table_empty.h"


// Conversión "Float".
#define SHIFT    16			// antes 14

// Registros GPIO de las PRU.
// Se define el registro R30 interno de la PRU de tipo volatile.
// Escribir en el registro R30 controla los pines GPO.
// Este registro es usado para los pines GPO de las PRUs y es usado para el parpadeo del LED.
volatile register uint32_t __R30;

struct pid_data {
	// Sintoniación PID.
	short Kp_f, Ki_f, Kd_f;
	
	// Controles PID.
	short setpoint, input;
	int int_err;
	short output, last_output;
	short min_output, max_output;
};						// Estas estructuras son compartidas entre las PRUs,
						// y así se forma la comunicación entre ambas.

struct cycles_data {
	int sum, loops;
	short min, med, max;
};
												
// Estructura del bloque de memoria compartida.
struct shared_mem {
	volatile char init_flag;
	volatile char ctrl_man;				// 'a' -> automático; 'm' -> manual.
	volatile struct cycles_data cycles;
	volatile struct pid_data pid1;
	volatile struct pid_data pid2;
	volatile int reg_pwmss_ctrl;
};


/* Establecimiento de la memoria compartida */
#pragma DATA_SECTION(share_buff, ".share_buff")    // Asigna el espacio de memoria de la
                                                   // sección ".share_buff" al símbolo share_buff
volatile far struct shared_mem share_buff;         // Se define el símbolo shared_buff como una instancia de la estructura
                                                   // shared_mem, tipo volatile y far ( 16 bits superiores de la memoria)

// Parámetros del Encoder de Cuadratura.
#define TICKS_PER_REV       334 
#define SAMPLES_PER_SEC     12	// Antiguo valor: 12
#define SEC_PER_MIN         60

/*
#define LOOPS (*((volatile unsigned int *)0x4A301000))	// Defino la variable LOOP en el espacio de memoria de datos PRU RAM0.
                                                        // De esta manera la otra PRU también tendrá esta variable sin necesidad
														// de definirla en su código.
*/

/* Declaración de funciones, prototipo */
void update_pid(volatile struct pid_data* pid1, volatile struct pid_data* pid2);    // Función de actualización del PID.
void write_output(volatile struct pid_data* pid1, volatile struct pid_data* pid2);	// Función de escritura de PWMs.
void init_pid(volatile struct pid_data* pid1, volatile struct pid_data* pid2, volatile struct cycles_data* cycles);      // Función de inicialización del PID.
int get_enc_rpm1();		// Obtención de la velocidad por el encoder.
int get_enc_rpm2(); 

/*
 * main.c
*/
void main(void) {
	
	short ncycles;
	
	while (!(share_buff.init_flag == 1));		// Permiso de PRU 1 para empezar el PID
	
	init_pid(&share_buff.pid1, &share_buff.pid2, &share_buff.cycles);
	share_buff.ctrl_man = 'a';
	
	// Bucle principal.
	while(1) {
		
		// Inicio del conteo de ciclo del segmento de código.
		PRU0_CTRL.CTRL_bit.CTR_EN = 0;               // Desactivo el contador (por seguridad) y lo limpio.
		PRU0_CTRL.CYCLE_bit.CYCLECOUNT = 0xFFFFFFFF;
		PRU0_CTRL.CTRL_bit.CTR_EN = 1;               // Inicio del conteo.
		ncycles = 0;
		
		// Lee Velocidad.
		// Guarda los ciclos de escritura esperando al evento de cambio.
		if (PWMSS1.EQEP_QFLG & 0x0800) {
			PWMSS1.EQEP_QCLR |= 0x0800;
			share_buff.pid1.input = get_enc_rpm1();
		}
		if (PWMSS2.EQEP_QFLG & 0x0800) {
			PWMSS2.EQEP_QCLR |= 0x0800;
			share_buff.pid2.input = get_enc_rpm2();
		}
		
		// Comrprueba control automático para realizar el control.
		if (share_buff.ctrl_man == 'a') {
			
			// Actualiza PID.
			update_pid(&share_buff.pid1, &share_buff.pid2);
			
		}
		
		// Establece la velocidad PWM.
		write_output(&share_buff.pid1, &share_buff.pid2);
		
		// Fin del conteo.
		PRU0_CTRL.CTRL_bit.CTR_EN = 0;                // Se detiene el contador.
		ncycles = PRU0_CTRL.CYCLE_bit.CYCLECOUNT;       // Copio el número de ciclos.

		if (share_buff.cycles.sum <= 2000000000)            // Evita el desbordamiento del dato sum (unsigned int).
		{
			share_buff.cycles.sum += ncycles;
			share_buff.cycles.med = share_buff.cycles.sum / (share_buff.cycles.loops + 1);	// Le sumo 1 porque share_buff.loops se actualiza después al final del bucle.
			share_buff.cycles.loops += 1;
		}

		if (ncycles > share_buff.cycles.max) share_buff.cycles.max = ncycles;
		if (ncycles < share_buff.cycles.min) share_buff.cycles.min = ncycles;
		
	}
}


/*
 * update_pid
 */
void update_pid(volatile struct pid_data* pid1, volatile struct pid_data* pid2) {
	int output_f, p_f, d_f;
	short error, output;
	
	// PID 1.
	// Cálculo del error.
	error = (pid1->setpoint - pid1->input);

	// Cálculo de la parte Proporcional.
	p_f = (int)pid1->Kp_f * error;

	// Cálculo de la parte Integral.
	pid1->int_err += ((int)pid1->Ki_f * error) >> SHIFT;

	// Cálculo de la parte Derivativa. QUITADA.
	//d_f = (int) pid1->Kd_f * (pid1->output - pid1->last_output);

	// Suma total de la salida PID.
	output_f = p_f + pid1->int_err;
	output = output_f >> SHIFT;
	
	// Establecimieto de la salida PID, comprobación min/max de la salida.
	if (output < pid1->min_output) {
		pid1->output = pid1->min_output;
	} else if (output > pid1->max_output) {
		pid1->output = pid1->max_output;
	} else {
		pid1->output = output;
	}
	

	// PID 2.
	// Cálculo del error.
	error = (pid2->input - pid2->setpoint);

	// Cálculo de la parte Proporcional.
	p_f = (int)pid2->Kp_f * error;

	// Cálculo de la parte Integral.
	pid2->int_err += ((int)pid2->Ki_f * error) >> SHIFT;

	// Cálculo de la parte Derivativa. QUITADA.
	//d_f = (int) pid2->Kd_f * (pid2->output - pid2->last_output);

	// Suma total de la salida PID.
	output_f = p_f + pid2->int_err;
	output = output_f >> SHIFT;
	
	// Establecimieto de la salida PID, comprobación min/max de la salida.
	if (output < pid2->min_output) {
		pid2->output = pid2->min_output;
	} else if (output > pid2->max_output) {
		pid2->output = pid2->max_output;
	} else {
		pid2->output = output;
	}
	
}

/*
 * write_output
 */
void write_output(volatile struct pid_data* pid1, volatile struct pid_data* pid2) {
	
	// Salida 1.
	// Establecimiento de los sentidos de giro.
	if (pid1->output > 0) {
		PWMSS1.EPWM_CMPA = pid1->output;		// R1 := PWM.	(Hacida adelante)
		PWMSS1.EPWM_CMPB = 0;							// R2 := 0.
	} else if (pid1->output < 0){
		PWMSS1.EPWM_CMPA = 0;							// R1 := 0.		(Hacia atrás)
		PWMSS1.EPWM_CMPB = - pid1->output;	// R2 := PWM.
	} else if (pid1->output == 0) {
		PWMSS1.EPWM_CMPA = 0;							// R1 := 0;		(Parado)
		PWMSS1.EPWM_CMPB = 0;							// R2 := 0;
	}
	
	// Salida 2.
	// Establecimiento de los sentidos de giro.
	if (pid2->output > 0) {
		PWMSS2.EPWM_CMPA = pid2->output;		// L1 := PWM.	(Hacida adelante)
		PWMSS2.EPWM_CMPB = 0;							// L2 := 0.
	} else if (pid2->output < 0){
		PWMSS2.EPWM_CMPA = 0;							// L1 := 0.		(Hacia atrás)
		PWMSS2.EPWM_CMPB = - pid2->output;	// L2 := PWM.
	} else if (pid2->output == 0) {
		PWMSS2.EPWM_CMPA = 0;							// L1 := 0;		(Parado)
		PWMSS2.EPWM_CMPB = 0;							// L2 := 0;
	}
	
}

/*
 * init_pid
 */
void init_pid(volatile struct pid_data* pid1, volatile struct pid_data* pid2, volatile struct cycles_data* cycles) {
	
	// PID.
	pid1->output = 0;
	pid2->output = 0;
	
	pid1->input = 0;
	pid2->input = 0;
	
	pid1->Kp_f = 500;	// 500
	pid1->Ki_f = 200;	// 200
	pid1->Kd_f = 0;
	
	pid2->Kp_f = 500;
	pid2->Ki_f = 900;
	pid2->Kd_f = 0;
	
	pid1->int_err = 0;
	pid1->max_output = 4095; // Decimal 4095, pwm de 12 bits Es el periodo del módulo eCAP para convertir en PWM. (Máximo ciclo de trabajo)
	pid1->min_output = -4095;
	
	pid2->int_err = 0;
	pid2->max_output = 4095; // Decimal 4095. Es el periodo del módulo eCAP para convertir en PWM. (Máximo ciclo de trabajo)
	pid2->min_output = -4095;
	
	pid1->setpoint = 0;
	pid2->setpoint = 0;
	
	// Cycles.
	cycles->loops = 0;
	cycles->min = 1000;
	cycles->med = 0;
	cycles->max = 0;
	cycles->sum = 0;
}

/*
 * get_enc_rpm()
 */
int get_enc_rpm1() {
	
	int rpm;
	
	// Comprobación de errores de desbordamiento, encender el LED y reestablecer a 0 el RPM.
	if (PWMSS1.EQEP_QEPSTS &= 0x0C) {
		PWMSS1.EQEP_QEPSTS |= 0x0C;
		__R30 |= 0x04;	// bit 2 en alto para encender el led asociado.
		rpm = 0;
	} else {
		__R30 &= 0xFFFFFFFB;	// bit 2 apagado.
		rpm = ( (int) PWMSS1.EQEP_QPOSLAT * SAMPLES_PER_SEC * SEC_PER_MIN) / TICKS_PER_REV;	// Se hace un typecast int explícito
																							// para que recozca posiciones negativas.
	}
	
	return rpm;
}

int get_enc_rpm2() {
	
	int rpm;
	
	// Comprobación de errores de desbordamiento, encender el LED y reestablecer a 0 el RPM.
	if (PWMSS2.EQEP_QEPSTS &= 0x0C) {
		PWMSS2.EQEP_QEPSTS |= 0x0C;
		__R30 |= 0x08;	// bit 3 en alto para encender el led asociado.
		rpm = 0;
	} else {
		__R30 &= 0xFFFFFFF7;	// bit 3 apagado.
		rpm = ( (int) PWMSS2.EQEP_QPOSLAT * SAMPLES_PER_SEC * SEC_PER_MIN) / TICKS_PER_REV;
	}
	
	return rpm;
}

