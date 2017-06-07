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
#include <limits.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <sys_pwmss.h>
#include "resource_table_empty.h"


// Conversión "Float".
//#define SHIFT    16			// antes 14

// Número de PIDs
#define NPID	2			// actualmente 2 PIDs.

// Registros GPIO de las PRU.
// Se define el registro R30 interno de la PRU de tipo volatile.
// Escribir en el registro R30 controla los pines GPO.
// Este registro es usado para los pines GPO de las PRUs y es usado para el parpadeo del LED.
volatile register uint32_t __R30;

struct pid_data {
	// Sintoniación PID.
	short Kp, Ki;
	
	// Controles PID.
	int setpoint;
	float input;
	float int_err;
	short output, last_output;
	short min_output, max_output;
};						// Estas estructuras son compartidas entre las PRUs,
						// y así se forma la comunicación entre ambas.

struct cycles_data {
	int sum, loops;
	short min, med, max, lenght;
};
												
// Estructura del bloque de memoria compartida.
struct shared_mem {
	volatile char init_flag;
	volatile char control;				// 'a' -> automático; 'm' -> manual.
	volatile struct cycles_data cycles;
	volatile struct pid_data pid[NPID];
};


/* Establecimiento de la memoria compartida */
#pragma DATA_SECTION(share_buff, ".share_buff")    // Asigna el espacio de memoria de la
                                                   // sección ".share_buff" al símbolo share_buff
volatile far struct shared_mem share_buff;         // Se define el símbolo shared_buff como una instancia de la estructura
                                                   // shared_mem, tipo volatile y far ( 16 bits superiores de la memoria)

// Parámetros del Encoder de Cuadratura.
#define TICKS_PER_REV       334 // es 4x 334 PREGUNTAR SI HAY QUE CAMBIARLO
#define SAMPLES_PER_SEC     24420	// Antiguo valor: 12
#define SEC_PER_MIN         60

/*
#define LOOPS (*((volatile unsigned int *)0x4A301000))	// Defino la variable LOOP en el espacio de memoria de datos PRU RAM0.
                                                        // De esta manera la otra PRU también tendrá esta variable sin necesidad
														// de definirla en su código.
*/

/* Declaración de funciones, prototipo */
void fdelay(short cycles);		// Función de espera.
void update_pid(volatile struct pid_data pid[]);    // Función de actualización del PID.
void write_output(volatile struct pid_data pid[]);	// Función de escritura de PWMs.
void init_pid(volatile struct pid_data pid[], volatile struct cycles_data* cycles);      // Función de inicialización del PID.
float get_enc_rpm1();		// Obtención de la velocidad por el encoder.
float get_enc_rpm2(); 

/*
 * main.c
*/
void main(void) {
	
	short ncycles;
	int delay;
	
	while (!(share_buff.init_flag == 1));		// Permiso de PRU 1 para empezar el PID
	
	init_pid(share_buff.pid, &share_buff.cycles);
	share_buff.control = 'a';
	
	PRU0_CTRL.CTRL_bit.CTR_EN = 0;               // Desactivo el contador (por seguridad) y lo limpio.
	PRU0_CTRL.CYCLE_bit.CYCLECOUNT = 0x0;
	
	// Bucle principal.
	while(1) {
		
		// Inicio del conteo de ciclo del segmento de código.

		PRU0_CTRL.CTRL_bit.CTR_EN = 1;               // Inicio del conteo.
		ncycles = 0;
		
		// Lee Velocidad.
		share_buff.pid[0].input = get_enc_rpm1();
		share_buff.pid[1].input = get_enc_rpm2();
		
		// Comrprueba control automático para realizar el control.
		if (share_buff.control == 'a') {
			
			// Actualiza PID.
			update_pid(share_buff.pid);
			
		}
		
		// Establece la velocidad PWM.
		write_output(share_buff.pid);
		
		// Fin del conteo.
		PRU0_CTRL.CTRL_bit.CTR_EN = 0;                // Se detiene el contador.
		ncycles = PRU0_CTRL.CYCLE_bit.CYCLECOUNT;      // Copio el número de ciclos.
		PRU0_CTRL.CYCLE_bit.CYCLECOUNT = 0x0;		// Borro registro.
		

		if (share_buff.cycles.sum <= 2000000000)            // Evita el desbordamiento del dato sum (int).
		{
			share_buff.cycles.sum += ncycles;
			share_buff.cycles.med = share_buff.cycles.sum / (share_buff.cycles.loops + 1);	// Le sumo 1 porque share_buff.loops se actualiza después al final del bucle.
			share_buff.cycles.loops += 1;
		}

		if (ncycles > share_buff.cycles.max) share_buff.cycles.max = ncycles;
		if (ncycles < share_buff.cycles.min) share_buff.cycles.min = ncycles;

		fdelay(ncycles);
	}
}

/*
 * fdelay
 */
void fdelay(short cycles){
	short i, delay;
	delay = share_buff.cycles.lenght - cycles;
	for (i = 0; i < delay; i++){
		
		__delay_cycles(1);
		
	}
	
}

/*
 * update_pid
 */
void update_pid(volatile struct pid_data pid[]) {
	float error, p;
	short output, i;
	
	for (i = 0; i < NPID; i++) {
		
		// Cálculo del error.
		error = (pid[i].setpoint - pid[i].input);

		// Cálculo de la parte Proporcional.
		p = pid[i].Kp * error;

		// Cálculo de la parte Integral.
		pid[i].int_err += (pid[i].Ki * error * share_buff.cycles.lenght);

		// Suma total de la salida PID.
		output = p + pid[i].int_err;
		
		// Establecimieto de la salida PID, comprobación min/max de la salida.
		if (output < pid[i].min_output) {
			pid[i].output = pid[i].min_output;
		} else if (output > pid[i].max_output) {
			pid[i].output = pid[i].max_output;
		} else {
			pid[i].output = output;
		}
	}
}

/*
 * write_output
 */
void write_output(volatile struct pid_data pid[]) {
	
	// Salida 1.
	// Establecimiento de los sentidos de giro.
	if (pid[0].output > 0) {
		PWMSS1.EPWM_CMPA = pid[0].output;		// R1 := PWM.	(Hacida adelante)
		PWMSS1.EPWM_CMPB = 0;							// R2 := 0.
	} else if (pid[0].output < 0){
		PWMSS1.EPWM_CMPA = 0;							// R1 := 0.		(Hacia atrás)
		PWMSS1.EPWM_CMPB = - pid[0].output;	// R2 := PWM.
	} else if (pid[0].output == 0) {
		PWMSS1.EPWM_CMPA = 0;							// R1 := 0;		(Parado)
		PWMSS1.EPWM_CMPB = 0;							// R2 := 0;
	}
	
	// Salida 2.
	// Establecimiento de los sentidos de giro.
	if (pid[1].output > 0) {
		PWMSS2.EPWM_CMPA = pid[1].output;		// L1 := PWM.	(Hacida adelante)
		PWMSS2.EPWM_CMPB = 0;							// L2 := 0.
	} else if (pid[1].output < 0){
		PWMSS2.EPWM_CMPA = 0;							// L1 := 0.		(Hacia atrás)
		PWMSS2.EPWM_CMPB = - pid[1].output;	// L2 := PWM.
	} else if (pid[1].output == 0) {
		PWMSS2.EPWM_CMPA = 0;							// L1 := 0;		(Parado)
		PWMSS2.EPWM_CMPB = 0;							// L2 := 0;
	}
	
}

/*
 * init_pid
 */
void init_pid(volatile struct pid_data pid[], volatile struct cycles_data* cycles) {
	
	short i;
	// PID.
	
	for (i = 0; i < NPID; i++) {
		
		pid[i].output = 0;
		pid[i].input = 0;
		
		pid[i].int_err = 0;
		pid[i].max_output = 4095; // Decimal 4095. Es el periodo del módulo ePWM ( a 100MHz) para convertir en PWM. (Máximo ciclo de trabajo)
		pid[i].min_output = -4095;
		
		pid[i].setpoint = 0;
	}
	
	pid[0].Kp = 500;	// 500
	pid[0].Ki = 900;	// 200
	
	pid[1].Kp = 500;
	pid[1].Ki = 900;
	
	// Cycles.
	cycles->lenght = 10000;
	
	cycles->loops = 0;
	cycles->min = SHRT_MAX;
	cycles->med = 0;
	cycles->max = 0;
	cycles->sum = 0;
}

/*
 * get_enc_rpm()
 */
float get_enc_rpm1() {
	
	float rpm;
	
	// Comprobación de errores de desbordamiento, encender el LED y reestablecer a 0 el RPM.
	if (PWMSS1.EQEP_QEPSTS &= 0x0C) {
		PWMSS1.EQEP_QEPSTS |= 0x0C;
		__R30 |= 0x04;	// bit 2 en alto para encender el led asociado.
		rpm = 0;
	} else {
		__R30 &= 0xFFFFFFFB;	// bit 2 apagado.
		rpm = (float)( (int) PWMSS1.EQEP_QPOSLAT * SAMPLES_PER_SEC * SEC_PER_MIN) / TICKS_PER_REV;	// Se hace un typecast int explícito
																							// para que recozca posiciones negativas.
	}
	
	return rpm;
}

float get_enc_rpm2() {
	
	float rpm;
	
	// Comprobación de errores de desbordamiento, encender el LED y reestablecer a 0 el RPM.
	if (PWMSS2.EQEP_QEPSTS &= 0x0C) {
		PWMSS2.EQEP_QEPSTS |= 0x0C;
		__R30 |= 0x08;	// bit 3 en alto para encender el led asociado.
		rpm = 0;
	} else {
		__R30 &= 0xFFFFFFF7;	// bit 3 apagado.
		rpm = (float)( (int) PWMSS2.EQEP_QPOSLAT * SAMPLES_PER_SEC * SEC_PER_MIN) / TICKS_PER_REV;
	}
	
	return rpm;
}

