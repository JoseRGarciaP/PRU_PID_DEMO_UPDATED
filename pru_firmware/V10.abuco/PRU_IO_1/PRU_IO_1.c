/* Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/ 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 * 	* Redistributions of source code must retain the above copyright 
 * 	  notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	  notice, this list of conditions and the following disclaimer in the 
 * 	  documentation and/or other materials provided with the   
 * 	  distribution.
 * 	* Neither the name of Texas Instruments Incorporated nor the names of
 * 	  its contributors may be used to endorse or promote products derived
 * 	  from this software without specific prior written permission.
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
 //  This version was revised by Gregory Raven to be compatible with
 //  recent PRU RemoteProc framework which changed to System Events
 //  (was Mailboxes).

#include <stdint.h>
#include <limits.h>
#include <pru_cfg.h>
#include <pru_ecap.h>
#include <sys_pwmss.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_virtqueue.h>
#include <pru_rpmsg.h>
#include "resource_table_1.h"


/* Estructura de datos del PID compartida - asegurar que ambas estructuras coinciden con PRU 0 */
struct pid_data {
    /* Sintoniación PID */
    	short Kp_f, Ki_f, Kd_f;

    /* Controles PID */
    	short setpoint;
    	int int_err;
    	short input, output, last_output;
    	short min_output, max_output;
};												// Estas estructuras son compartidas entre las PRUs,
												// y así se forma la comunicación entre ambas.

struct cycles_data {
	int min, med, max, sum, loops;
};
												
/* Estructura del bloque de memoria compartida */
struct shared_mem {
    	volatile char init_flag;
	volatile struct cycles_data cycles;
    	volatile struct pid_data pid1;
	volatile struct pid_data pid2;
};


/* Establecimiento de la memoria compartida */
#pragma DATA_SECTION(share_buff, ".share_buff")    // Asigna el espacio de memoria de la
                                                   // sección ".share_buff" al símbolo share_buff
volatile far struct shared_mem share_buff;         // Se define el símbolo shared_buff como una instancia de la estructura
                                                   // shared_mem, tipo volatile y far ( 16 bits superiores de la memoria)

/* Estructura de datos RPMsg */
struct rpmsg_unit {
    char cmd;                  // Comando. Es el primer byte del buffer RPMsg.
    int msg;     	     // Mensaje (Dato numérico de sintonización PID ó de parámetro). Son los bytes siguientes con el valor numerico, si lo hubiera.
				// Modificado, antes era unsigned int msg;
};

/* Registros GPIO de las PRU */
// Se define el registro R30 interno de la PRU de tipo volatile.
// Escribir en el registro R30 controla los pines GPO.
volatile register uint32_t __R30;

// Se define el registro R31 interno de la PRU de tipo volatile.
// El registro R31 sirve como interfaz con los pines dedicados a entradas de propósito general de la
// PRU (GPI) y el controlador de interrupciones de la PRU (INTC). Leer el registro R31 devuelve
// el estado de los pines GPI e INTC (bits 30 y 31) vía PRU Real-Time Status Interface.
// Escribir en el registro R31 genera interrupciones - ver las especificaciones del dispositivo TRM para más información.
volatile register uint32_t __R31;

/* Configuración del Encoder de Cuadratura eQEP. */
/* Definición de registro Non-CT. Registro externo de la PRU (Clock Module Peripheral Registers). */
/* Este registro CM_PER_EPWMSS1_CLKCTRL activa el reloj del modulo PWMSS1. */
#define CM_PER_EPWMSS1 (*((volatile unsigned int *)0x44E000CC))
/* Este registro CM_PER_EPWMSS0_CLKCTRL activa el reloj del modulo PWMSS0. */
#define CM_PER_EPWMSS0 (*((volatile unsigned int *)0x44E000D4))
/* Este registro CM_PER_EPWMSS2_CLKCTRL activa el reloj del modulo PWMSS2. */
#define CM_PER_EPWMSS2 (*((volatile unsigned int *)0x44E000D8))

/* Configuración del Módulo eCAP PWM. */
/* Parámetro del periodo para la generación PWM*/
#define PERIOD_CYCLES       0x1000                 // ciclos de un periódo = 4096 decimal.

/* Configuración del módulo eQEP. */

/* Parámetros del Encoder de Cuadratura */
#define TICKS_PER_REV       40 
#define SAMPLES_PER_SEC     12
#define SEC_PER_MIN         60

/* Configuración RPMsg. */

/* La Interrupción Host-1 establece el bit 31 en el registro R31 */
#define HOST_INT            ((uint32_t) 1 << 31) 

//  Los eventos de sistema de PRU-ICSS usados para RPMsg son definidos en el device
//  tree de Linux.
//  PRU0 usa system event 16 (hacia ARM) y 17 (desde ARM)
//  PRU1 usa system event 18 (hacia ARM) y 19 (desde ARM)
#define TO_ARM_HOST 18
#define FROM_ARM_HOST 19

/*
 * El nombre 'rpmsg-pru' corresponde al driver rmpsg_pru encontrado
 * en linux-x.y.z/drivers/rpmsg/rpmsg_pru.c
 */
#define CHAN_NAME           "rpmsg-pru"

#define CHAN_DESC           "Channel 30"
#define CHAN_PORT           30

#define CHAN_DESC_2         "Channel 31"
#define CHAN_PORT_2         31

/* Declaración de funciones, prototipo */
void init_eqep();                            // Inicialización del módulo eQEP, y relojes de PWMSS.
void init_pwm();                             // Inicialización del módulo eCAP PWM.
int get_enc_rpm1();                          // Obtención de la velocidad por el encoder.
int get_enc_rpm2(); 
void init_rpmsg(struct pru_rpmsg_transport* transport);   // Inicialización del bloque RPMsg.
void rpmsg_interrupt(volatile struct pid_data* pid1, volatile struct pid_data* pid2, volatile struct cycles_data* cycles, struct pru_rpmsg_transport *transport, uint8_t *payload,
        uint16_t src, uint16_t dst, uint16_t len);    // Comprobación de las interrupciones generadas por ARM.
void rpmsg_isr(volatile struct pid_data* pid1, volatile struct pid_data* pid2, volatile struct cycles_data* cycles, struct pru_rpmsg_transport *transport, uint8_t *payload,
        uint16_t src, uint16_t dst);                  // Servicio de la interrupción.
void int_to_payload(uint8_t *payload, int data);      // Entero a dato de comunicación.
unsigned int payload_to_int(uint8_t *payload);        // Dato de comunicación a entero.

/*
 * Usado para estar seguro que los drivers de Linux están preparados para la comunicación RPMsg
 * Encontrado en linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

/* Buffer RPMsg, global para reducir el tamaño de pila */
uint8_t payload[RPMSG_BUF_SIZE];

												   
/*
 * main.c
 */
void main(void) {
    /* Variables RPMsg */
    struct pru_rpmsg_transport transport;
    uint16_t src, dst, len;

    /* Permite el acceso al puerto OCP master por la PRU y así la PRU puede leer memorias externas */
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    /* Inicializa el funcionamiento de los periféricos */
    init_eqep();       // Poner este primero ya que inicializa los relojes de los módulos PWMSS.
    init_pwm();

    /* Inicializa RPMsg */
    init_rpmsg(&transport);

    /* Establece init flag para indicar el comienzo del PID */
    share_buff.init_flag = 1;

    while (1) {
        /* Obtiene los mensajes del espacio de usuario */
        rpmsg_interrupt(&share_buff.pid1, &share_buff.pid2, &share_buff.cycles, &transport, payload, dst, src, len);

        /* Establece la velocidad PWM (registro ACMP del eCAP) */
        CT_ECAP.CAP2_bit.CAP2 = share_buff.pid1.output;
		PWMSS2.ECAP_CAP2_bit.CAP2 = share_buff.pid2.output;
//      CT_ECAP.CAP2_bit.CAP2 = 0x0AF0;  //  Esto fuerza la salida a un valor determinado PWM.
//      PWMSS2.ECAP_CAP2_bit.CAP2 = 0x0AF0;

        /* Guarda los ciclos de escritura esperando al evento de cambio */
        if (PWMSS1.EQEP_QFLG & 0x0800) {
            PWMSS1.EQEP_QCLR |= 0x0800;
            share_buff.pid1.input = get_enc_rpm1();
        }
		if (PWMSS2.EQEP_QFLG & 0x0800) {
            PWMSS2.EQEP_QCLR |= 0x0800;
            share_buff.pid2.input = get_enc_rpm2();
        }
    }
}

/*
 * Inicializa eQEP
 */
void init_eqep() {															//Establecer otro periférico para el otro encoder.

	// eQEP PWMSS1

    /* Habilita la generación de la señal de reloj PWMSS1 */
    while (!(CM_PER_EPWMSS1 & 0x2))
        CM_PER_EPWMSS1 |= 0x2;

    /* Establece valores por defecto en modo de cuadratura */
    PWMSS1.EQEP_QDECCTL = 0x00;

    /* Activa temporizador de unidad
     * Activa bloqueo de captura en el tiempo de espera de la unidad
     * Activa el contador de posición de cuadratura
     * Activa la carga de software del contador de posición
     * Resetea el contador de posición en el evento de tiempo de unidad para medir RPM
     */
    PWMSS1.EQEP_QEPCTL = 0x308E;

    /* Establece preescalares para el timer de captura EQEP y UPEVNT */
    /* Nota: La unidad de captura EQEP debe estar deshabilitada antes de cambiar los preescalares */
    PWMSS1.EQEP_QCAPCTL = 0x0070;

    /* Habilita captura EQEP */
    PWMSS1.EQEP_QCAPCTL |= 0x8000;

    /* Habilita la interrupción del tiempo de espera de la unidad */
    PWMSS1.EQEP_QEINT |= 0x0800;

    /* Borra el conteo del encoder */
    PWMSS1.EQEP_QPOSCNT_bit.QPOSCNT = 0x00000000;

    /* Establece el máximo conteo del encoder */
    PWMSS1.EQEP_QPOSMAX_bit.QPOSMAX = UINT_MAX;

    /* Borra el timer */
    PWMSS1.EQEP_QUTMR_bit.QUTMR = 0x00000000;

    /* Establece el periodo de conteo del timer de la unidad */
    /*  QUPRD = Period * 100MHz */
    PWMSS1.EQEP_QUPRD_bit.QUPRD = 0x007FFFFF; // (~1/12s) @ 100MHz

    /* Borra todos los bits de interrupción */
    PWMSS1.EQEP_QCLR = 0xFFFF;
	
	// eQEP PWMSS2

	/* Habilita la generación de la señal de reloj PWMSS2 */
    while (!(CM_PER_EPWMSS2 & 0x2))
        CM_PER_EPWMSS2 |= 0x2;

    /* Establece valores por defecto en modo de cuadratura */
    PWMSS2.EQEP_QDECCTL = 0x00;

    /* Activa temporizador de unidad
     * Activa bloqueo de captura en el tiempo de espera de la unidad
     * Activa el contador de posición de cuadratura
     * Activa la carga de software del contador de posición
     * Resetea el contador de posición en el evento de tiempo de unidad para medir RPM
     */
    PWMSS2.EQEP_QEPCTL = 0x308E;

    /* Establece preescalares para el timer de captura EQEP y UPEVNT */
    /* Nota: La unidad de captura EQEP debe estar deshabilitada antes de cambiar los preescalares */
    PWMSS1.EQEP_QCAPCTL = 0x0070;

    /* Habilita captura EQEP */
    PWMSS2.EQEP_QCAPCTL |= 0x8000;

    /* Habilita la interrupción del tiempo de espera de la unidad */
    PWMSS2.EQEP_QEINT |= 0x0800;

    /* Borra el conteo del encoder */
    PWMSS2.EQEP_QPOSCNT_bit.QPOSCNT = 0x00000000;

    /* Establece el máximo conteo del encoder */
    PWMSS2.EQEP_QPOSMAX_bit.QPOSMAX = UINT_MAX;

    /* Borra el timer */
    PWMSS2.EQEP_QUTMR_bit.QUTMR = 0x00000000;

    /* Establece el periodo de conteo del timer de la unidad */
    /*  QUPRD = Period * 100MHz */
    PWMSS2.EQEP_QUPRD_bit.QUPRD = 0x007FFFFF; // (~1/12s) @ 100MHz

    /* Borra todos los bits de interrupción */
    PWMSS2.EQEP_QCLR = 0xFFFF;
}

/*
 * Inicia APWM
 */
void init_pwm() {													// Establecer otro periférico para el otro motor.

	// PRU_eCAP0 PWM

    /* Habilita el modo APWM y en operación asíncrona; Establece la polaridad a activa alta */
    CT_ECAP.ECCTL2 = 0x02C0;

    /* Establece el número de ciclos de reloj en el periodo PWM (APRD) */
    CT_ECAP.CAP1_bit.CAP1 = PERIOD_CYCLES;

    /* Habilita el contador ECAP PWM Freerun */
    CT_ECAP.ECCTL2 |= 0x0010;

	// PWMSS2 eCAP2 PWM

	/* Habilita el modo APWM y en operación asíncrona; Establece la polaridad a activa alta */
    PWMSS2.ECAP_ECCTL2 = 0x02C0;

    /* Establece el número de ciclos de reloj en el periodo PWM (APRD) */
    PWMSS2.ECAP_CAP1_bit.CAP1 = PERIOD_CYCLES;

    /* Habilita el contador ECAP PWM Freerun */
    PWMSS2.ECAP_ECCTL2 |= 0x0010;

}

//  get_enc_rpm()

int get_enc_rpm1() {
    short rpm = 0;

    /* Comprobación de errores de desbordamiento, encender el LED y reestablecer a 0 el RPM */
    if (PWMSS1.EQEP_QEPSTS &= 0x0C) {
        PWMSS1.EQEP_QEPSTS |= 0x0C;
        __R30 |= 0x04;    // bit 2 en alto para encender el led asociado.
        rpm = 0;
    } else {
        __R30 &= 0xFFFFFFFB;	 // bit 2 apagado.
        rpm = (PWMSS1.EQEP_QPOSLAT * SAMPLES_PER_SEC * SEC_PER_MIN) / TICKS_PER_REV;
    }

    return rpm;
}

int get_enc_rpm2() {
    short rpm = 0;

    /* Comprobación de errores de desbordamiento, encender el LED y reestablecer a 0 el RPM */
    if (PWMSS2.EQEP_QEPSTS &= 0x0C) {
        PWMSS2.EQEP_QEPSTS |= 0x0C;
        __R30 |= 0x08;	 // bit 3 en alto para encender el led asociado.
        rpm = 0;
    } else {
        __R30 &= 0xFFFFFFF7;	 // bit 3 apagado.
        rpm = (PWMSS2.EQEP_QPOSLAT * SAMPLES_PER_SEC * SEC_PER_MIN) / TICKS_PER_REV;
    }

    return rpm;
}

//  inicializacion rpmsg
//  Ulilizando eventos del sistema en vez de mailboxes. 
void init_rpmsg(struct pru_rpmsg_transport* transport) {
	volatile uint8_t *status;

	// Borra el estado de los eventos del sistema de PRU-ICSS que usará ARM para hacer el "kick".
    CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

	/* Comprobación de que los drivers de Linux están preparados para la comunicación RPMsg */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	/* Inicializa la estructura de transporte RPMsg */
    pru_rpmsg_init(transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

    /* Crea los canales RPMsg entre la PRU y el espacio de usuario ARM usando la estructura de transporte. */
    while (pru_rpmsg_channel(RPMSG_NS_CREATE, transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS);
    while (pru_rpmsg_channel(RPMSG_NS_CREATE, transport, CHAN_NAME, CHAN_DESC_2, CHAN_PORT_2) != PRU_RPMSG_SUCCESS);
}

/*
 * rpmsg_interrupt
 */
void rpmsg_interrupt(volatile struct pid_data* pid1, volatile struct pid_data* pid2, volatile struct cycles_data* cycles, struct pru_rpmsg_transport *transport, uint8_t *payload,
        uint16_t src, uint16_t dst, uint16_t len) {
    // Comprueba el bit 31 del registro R31 para ver si ARM ha mensajeado */
    if(__R31 & HOST_INT){

        /* Borra el estado de eventos */
        CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

/* Recibe todos los mensajes disponibles, múltiples mensajes pueden ser enviados por vez */
                /* Recibe los mensajes */
                if(pru_rpmsg_receive(transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS){
                    /* Servicio de la interrupción */
                    rpmsg_isr(pid1, pid2, cycles, transport, payload, src, dst);
                }
	}
}

/*
 * rpmsg_isr
 */
void rpmsg_isr(volatile struct pid_data* pid1, volatile struct pid_data* pid2, volatile struct cycles_data* cycles, struct pru_rpmsg_transport *transport, uint8_t *payload,
        uint16_t src, uint16_t dst) {
    struct rpmsg_unit* rpunit;

    rpunit = (struct rpmsg_unit *) payload;           // Puntero al buffer de comando y datos, ambos vienen 
													// correlativos en la dirección del puntero payload
													// Se inicializa el puntero de la estructura struct rpmsg_unit con los valores del puntero payload del buffer.
													// El primer byte contiene el puntero al dato del comando, este dato es un char con el código equivalente 
													// al comando.
													// Le siguen los bytes de los punteros del dato numerico del parámetro a cambiar, que es entero y 
													// se cargan en rpunit->msg.
    /* Comprueba comando */
    switch(rpunit->cmd) {
		
	/* Comandos del PID 1 */
		
    /* Establece setpoint */
    case ('s'^'p'^'d'):							// 0x67
        pid1->setpoint = rpunit->msg;
        rpunit->msg = pid1->setpoint;
        break;
    /* Establece Kp */
    case ('k'^'p'^'d'):							// 0x7F
        pid1->Kp_f = rpunit->msg;
        rpunit->msg = pid1->Kp_f;
        break;
    /* Establece Ki */
    case ('k'^'i'^'d'):							// 0x66
        pid1->Ki_f = rpunit->msg;
        rpunit->msg = pid1->Ki_f;
        break;
    /* Establece Kd */
    case ('k'^'d'^'d'):							// 0x6B
        pid1->Kd_f = rpunit->msg;
        rpunit->msg = pid1->Kd_f;
        break;
    /* Establecer salida PWM */
    case ('o'^'d'):								// 0x0B
        pid1->output = rpunit->msg;
        rpunit->msg = pid1->output;
        break;
    /* Leer setpoint */
    case ('r'^'s'^'d'):							// 0x65
        rpunit->msg = pid1->setpoint;
        break;
    /* Leer Kp */
    case ('r'^'k'^'p'^'d'):						// 0x0D
        rpunit->msg = pid1->Kp_f;
        break;
    /* Leer Ki */
    case ('r'^'k'^'i'^'d'):						// 0x14
        rpunit->msg = pid1->Ki_f;
        break;
    /* Leer Kd */
    case ('r'^'k'^'d'^'d'):						// 0x19
        rpunit->msg = pid1->Kd_f;
        break;
    /* Leer encoder RPM */
    case ('r'^'e'^'d'):							// 0x73
        rpunit->msg = pid1->input;
        break;
    /* Leer salida PWM */
    case ('r'^'o'^'d'):							// 0x79
        rpunit->msg = pid1->output;
        break;
		
	/* Comandos del PID 2 */
	
	/* Establece setpoint */	
	case ('s'^'p'^'i'):							// 0x6A
        pid2->setpoint = rpunit->msg;
        rpunit->msg = pid2->setpoint;
        break;
    /* Establece Kp */
    case ('k'^'p'^'i'):							// 0x72
        pid2->Kp_f = rpunit->msg;
        rpunit->msg = pid2->Kp_f;
        break;
    /* Establece Ki */
    case ('k'^'i'^'i'^'q'):						// 0x1A
        pid2->Ki_f = rpunit->msg;
        rpunit->msg = pid2->Ki_f;
        break;
    /* Establece Kd */
    case ('k'^'d'^'i'^'q'):						// 0x17
        pid2->Kd_f = rpunit->msg;
        rpunit->msg = pid2->Kd_f;
        break;
    /* Establecer salida PWM */
    case ('o'^'i'):								// 0x06
        pid2->output = rpunit->msg;
        rpunit->msg = pid2->output;
        break;
    /* Leer setpoint */
    case ('r'^'s'^'i'):							// 0x68
        rpunit->msg = pid2->setpoint;
        break;
    /* Leer Kp */
    case ('r'^'k'^'p'^'i'):						// 0x00
        rpunit->msg = pid2->Kp_f;
        break;
    /* Leer Ki */
    case ('r'^'k'^'i'^'q'):						// 0x01
        rpunit->msg = pid2->Ki_f;
        break;
    /* Leer Kd */
    case ('r'^'k'^'d'^'q'):						// 0x0C
        rpunit->msg = pid2->Kd_f;
        break;
    /* Leer encoder RPM */
    case ('r'^'e'^'i'):							// 0x7E
        rpunit->msg = pid2->input;
        break;
    /* Leer salida PWM */
    case ('r'^'o'^'i'):							// 0x74
        rpunit->msg = pid2->output;
        break;
	
	/* Comandos del conteo de ciclos */
	
	/* Leer número medio de ciclos del PID */
    case ('m'^'e'^'d'):						    // 0x6C
        rpunit->msg = cycles->med;
        break;
	/* Leer número maximo de ciclos del PID */
    case ('m'^'x'):							    // 0x15
        rpunit->msg = cycles->max;
        break;
	/* Leer número mínimo de ciclos del PID */
    case ('m'^'n'):								// 0x03
        rpunit->msg = cycles->min;
        break;
	/* Leer suma total de los ciclos del PID */
    case ('s'^'u'^'m'^'c'):						// 0x08
        rpunit->msg = cycles->sum;
        break;
	/* Leer total de bucles */
	case ('r'^'l'):							    // 0x1E
        rpunit->msg = cycles->loops;
        break;	
    }
    /* Envío de mensaje de vuelta al host */
    pru_rpmsg_send(transport, dst, src, &rpunit->msg, 4);
}
