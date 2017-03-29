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
    int Kp_f, Ki_f, Kd_f;

    /* Controles PID */
    int setpoint;
    int int_err;
    int input, output, last_output;
    int min_output, max_output;
	int min, med, max, sum;
};												//Estas estructuras son compartidas entre las PRUs,
												// y así se forma la comunicación entre ambas.
/* Estructura del bloque de memoria compartida */
struct shared_mem {
    volatile char init_flag;
    volatile unsigned int pwm_out;
    volatile int enc_rpm;
    volatile struct pid_data pid;
};

/* Estructura de datos RPMsg */
struct rpmsg_unit {
    char cmd;                  // Comando.
    unsigned int msg;          // Mensaje (Dato numérico de sintonización PID ó de parámetro)
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
/* Este registro activa el reloj del modulo PWMSS1. */
#define CM_PER_EPWMSS1 (*((volatile unsigned int *)0x44E000CC))

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
void init_pwm();                             // Inicialización del módulo eCAP PWM.
void init_eqep();                            // Inicialización del módulo eQEP.
int get_enc_rpm();                           // Obtención de la velocidad por el encoder.
void init_rpmsg(struct pru_rpmsg_transport* transport);   // Inicialización del bloque RPMsg.
void rpmsg_interrupt(volatile struct pid_data* pid, struct pru_rpmsg_transport *transport, uint8_t *payload,
        uint16_t src, uint16_t dst, uint16_t len);    // Comprobación de las interrupciones generadas por ARM.
void rpmsg_isr(volatile struct pid_data* pid, struct pru_rpmsg_transport *transport, uint8_t *payload,
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

/* Establecimiento de la memoria compartida */
#pragma DATA_SECTION(share_buff, ".share_buff")    // Asigna el espacio de memoria de la
                                                   // sección ".share_buff" al símbolo share_buff
volatile far struct shared_mem share_buff;         // Se define el símbolo shared_buff como una instancia de la estructura
                                                   // shared_mem, tipo volatile y far ( 16 bits superiores de la memoria)
												   
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
    init_eqep();
    init_pwm();

    /* Inicializa RPMsg */
    init_rpmsg(&transport);

    /* Establece init flag para indicar la continuación de la inicializacion del PID */
    share_buff.init_flag = 1;

    while (1) {
        /* Obtiene los mensajes del espacio de usuario */
        rpmsg_interrupt(&share_buff.pid, &transport, payload, dst, src, len);

        /* Establece la velocidad PWM (registro ACMP del eCAP) */

        CT_ECAP.CAP2_bit.CAP2 = share_buff.pid.output;
//        CT_ECAP.CAP2_bit.CAP2 = 0x0AF0;  //  Esto fuerza la salida a un valor determinado PWM.
        /* Guarda los ciclos de escritura esperando al evento de cambio */
        if (PWMSS1.EQEP_QFLG & 0x0800) {
            PWMSS1.EQEP_QCLR |= 0x0800;
            share_buff.pid.input = get_enc_rpm();
        }
    }
}

/*
 * Inicia APWM
 */
void init_pwm() {
    /* Establece en la memoria el PWM a 0 */
    share_buff.pwm_out = 0;

    /* Habilita el modo APWM y en operación asíncrona; Establece la polaridad a activa alta */
    CT_ECAP.ECCTL2 = 0x02C0;

    /* Establece el número de ciclos de reloj en el periodo PWM (APRD) */
    CT_ECAP.CAP1 = PERIOD_CYCLES;

    /* Habilita el contador ECAP PWM Freerun */
    CT_ECAP.ECCTL2 |= 0x0010;
}

/*
 * Inicializa eQEP
 */
void init_eqep() {
    /* Establece en la memoria el RPM a 0 */
    share_buff.enc_rpm = 0;

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
}

//  get_enc_rpm()

int get_enc_rpm() {
    int rpm = 0;

    /* Comprobación de errores de desbordamiento, encender el LED y reestablecer a 0 el RPM */
    if (PWMSS1.EQEP_QEPSTS &= 0x0C) {
        PWMSS1.EQEP_QEPSTS |= 0x0C;
        __R30 = 0xFF;
        rpm = 0;
    } else {
        __R30 = 0x00;
        rpm = (PWMSS1.EQEP_QPOSLAT * SAMPLES_PER_SEC * SEC_PER_MIN) / TICKS_PER_REV;
    }

    return rpm;
}

//  init_rpmsg
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
void rpmsg_interrupt(volatile struct pid_data* pid, struct pru_rpmsg_transport *transport, uint8_t *payload,
        uint16_t src, uint16_t dst, uint16_t len) {
    // Comprueba el bit 31 del registro R31 para ver si ARM ha mensajeado */
    if(__R31 & HOST_INT){

        /* Borra el estado de eventos */
        CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

/* Recibe todos los mensajes disponibles, múltiples mensajes pueden ser enviados por vez */
                /* Recibe los mensajes */
                if(pru_rpmsg_receive(transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS){
                    /* Servicio de la interrupción */
                    rpmsg_isr(pid, transport, payload, src, dst);
                }}}

/*
 * rpmsg_isr
 */
void rpmsg_isr(volatile struct pid_data* pid, struct pru_rpmsg_transport *transport, uint8_t *payload,
        uint16_t src, uint16_t dst) {
    struct rpmsg_unit* rpunit;

    rpunit = (struct rpmsg_unit *) payload;

    /* Comprueba comando */
    switch(rpunit->cmd) {
    /* Establece setpoint */
    case ('s'):
        pid->setpoint = rpunit->msg;
        rpunit->msg = pid->setpoint;
        break;
    /* Establece Kp */
    case ('p'):
        pid->Kp_f = rpunit->msg;
        rpunit->msg = pid->Kp_f;
        break;
    /* Establece Ki */
    case ('i'):
        pid->Ki_f = rpunit->msg;
        rpunit->msg = pid->Ki_f;
        break;
    /* Establece Kd */
    case ('d'):
        pid->Kd_f = rpunit->msg;
        rpunit->msg = pid->Kd_f;
        break;
    /* Establecer salida PWM */
    case ('o'):
        pid->output = rpunit->msg;
        rpunit->msg = pid->output;
        break;
    /* Leer setpoint */
    case ('r'^'s'):
        rpunit->msg = pid->setpoint;
        break;
    /* Leer Kp */
    case ('r'^'p'):
        rpunit->msg = pid->Kp_f;
        break;
    /* Leer Ki */
    case ('r'^'i'):
        rpunit->msg = pid->Ki_f;
        break;
    /* Leer Kd */
    case ('r'^'d'):
        rpunit->msg = pid->Kd_f;
        break;
    /* Leer encoder RPM */
    case ('r'^'e'):
        rpunit->msg = pid->input;
        break;
    /* Leer salida PWM */
    case ('r'^'o'):
        rpunit->msg = pid->output;
        break;
	/* Leer número medio de ciclos del PID */
    case ('m'^'d'):
        rpunit->msg = pid->med;
        break;
	/* Leer número maximo de ciclos del PID */
    case ('m'^'x'):
        rpunit->msg = pid->max;
        break;
	/* Leer número mínimo de ciclos del PID */
    case ('m'^'n'):
        rpunit->msg = pid->min;
        break;
		/* Leer suma total de los ciclos del PID */
    case ('s'^'u'):
        rpunit->msg = pid->sum;
        break;
    }
    /* Envío de mensaje de vuelta al host */
    pru_rpmsg_send(transport, dst, src, &rpunit->msg, 4);
}
