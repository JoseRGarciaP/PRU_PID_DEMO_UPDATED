#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>

#define RPMSG_BUF_SIZE    512

int main(int argc, char *argv[]) {		// argv es un array de punteros char. Y argc es un entero con el número de punteros (partes del comando).
	
	int fd, wr, rd, idx;
	float val;		// fd: Valor de retorno de la función open(). Entero que representa
						// el descriptor del archivo. wr: Devuelve si se ha escrito adecuadamente. 
						//rd: Devuelve si se ha leído adecuadamente.
	
	char file[80];				// String de la ubicación del archivo del canal RPMsg.
	char msg_buff[RPMSG_BUF_SIZE];		// String buffer del mensaje.
	struct timespec t = {0};
	
	if ((argc <= 2) || (argc > 4)) {
		printf("Invalid RPMsg communication\nThe format is: ``prumsg [channel] command data´´\ne.g. prumsg 30 s 3000\nExiting...\n");
		return -1;
	}
	
	// Abrir el archivo del canal RPMsg.
	sprintf(file, "/dev/rpmsg_pru%s", argv[1]);	// Devuelve un string formateado "file". Con la cadena escrita y los datos
							// del puntero argv[1].
	
	fd = open(file, O_RDWR);			// Abre el archivo del canal RPMsg dado por el puntero de strings "file" y 
							// con el modo de acceso O_RDWR (Open for reading and writing).
							// La función abrirá el archivo y devolverá un entero no negativo
							// que represente el descriptor de archivo.
	
	// Cierra el programa si el archivo no existe.
	if(fd < 0) {
		perror(argv[1]);			// Pinta el valor argv[1] seguido de un mensaje descriptivo de error.
		printf("Invalid RPMsg channel file\nExiting...\n");
		return -1;
	}
	
	// Zero message buffer.
	memset(msg_buff, '\0', 512);			// Inicializa a cero msg_buff.
	
	// XOR a todas la partes del comando y se guarda en el primer byte de msg_buff.
	// Se codifica el comando a un valor de un byte.
	for (idx = 0; idx < strlen(argv[2]); idx++)
		msg_buff[0] ^= argv[2][idx];
	
	// Si se proporcionan datos, se convierten a float y se escriben en el buffer.
	if (argc == 4) {
		val = atof(argv[3]);						// Convierto el string del puntero 3 de argv a float. ( es donde está el valor de cambio)
		//for (index = 0; index < 4; index++) {				// Lo guardo en los siguientes bytes del buffer.
		msg_buff[1] = val & 0x000000FF;					// Cada índice del array debe contener un byte.
		msg_buff[2] = (val & 0x0000FF00) >> 0x08;
		msg_buff[3] = (val & 0x00FF0000) >> 0x10;			// **** Modificado quitando el bucle for que no se le aprecia sentido en primer momento.
		msg_buff[4] = val >> 0x18;
	//}
	// Escribe el dato a la PRU.
		wr = write(fd, msg_buff, 5);					// Escribe n bytes, donde n son los 5 bytes que hemos introducido en el array msg_buff.
	} else {
		wr = write(fd, msg_buff, 1);					// Aquí sólo el primer byte, que contiene el comando.
	}
  
	if (wr == -1){
		printf("Writing error\nExiting...\n");
		close(fd);
		return -1;
	}
	
	// Read back PRU message.
	rd = read(fd, msg_buff, 4);
	
	if (rd == -1) {
		printf("Reading error\nExiting...\n");
		close(fd);
		return -1;
	}
	
	val = (msg_buff[0] | msg_buff[1] << 0x08 | msg_buff[2] << 0x10 | msg_buff[3] << 0x18);
	
	/* Este fragmento es para cuando en la PRU 1 se declara el mensaje en unsigned int, yo lo he cambiado a float.
	// Convert back to signed
	if (val >> 0x1B) {			// Si desplazando 27 bits a la derecha siguen quedando 1`s, es decir, val mayor que 134217727.
	val &= (0x7FFFFFF);				// Se borran los 1`s esos y se resta a val 134217727. Está MAL además.
	val -= 0x7FFFFFF;
	}
	*/
	
	
	printf("El valor de %s es: %f\n",argv[2], val);
	
	close(fd);
	
	return 0;
}

