#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>

#define RPMSG_BUF_SIZE    512

int main (int argc, char *argv[]) {
	
	int fd, wr, rd, idx, valorEntrada, valorSalida;
	char comando[5];
	char file[80];
	char msg_buffer[RPMSG_BUF_SIZE];
	char regreso[RPMSG_BUF_SIZE];
	
	sprintf(file, "/dev/rpmsg_pru30");
	
	do {
		fd = open(file, O_RDWR);
	} while (fd < 0);

	memset(msg_buffer, '\0', RPMSG_BUF_SIZE);
	
	printf("Introduce comando: \n");
	printf("'FIN' para acabar \n");
	scanf("%s", comando);
	
	while(strcmp(comando, "FIN") != 0) {
		
		printf("Introduce valor (0 en caso de no necesitarlo): \n");
		scanf("%i", &valorEntrada);

		for (idx = 0; idx < strlen(comando); idx++)
		msg_buffer[0] ^= comando[idx];
		
		msg_buffer[1] = valorEntrada & 0x000000FF;
		msg_buffer[2] = (valorEntrada & 0x0000FF00) >> 0x08;
		msg_buffer[3] = (valorEntrada & 0x00FF0000) >> 0x10;
		msg_buffer[4] = valorEntrada >> 0x18;
		
		wr = write(fd, msg_buffer, 5);
		
		memset(msg_buffer, '\0', RPMSG_BUF_SIZE);
		memset(regreso, '\0', RPMSG_BUF_SIZE);
		
		rd = read(fd, msg_buffer, 4);
		strcpy(regreso, msg_buffer);
		
		if(rd < 4){
			
			int cont;
			cont = rd;
			
			while(cont < 4){
				
				rd = read(fd, msg_buffer, 4 - cont);
				strcat(regreso, msg_buffer);
				cont += rd;
				
			}
		}

		valorSalida = (regreso[0] | regreso[1] << 0x08 | regreso[2] << 0x10 | regreso[3] << 0x18);
		
		printf("El valor de %s es: %i\n",comando, valorSalida);
		
		printf("Introduce comando: \n");
		printf("'FIN' para acabar \n");
		scanf("%s", comando);
	}
	
	close(fd);
	printf("Cerrando... \n");

	return 0;
}
