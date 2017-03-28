#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>

#define RPMSG_BUF_SIZE    512

int main(int argc, char *argv[]) {
  int fd, index, val;
  char file[80];
  char msg_buff[RPMSG_BUF_SIZE];
  struct timespec t = {0};

  /* Open RPMsg channel file */
  sprintf(file, "/dev/rpmsg_pru%s", argv[1]);
  fd = open(file, O_RDWR);

  /* Close program if file does not exist */
  if(fd < 0) {
    perror(argv[1]);
    printf("Invalid RPMsg file\nExiting...\n");
    return 1;
  }

  /* Zero message buffer */
  memset(msg_buff, '\0', 512);

  /* XOR all command options together and store in first byte */
  for (index = 0; index < strlen(argv[2]); index++)
    msg_buff[0] ^= argv[2][index];

  /* If data is supplied, convert to int and write to buffer */
  if (argc == 4) {
    val = atoi(argv[3]);
    for (index = 0; index < 4; index++) {
      msg_buff[1] = val & 0x000000FF;
      msg_buff[2] = (val & 0x0000FF00) >> 0x08;
      msg_buff[3] = (val & 0x00FF0000) >> 0x10;
      msg_buff[4] = val >> 0x18;
    }
    /* Write data to PRU */
    write(fd, msg_buff, 5);
  } else {
    write(fd, msg_buff, 1);
  }

  /* Read back PRU message */
  read(fd, msg_buff, 4);
  val = (msg_buff[0] | msg_buff[1] << 0x08 | msg_buff[2] << 0x10 | msg_buff[3] << 0x18);

  /* Convert back to signed */
  if (val >> 0x1B) {
    val &= (0x7FFFFFF);
    val -= 0x7FFFFFF;
  }
  
  printf("%d", val);

  close(fd);

  return 0;
}
