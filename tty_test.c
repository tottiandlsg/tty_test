#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <pthread.h>

#define HANDLE_REC_DATA_MAX  100

void *uart_send_handle(void *data)
{
	char buf[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
	int fd = *(int*)data;
	int ret = 0;
	while(1)
	{
		ret = write(fd, buf, 5);
		sleep(1);
	}
	return 0;
}

void *uart_rec_handle(void *data)
{
	int len;
	char buf[HANDLE_REC_DATA_MAX];
	int i;
	int ret = 0;
	int fd = *(int*)data;
	while(1)
	{
		len = 0;
		ret = ioctl(fd, FIONREAD, &len);
		if (!len)
			continue;
		if (len >= HANDLE_REC_DATA_MAX)
			len = HANDLE_REC_DATA_MAX;
		ret = read(fd, buf, len);
		for (i = 0; i < len; i++)
			printf("the read data is %d\n", buf[i]);
	}
	return 0;
}

int main(void)
{
	int fd;
	int ret = 0;
	pthread_t pthread_send, pthread_rec;
	struct termios options;
	fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd < 0)
	{
		printf("ttys2 open failed\n");
		return -1;
	}
	
	options.c_cflag &= ~CSTOPB; 
	options.c_cflag &= ~CSIZE;  //屏蔽字符大小位  
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~PARODD;
	options.c_cflag |= CS8;
	options.c_cflag &= ~CRTSCTS;
	//ECHO禁止回环输出
	options.c_lflag &= ~(ICANON | IEXTEN | ISIG | ECHO);
	options.c_oflag &= ~OPOST; //原始数据输出
	options.c_iflag &= ~(ICRNL | INPCK | ISTRIP | IXON | BRKINT );
	options.c_cc[VMIN] = 1;
	options.c_cc[VTIME] = 0;
	//CREAD使能输入
	options.c_cflag |= (CLOCAL | CREAD);
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	tcsetattr(fd, TCSANOW, &options);

	pthread_create(&pthread_send, NULL, uart_send_handle, &fd);
	pthread_create(&pthread_rec, NULL, uart_rec_handle, &fd);
	pthread_join(pthread_send, NULL);
	pthread_join(pthread_rec, NULL);
	close(fd);
	return 0;
}
