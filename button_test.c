#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <poll.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>


int fd;

void my_signal_fun(int signum)
{
    unsigned char key_val;
    read(fd, &key_val, 1);
    printf("key_val: 0x%x\n", key_val);
}

int main(int argc, char **argv)
{
    unsigned char key_val;
    int ret;
    int Oflags;

    //��Ӧ�ó����в�׽SIGIO�źţ������������ͣ�
    signal(SIGIO, my_signal_fun);

    fd = open("/dev/irqkey", O_RDWR);
    if (fd < 0)
    {
        printf("can't open!\n");
        return 0;
    }

    //����ǰ����PID����Ϊfd�ļ�����Ӧ��������Ҫ����SIGIO,SIGUSR�źŽ���PID
    fcntl(fd, F_SETOWN, getpid());

    //��ȡfd�Ĵ򿪷�ʽ
    Oflags = fcntl(fd, F_GETFL);

    //��fd�Ĵ򿪷�ʽ����ΪFASYNC --- �� ֧���첽֪ͨ
    // ���д���ִ�лᴥ�� ���������� file_operations->fasync ���� ------fasync�������� fasync_helper��ʼ��һ��fasync_struct�ṹ�壬�ýṹ�������˽�Ҫ�����źŵĽ��� PID (fasync_struct->fa_file->f_owner->pid)
    fcntl(fd, F_SETFL, Oflags | FASYNC);


    while (1)
    {
        sleep(1000);
    }

    return 0;
}