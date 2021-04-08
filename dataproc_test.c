#include <src/data_processor.c>
#include <string.h>

void* procFun(void* args)
{
    char* arg_str = (char*) args;
    printf("%s %d\n", (char*)arg_str, strlen(arg_str));
}

int main()
{
    pthread_t data_proc_tid;

    dataproc_t* data_proc = dataprocCreate(10);

    char data[3][20] = 
    {
        "Hello World",
        "Greetings Galaxy",
        "Salutations Universe"
    };

    pthread_create(&data_proc_tid, NULL, &dataprocMain,data_proc);

    for (int i = 0; i < 3; i++)
    {
        dataprocSendData(data_proc, &procFun, data[i], sizeof(data[i]), 0, true);
    }

    dataprocSendStop(data_proc, 0, true);

    pthread_join(data_proc_tid, NULL);
}