#include "../inc/lux_task_init"

void lux_task_init(void)
{
    printf("[LUX ] Task Started - PID %ld - %d\n", (long)getpid(), ASDF);
}