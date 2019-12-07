/* References: http://man7.org/training/download/posix_shm_slides.pdf */
// pshm_create_simple.c

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/types.h>

int main(int argc, char* argv[])
{
    int fd;
    size_t size;
    void* addr;

    size = atoi(argv[2]);
    fd = shm_open(argv[1], O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
    ftruncate(fd, size);
    addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    return 0;
}