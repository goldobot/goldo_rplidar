#include <sys/ioctl.h>
#include <limits.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <stdio.h>
#include <unistd.h>

/* Raspi 3 */
//#define PI_PERI_PHYS  (0x3F000000)
/* Raspi 4 */
#define PI_PERI_PHYS  (0xFE000000)
#define GPIO_BASE     (PI_PERI_PHYS + 0x00200000)
#define GPIO_LEN      0xF4

#define GPSET0     7
#define GPSET1     8

#define GPCLR0    10
#define GPCLR1    11

#define BANK (gpio>>5)

#define BIT  (1<<(gpio&0x1F))

typedef unsigned int uint32_t;

static int fd_gpio = -1;
static volatile uint32_t * gpio_reg = (uint32_t *) MAP_FAILED;

#define SHMEM_SIZE   4096
static int fd_mmap = -1;
char* rp_shmem = NULL;

static uint32_t * init_map_mem(int fd, uint32_t addr, uint32_t len)
{
  return (uint32_t *) mmap(0, len,
                           PROT_READ|PROT_WRITE,
                           MAP_SHARED|MAP_LOCKED,
                           fd, addr);
}

int goldo_gpio_init()
{
  if ((fd_gpio = open("/dev/mem", O_RDWR | O_SYNC) ) < 0)
  {
    printf ("open(/dev/mem) failed\n");
    return -1;
  }

  gpio_reg = init_map_mem(fd_gpio, GPIO_BASE, GPIO_LEN);
  if (gpio_reg == MAP_FAILED)
  {
    printf ("init_map_mem() failed\n");
    return -1;
  }

  if ((fd_mmap = open("/home/goldorak/workspace/goldo_main/rplidar_shmem.txt", O_RDWR, 0)) == -1)
  {
    printf("unable to open 'rplidar_shmem.txt'\n");
    return 0;
  }

  rp_shmem = (char*) mmap(NULL, SHMEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mmap, 0);

  return 0;
}

int goldo_gpio_uninit()
{
  munmap((void *)gpio_reg, GPIO_LEN);

  close (fd_gpio);

  return 0;
}

void goldo_gpio_check_shmem()
{
  if (gpio_reg == MAP_FAILED)
  {
    return;
  }
  if (rp_shmem != NULL)
  {
    if (rp_shmem[0] == 0x00)
    {
      *(gpio_reg + GPCLR0) = 0x200000;
    }
  }
}

void goldo_gpio_set()
{
  if (gpio_reg == MAP_FAILED)
  {
    return;
  }
  if (rp_shmem != NULL)
  {
    if (rp_shmem[0] == 0x00)
    {
      return;
    }
  }
  *(gpio_reg + GPSET0) = 0x200000;
}

void goldo_gpio_clr()
{
  if (gpio_reg == MAP_FAILED)
  {
    return;
  }
  if (rp_shmem != NULL)
  {
    if (rp_shmem[0] == 0x00)
    {
      return;
    }
  }
  *(gpio_reg + GPCLR0) = 0x200000;
}

