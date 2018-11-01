#include "gpio_nif.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/mman.h>

#define GPIO_MAP_BLOCK_SIZE (4*1024)
#define PAGE_SIZE  (4*1024)
#define GPIO_BASE_OFFSET    0x200000

// sysfs_utils.c
int sysfs_write_file(const char *pathname, const char *value);

// Copied from host_bcm.c source, bcm_host_get_peripheral_address()
// was undefined when loading the GPIO NIF
static uint32_t get_dt_ranges(const char *filename, unsigned offset)
{
    uint32_t address = 0xffffffff;

    FILE *fp = fopen(filename, "rb");
    if (fp)
    {
        unsigned char buf[4];
        fseek(fp, offset, SEEK_SET);
        if (fread(buf, 1, sizeof(buf), fp) == sizeof(buf))
            address = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3] << 0;
        fclose(fp);
    }
    return address;
}

// Copied from host_bcm.c source.
static uint32_t bcm_host_get_peripheral_address(void)
{
    uint32_t address = get_dt_ranges("/proc/device-tree/soc/ranges", 4);
    return address == 0xffffffff ? 0x20000000 : address;
}

// Need gpio access to set pull up/down resistors
int get_gpio_map(uint32_t **gpio_map)
{
    int mem_fd;
    void *map;

    debug("get_gpio_map()");
    // Prefer using "/dev/gpiomem" to "/dev/mem"
    if ((mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC)) > 0)
    {
        debug("get_gpio_map() open() /dev/gpiomem, success");
        map = mmap(NULL, GPIO_MAP_BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0);
        if (*((int32_t*)map) < 0) {
            error("get_gpio_map() mmap(), failed");
            return -1;
        } else {
            *gpio_map = (uint32_t *) map;
            debug("get_gpio_map() mmap(), success");
            return 0;
        }
    }
    error("get_gpio_map() open() /dev/gpiomem, failed");

    uint32_t peri_addr =  bcm_host_get_peripheral_address();
    uint32_t gpio_base = peri_addr + GPIO_BASE_OFFSET;
    debug("get_gpio_map() 2 peri_addr %d", gpio_base);

    // mmap the GPIO memory registers
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        error("get_gpio_map() 2 open(), failed");
        return -1;
    }
    uint8_t *gpio_mem;
    if ((gpio_mem = malloc(GPIO_MAP_BLOCK_SIZE + (PAGE_SIZE-1))) == NULL) {
        error("get_gpio_map() 2 malloc(), failed");
        return -1;
    }

    if ((uint32_t)gpio_mem % PAGE_SIZE)
        gpio_mem += PAGE_SIZE - ((uint32_t)gpio_mem % PAGE_SIZE);

    *gpio_map = (uint32_t *)mmap( (void *)gpio_mem, GPIO_MAP_BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED, mem_fd, gpio_base);

    if (*((int32_t*)gpio_map) < 0) {
        error("get_gpio_map() 2 mmap(), failed");
        return -1;
    }

    debug("get_gpio_map() 2 mmap(), success");
    return 0;
}


#define GPPUD_OFFSET        37
#define GPPUDCLK0_OFFSET    38
#define DISABLE_PULLUP_DOWN 0
#define ENABLE_PULLDOWN     1
#define ENABLE_PULLUP       2

static int write_pull_mode(uint32_t *gpio_map, int pin_number, enum pull_mode pull)
{
    uint32_t  clk_bit_to_set = 1 << (pin_number%32);
    uint32_t *gpio_pud_clk = gpio_map + GPPUDCLK0_OFFSET + (pin_number/32);
    uint32_t *gpio_pud = gpio_map + GPPUD_OFFSET;

    if (pull == PULL_NOT_SET)
        return 0;

    // Steps to connect or disconnect pull up/down resistors on a gpio pin:

    // 1. Write to GPPUD to set the required control signal
    if (pull == PULL_DOWN)
        *gpio_pud = (*gpio_pud & ~3) | ENABLE_PULLDOWN;
    else if (pull == PULL_UP)
        *gpio_pud = (*gpio_pud & ~3) | ENABLE_PULLUP;
    else  // pull == PULL_NONE
        *gpio_pud &= ~3;  //DISABLE_PULLUP_DOWN

    // 2. Wait 150 cycles  this provides the required set-up time for the control signal
    usleep(1);

    // 3. Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you wish to modify
    *gpio_pud_clk = clk_bit_to_set;

    // 4. Wait 150 cycles  this provides the required hold time for the control signal
    usleep(1);

    // 5. Write to GPPUD to remove the control signal
    *gpio_pud &= ~3;

    // 6. Write to GPPUDCLK0/1 to remove the clock
    *gpio_pud_clk = 0;

    return 0;
}

static int get_pull_mode(ErlNifEnv *env, ERL_NIF_TERM term, enum pull_mode *pull)
{
    char buffer[16];
    if (!enif_get_atom(env, term, buffer, sizeof(buffer), ERL_NIF_LATIN1))
        return false;

    if (strcmp("not_set", buffer) == 0) *pull = PULL_NOT_SET;
    else if (strcmp("none", buffer) == 0) *pull = PULL_NONE;
    else if (strcmp("pullup", buffer) == 0) *pull = PULL_UP;
    else if (strcmp("pulldown", buffer) == 0) *pull = PULL_DOWN;
    else return false;

    return true;
}

