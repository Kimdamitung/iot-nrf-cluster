#include "gpio/gpio.h"

static int gpio_fd[GPIO_MAX] = {0};

void export(gpio_pin_t pin){
    FILE *file = fopen("/sys/class/gpio/export", "w");
    if(file == NULL){
        perror("Failed to open GPIO export pins");
        return;
    }
    fprintf(file, "%d", pin);
    fclose(file);
    char path[35];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    int fd = open(path, O_WRONLY);
    if(fd < 0)
        perror("Open pin value error");
    else
        gpio_fd[pin] = fd;
}

void mode(gpio_pin_t pin, gpio_mode_t mode){
    char path[35];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    FILE *file = fopen(path, "w");
    if(file == NULL){
        perror("Failed to set mdoe output pins");
        return;
    }
    fprintf(file, mode == OUTPUT ? "out" : "in");
    fclose(file);

}

void level(gpio_pin_t pin, gpio_level_t level){
    int fd = gpio_fd[pin];
    if(fd > 0)
        write(fd, level == HIGH ? "1" : "0", 1);
}

int value(gpio_pin_t pin){
    int fd = gpio_fd[pin];
    if(fd < 0) 
        return -1; 
    char value;
    lseek(fd, 0, SEEK_SET);
    if(read(fd, &value, 1) != 1) 
        return -1;
    return (value == '1') ? 1 : 0;
}

void unexport(gpio_pin_t pin){
    if(gpio_fd[pin] > 0) 
        close(gpio_fd[pin]);
    FILE *file = fopen("/sys/class/gpio/unexport", "w");
    if(file == NULL){
        perror("Failed to open GPIO export pins");
        return;
    }
    fprintf(file, "%d", pin);
    fclose(file);
}

gpio_t GPIO = {
    .gpio_export = export,
    .gpio_mode = mode,
    .gpio_level = level,
    .gpio_value = value,
    .gpio_unexport = unexport,
};