#ifndef _GPIO_H_
#define _GPIO_H_ 

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#define GPIO_MAX 200

typedef enum {
    GPIO0_A4_D = 4,
    GPIO1_A2_D = 34,
    GPIO1_B2_D = 42,
    GPIO1_B3_D = 43,
    GPIO1_C0_D = 48,
    GPIO1_C1_D = 49,
    GPIO1_C2_D = 50,
    GPIO1_C3_D = 51,
    GPIO1_C4_D = 52,
    GPIO1_C5_D = 53,
    GPIO1_C6_D = 54,
    GPIO1_C7_D = 55,
    GPIO1_D0_D = 56,
    GPIO1_D1_D = 57,
    GPIO1_D2_D = 58,
    GPIO1_D3_D = 59,
    GPIO4_A2_D = 130,
    GPIO4_A3_D = 131,
    GPIO4_A4_D = 132,
    GPIO4_A6_D = 134,
    GPIO4_B0_D = 136,
    GPIO4_B1_D = 137,
    GPIO4_C0_Z = 144,
    GPIO4_C1_Z = 145,
} gpio_pin_t;

typedef enum {
    HIGH = 1,
    LOW = 0
} gpio_level_t;

typedef enum {
    OUTPUT = 0,
    INPUT = 1
} gpio_mode_t;

typedef struct {
    void (*gpio_export)(gpio_pin_t pin);
    void (*gpio_mode)(gpio_pin_t pin , gpio_mode_t mode);
    void (*gpio_level)(gpio_pin_t pin, gpio_level_t level);
    int (*gpio_value)(gpio_pin_t pin);
    void (*gpio_unexport)(gpio_pin_t pin);
} gpio_t;

void export(gpio_pin_t pin);
void mode(gpio_pin_t pin, gpio_mode_t mode);
void level(gpio_pin_t pin, gpio_level_t level);
int value(gpio_pin_t pin);
void unexport(gpio_pin_t pin);

extern gpio_t GPIO;

#endif // _GPIO_H_