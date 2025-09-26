#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include "gpio/gpio.h"

volatile sig_atomic_t  isTrue = 1;

void handle(int sig){
    isTrue = 0;
}

int main() {
    signal(SIGINT, handle);
    // output
    GPIO.gpio_export(GPIO4_C1_Z);
    GPIO.gpio_export(GPIO4_C0_Z);
    GPIO.gpio_export(GPIO4_A4_D);
    GPIO.gpio_export(GPIO4_A3_D);
    GPIO.gpio_export(GPIO4_A2_D);
    GPIO.gpio_export(GPIO4_A6_D);
    GPIO.gpio_export(GPIO4_B0_D);
    GPIO.gpio_export(GPIO4_B1_D);
    GPIO.gpio_mode(GPIO4_C1_Z, OUTPUT);
    GPIO.gpio_mode(GPIO4_C0_Z, OUTPUT);
    GPIO.gpio_mode(GPIO4_A4_D, OUTPUT);
    GPIO.gpio_mode(GPIO4_A3_D, OUTPUT);
    GPIO.gpio_mode(GPIO4_A2_D, OUTPUT);
    GPIO.gpio_mode(GPIO4_A6_D, OUTPUT);
    GPIO.gpio_mode(GPIO4_B0_D, OUTPUT);
    GPIO.gpio_mode(GPIO4_B1_D, OUTPUT);
    while(isTrue){
        GPIO.gpio_level(GPIO4_C1_Z, HIGH);
        GPIO.gpio_level(GPIO4_C0_Z, HIGH);
        GPIO.gpio_level(GPIO4_A4_D, HIGH);
        GPIO.gpio_level(GPIO4_A3_D, HIGH);
        GPIO.gpio_level(GPIO4_A2_D, HIGH);
        GPIO.gpio_level(GPIO4_A6_D, HIGH);
        GPIO.gpio_level(GPIO4_B0_D, HIGH);
        GPIO.gpio_level(GPIO4_B1_D, HIGH);
        sleep(1);
        GPIO.gpio_level(GPIO4_C1_Z, LOW);
        GPIO.gpio_level(GPIO4_C0_Z, LOW);
        GPIO.gpio_level(GPIO4_A4_D, LOW);
        GPIO.gpio_level(GPIO4_A3_D, LOW);
        GPIO.gpio_level(GPIO4_A2_D, LOW);
        GPIO.gpio_level(GPIO4_A6_D, LOW);
        GPIO.gpio_level(GPIO4_B0_D, LOW);
        GPIO.gpio_level(GPIO4_B1_D, LOW);
        sleep(1);
    }
    GPIO.gpio_unexport(GPIO4_C1_Z);
    GPIO.gpio_unexport(GPIO4_C0_Z);
    GPIO.gpio_unexport(GPIO4_A4_D);
    GPIO.gpio_unexport(GPIO4_A3_D);
    GPIO.gpio_unexport(GPIO4_A2_D);
    GPIO.gpio_unexport(GPIO4_A6_D);
    GPIO.gpio_unexport(GPIO4_B0_D);
    GPIO.gpio_unexport(GPIO4_B1_D);
    return 0;
}
