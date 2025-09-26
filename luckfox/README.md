Shared internet laptop send to luckfox

# Setup shared internet laptop/pc Ubuntu 22.04

```bash
sudo ip addr flush dev enxf21280edf227
```

```bash
sudo ip addr add 172.32.0.1/16 dev enxf21280edf227
```

```bash
sudo ip link set enxf21280edf227 up
```

# Setup recevei internet Luckfoxpico(use firmware Ubuntu 22.04)

```bash
sudo dhclient usb0
```

```bash
udhcpc -i usb0
```

```bash
sudo date -s "2025-09-21 22:10:00"
```

```bash
ping -c 4 8.8.8.8
```

```bash
ping -c 4 google.com
```

```bash
sudo apt update
```


# Setup python3 library 
```bash
pip3 install python-periphery
```


# Send file or folder to Luckfoxpico 
```bash
scp -r ../src/ pico@10.42.0.62:/home/pico/
```


# Build languge gcc on host and send Luckfoxpico
```bash
../tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc --static main.c -o main
```

# Tutorial

## GPIO

### Read number pin is Luckfoxpico

Example: GPIO1_C7_d

- Bank: 1
- Group: [A = 0, B = 1, C = 2, D = 3]
- x: 7

GPIO pin number calculation formula: pin = bank * 32 + number

GPIO group number calculation formula: number = group * 8 + X

Therefore: pin = bank * 32 + (group * 8 + X)

Use 7 led gpio output

LED 1 - GPIO4_C1_z - 145
LED 2 - GPIO4_C0_z - 144
LED 3 - GPIO4_A4_d - 132
LED 4 - GPIO4_A3_d - 131
LED 5 - GPIO4_A2_d - 130
LED 6 - GPIO4_A6_d - 134
LED 7 - GPIO4_B0_d - 136
LED 8 - GPIO4_B1_d - 137

Use Encoder Volune gpio input

CLK - GPIO1_C6_d - 53
DT  - GPIO1_C5_d - 52
SW  - GPIO1_C4_d - 54

### Test function  number 

> echo 145 | sudo tee /sys/class/gpio/export

> echo out | sudo tee /sys/class/gpio/gpio145/direction 

> echo 1 | sudo tee /sys/class/gpio/gpio145/value

> echo 0 | sudo tee /sys/class/gpio/gpio145/value

> echo 145 | sudo tee /sys/class/gpio/unexport

### Test full nè 