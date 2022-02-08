# Virtual port

### Pinout:

1. In RCC set HSE to "**Crystal/Ceramic Resonator**", leave LSE disabled
2. Set USB_OTG_FS mode to "**Device_Only**"
3. In USB_DEVICE set class for FS IP to "**Communication Device Class (Virtual Port Com)**"
4. Set pin PB9 to GPIO_Output to fix warning

### Clock Configuration:

1. Set input frequency to 8MHz
2. In PLL Source Mux select HSE
3. In System Clock Mux select PLLCLK
4. Set HCLK to 168MHz
5. Automatically recalculate the scheme

STLinkv2 (mini-USB) is (usually) ttyACM0, Virtual port (micro-USB) is (usually) ttyACM1

### Setting up udev rule (optional):

In order to have constant name for serial port (instead of ttyACM[0-9]) you need to create custom udev rule:

1. Connect all the cables to your stm32 board with configured virtual port
2. Find all the serial port names (usually ttyACM0 and ttyACM1) and enter commands in the terminal: "**udevadm info -a /dev/ttyACM0**" and "**udevadm info -a /dev/ttyACM1**"
3. Find the "**vendorId**" (in our case it is **0483**) and something you can use to differentiate between ports (in our case it will be "**productId**": "**374b**" for StLink and "**5740**" for virtual port); this also can be done using command "**lsusb**". To use two or more such boards you can also add "**serial**" field (**375C344F3339** in our case)
4. Create custom udev rule using "**sudo nano /etc/udev/rules.d/49-custom.rules**"
5. Add line (enter your own idProduct or other parameters and tty name you want to use for vitual port)
```
KERNEL=="ttyACM[0-9]*", SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="375C344F3339", MODE="0666", SYMLINK="ttySensorsSTM32"
```
6. Unplug and plug again virtual port micro usb

# Setting timer for IMU data capturing

### Pinout:

1. In TIM5 set Clock Source to "**Internal Clock**"
2. Set Channel1 to "**Output Compare CH1**"

### Timer parameters

1. In "**TIM5 -> Parameter Settings**" set "**Prescaler**" (3-1 in our case)
2. Set "**Counter Mode**" to "**Up**"
3. Set "**Counter Period**" to value that you need (28000000-1 in our case) taking into account the "**APB1 timer clocks**".

# Additional information

Everything related to clock values and setting up IDE (CLion) to work with stm32 can be found in the "**STM32**" directory README.md file
