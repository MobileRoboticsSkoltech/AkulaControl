# Virtual port

Pinout:
1. In RCC set HSE to "Crystal/Ceramic Resonator", leave LSE disabled
2. Set USB_OTG_FS mode to "Device_Only"
3. In USB_DEVICE set class for FS IP to "Communication Device Class (Virtual Port Com)"
4. Set pin PB9 to GPIO_Output

Clock Configuration:
1. Set input frequency to 8MHz
2. In PLL Source Mux select HSE
3. In System Clock Mux select PLLCLK
4. Set HCLK to 168MHz
5. Automatically recalculate the scheme

STLinkv2 (mini-USB) is (usually) ttyACM0
Virtual port (micro-USB) is (usually) ttyACM1
