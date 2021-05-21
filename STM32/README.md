# Virtual port

Pinout:

1. In RCC set HSE to "Crystal/Ceramic Resonator", leave LSE disabled
2. Set USB_OTG_FS mode to "Device_Only"
3. In USB_DEVICE set class for FS IP to "Communication Device Class (Virtual Port Com)"
4. Set pin PB9 to GPIO_Output to fix warning

Clock Configuration:

1. Set input frequency to 8MHz
2. In PLL Source Mux select HSE
3. In System Clock Mux select PLLCLK
4. Set HCLK to 168MHz
5. Automatically recalculate the scheme

STLinkv2 (mini-USB) is (usually) ttyACM0, Virtual port (micro-USB) is (usually) ttyACM1

# Setting Clion for stm32 development (optional)

Preparing the environment:

1. Download stm32CubeMX from https://www.st.com/en/development-tools/stm32cubemx.html
2. Install OpenOCD using command sudo apt install openocd
3. Install ARM toolchain using cammand sudo apt install gcc-arm-none-eabi
4. In the settings (Ctrl+Alt+S) go to "Build, Execution, Deployment", "Embedded Development"
5. For "OpenOCD Location" set "/usr/bin/openocd" or something custom
6. For "Stm32CubeMX Location" set path to Stm32CubeMX binary where you installed it to

Setting up a project:

1. Select "File -> New Project"
2. On the left panel choose "STM32CubeMX", set the location and create the project
3. Clion will create .ioc file and will suggest to open it with STM32CubeMX (in case there is no file, check the settings and create a project again)
4. Click "Open with STM32CubeMX"
5. In the STM32CubeMX window click on the board tab (near the home tab) and choose the board you want (in our case it is stm32f407G-DISC1)
6. Set the pinout and clock configuration you want then move to the "Project Manager" tab
7. In the "Project Name" set the same name you chose for the Clion project
8. In the "Project Location" set path to the project one directory upper (path should not include the name of the project)
9. In the "Toolchain / IDE" drop list select "STM32CubeISE" and be sure to have "Generate Under Root" check box enabled
10. Click "GENERATE CODE" and agree to all the overwriting suggestions
11. Return to Clion and select "stm324discovery.cfg" (in our case) in the "Board Config Files" window
12. (optional) If the "Board Config Files" window didn't show up select "Edit Configurations..." near the "Run" button, select "OpenOCD Download & Run -> OCD 'project_name'" and in the field "Board config file" click either "Assist..." or "..." to select config manually (configs can be found in "/usr/share/openocd/scripts/board")
13. In the "Edit Configurations... -> Download & Run -> OCD 'project_name'" be sure have selected "'project_name'.elf" in the "Target" and "Executable" drop boxes
14. Build the project and click "Run" icon
15. (optional) In case downloading software (run) is not successful, the problem can be the wrong ST-Link version in the config, so, to change it go to "/usr/share/openocd/scripts/board", open a config with sudo (in our case it is stm324discovery.cfg) and change the "source" line to "source [find interface/stlink-v2-1.cfg]"
