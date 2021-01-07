# Ceasar SpotMicro configuration for CHAMP controller
Refer to [CHAMP controller](https://github.com/chvmp/champ)

## Requirements
I2C PCA9685 Controller: https://github.com/cholan2100/i2c_pwm_board
CHAMP base: https://github.com/chvmp/champ

20 kgcm servos are recommended, cheaper MG996r are underwhelming to support dual leg movements. For MG996r i recommend using other gait implementation here,
https://github.com/cholan2100/ceasar.git

## Ceasar description
https://github.com/cholan2100/spotmicro_description


## Run the controller
### 1. PCA9685 controller
    Configure config/servos_calibration.yaml

### 2. Run CHAMP controller

    roslaunch ceasar_config bringup.launch hardware_connected:=true

### 3. Eun remote controller
    roslaunch champ_teleop teleop.launch joy:=true

## NOTE:
Refer to original CHAMP controller for detailed documentation
