# Cyberlamp

![lamp photo](lamp_open.png)

An experiment about creating a tetrachromatic lamp, including infrared and ultraviolet light sources.

The LEDs are driven by ICs designed to control columns on hub75 matrices. Each LED can be toggled separately.

The onboard mcu CH582 can do bluetooth, so the lamps could be remotely controlled, atleast in theory.

In practice, i should've used PWM-enabled pin for `output enable` pin. Software PWM is glitchy, and becomes
unusable when handling bluetooth at the same time.

Next revision will most likely just use some dedicated LED drivers, and their PWM
will be controlled by dedicated PWM pin(s).

## Interface

The lamp is dedicated to work with [i2current](https://github.com/cr1tbit/i2current),
so it becomes fully battery powered. USB, power and I2C are routed through the 
side walls via stripped down CAT5 differential pairs. Voltage drop is not an issue,
I actually had to modify the powerboard to deliver ~4.2V - because the drivers would
not respond to 3.3v logic signals when powered by 5V.