# stepper_motor_driver
An attempt for a reusable stepper motor driver framework.  Used in this project with a Tiva Microcontroller, a BOOST-DRV8711 stepper driver, and Mitsumi M42SP stepper motor.  It should be easily portable to other microcontrollers (with its own stepper controller module made up of internal peripherals like timer, gpio, etc.) and stepper motor drivers.

The speed ramping algorithm that was implemented is based on the AVR446 application note titled: Linear speed control of stepper motor.  Except that this project uses a Cortex-M4F controller with floating-point unit.
