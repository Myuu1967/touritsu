add_executable(DHA_Pendulum
        DHA_Pendulum.c
        )

# Pull in our (to be renamed) simple get you started dependencies
target_link_libraries(DHA_Pendulum pico_stdlib
                             hardware_uart
                             pico_binary_info
                             hardware_gpio
                             hardware_pwm
                             hardware_adc
                             hardware_i2c
                             hardware_timer
                             )

# create map/bin/hex file etc.
pico_add_extra_outputs(DHA_Pendulum)

# add url via pico_set_program_url
example_auto_set_url(DHA_Pendulum)
