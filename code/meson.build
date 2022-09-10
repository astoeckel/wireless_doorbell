project('avr_remote_doorbell', 'cpp',
    default_options: ['cpp_std=c++14'])

inc_local = include_directories('src')

exe_main_atmega168 = executable(
    'main_atmega168',
    sources: [
        'src/utils/buffer.cpp',
        'src/utils/byte_suffix.cpp',
        'src/main_atmega168.cpp',
        'src/platform.cpp',
        'src/uart.cpp',
        'src/si4463.cpp',
    ],
    include_directories: [
        inc_local,
    ]
)

exe_avr_objcopy = find_program('avr-objcopy')
exe_avrdude = find_program('avrdude', required: false)

hex_main_atmega168 = custom_target(
    'main_atmega168.hex',
    input: exe_main_atmega168,
    output: 'main_atmega168.hex',
    command: [
        exe_avr_objcopy,
        '-j', '.text', '-j', '.data', '-O', 'ihex',
        '@INPUT@', '@OUTPUT@',
    ],
)

if exe_avrdude.found()
    run_target(
        'flash',
        command: [
            exe_avrdude,
            '-c', 'linuxspi' ,
            '-P', '/dev/spidev0.0',
            '-p', 'm168',
            '-U', 'flash:w:@0@:i'.format(hex_main_atmega168.full_path())
        ],
        depends: hex_main_atmega168,
    )
endif
