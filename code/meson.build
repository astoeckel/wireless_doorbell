#   AVR/Si4463 Wireless Doorbell
#   Copyright (C) 2022  Andreas Stöckel
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <https://www.gnu.org/licenses/>.

project('wireless_doorbell', 'cpp',
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
