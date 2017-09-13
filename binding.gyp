{
  'targets': [
    {
      'target_name': 'rclnodejs',
      'variables': {
        'ROS2_INSTALL_PATH': '<!(echo $AMENT_PREFIX_PATH)',
      },
      'sources': [
        './src/addon.cpp',
        './src/executor.cpp',
        './src/handle_manager.cpp',
        './src/rcl_bindings.cpp',
        './src/rcl_handle.cpp',
        './src/rcl_utilities.cpp',
        './src/shadow_node.cpp',
      ],
      'include_dirs': [
        '.',
        '<!(node -e \'require("nan")\')',
        '<(ROS2_INSTALL_PATH)/include/',
      ],
      'cflags!': [
        '-fno-exceptions'
      ],
      'cflags': [
        '-fstack-protector-strong',
        '-fPIE -fPIC',
        '-O2 -D_FORTIFY_SOURCE=2',
        '-Wformat -Wformat-security'
      ],
      'cflags_cc!': [
        '-fno-exceptions'
      ],
      'libraries': [
        '-lrcl',
        '-lrcutils',
        '-L<(ROS2_INSTALL_PATH)/lib'
      ],
      'conditions': [
        [
          'OS=="linux"', {
            'defines': [
              'OS_LINUX'
            ],
            'cflags_cc': [
              '-std=c++14'
            ]
          }
        ],
        [
          'OS=="win"',
          {
          }
        ],
        [
          'OS=="mac"',
          {
            'defines': [
              'OS_MACOS'
            ],
            'xcode_settings': {
              'OTHER_CPLUSPLUSFLAGS': [
                '-std=c++14',
                '-stdlib=libc++'
              ],
              'GCC_ENABLE_CPP_EXCEPTIONS': 'YES',
            }
          }
        ]
      ]
    }
  ]
}
