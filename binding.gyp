{
  'target_defaults': {
    'default_configuration': 'Release',
    'configurations': {
      'Debug': {
        'defines': ['DEBUG_ON'],
      },
    }
  },
  'variables': {
    'ros_version': '<!(node scripts/ros_distro.js)',
  },
  'targets': [
    {
      'target_name': 'rclnodejs',
      'sources': [
        './src/addon.cpp',
        './src/executor.cpp',
        './src/handle_manager.cpp',
        './src/rcl_action_bindings.cpp',
        './src/rcl_bindings.cpp',
        './src/rcl_handle.cpp',
        './src/rcl_lifecycle_bindings.cpp',
        './src/rcl_utilities.cpp',
        './src/shadow_node.cpp',
      ],
      'include_dirs': [
        '.',
        "<!(node -e \"require('nan')\")",
      ],
      'cflags!': [
        '-fno-exceptions'
      ],
      'cflags': [
        '-fstack-protector-strong',
        '-fPIE -fPIC',
        '-O2 -D_FORTIFY_SOURCE=2',
        '-Wformat -Wformat-security -Wextra -Wno-cast-function-type'
      ],
      'cflags_cc!': [
        '-fno-exceptions'
      ],
      'libraries': [
        '-lrcl',
        '-lrcl_action',
        '-lrcl_lifecycle',
        '-lrcutils',
        '-lrcl_yaml_param_parser',
        '-lrmw',
        '-lrosidl_runtime_c',
      ],
      'defines': [
        'ROS_VERSION=<(ros_version)'
      ],
      'conditions': [
        [
          'OS=="linux"', {
            'defines': [
              'OS_LINUX'
            ],
            'cflags_cc': [
              '-std=c++14'
            ],
            'include_dirs': 
            [
              "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/ ') + '/include/')\")",
            ],
            'library_dirs': [
              "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/lib/ ') + '/lib/')\")",
            ],
            'conditions': [
              [
                'ros_version > 2105', # Humble, Rolling, ...
                {
                  'include_dirs': 
                  [
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/ ') + '/include/')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/rcl/ ') + '/include/rcl')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/rcutils/ ') + '/include/rcutils/')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/rmw/ ') + '/include/rmw/')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/rcl_yaml_param_parser/ ') + '/include/rcl_yaml_param_parser/')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/rosidl_typesupport_interface/ ') + '/include/rosidl_typesupport_interface/')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/rcl_action/ ') + '/include/rcl_action/')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/action_msgs/ ') + '/include/action_msgs/')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/unique_identifier_msgs/ ') + '/include/unique_identifier_msgs/')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/builtin_interfaces/ ') + '/include/builtin_interfaces/')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/rcl_lifecycle/ ') + '/include/rcl_lifecycle/')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/lifecycle_msgs/ ') + '/include/lifecycle_msgs/')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/rosidl_runtime_c/ ') + '/include/rosidl_runtime_c/')\")",
                  ],
                }
              ],
            ],
          }
        ],
        [
          'OS=="win"',
          {
            'defines': [
              'OS_WINDOWS'
            ],
            'cflags_cc': [
              '-std=c++14'
            ],
            'include_dirs': [
              './src/third_party/dlfcn-win32/',
              "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include ').replace(/\\\/g, '/') + '/include')\")",
            ],
            'msvs_settings': {
              'VCCLCompilerTool': {
                'ExceptionHandling': '2', # /EHsc
              },
              'VCLinkerTool': {
                'AdditionalDependencies': ['psapi.lib'],
                'AdditionalLibraryDirectories': ["<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '\\\lib ').replace(/\\\/g, '/') + '/lib')\")",],
              }
            },
            'conditions': [
              [ 
                'ros_version > 2105', # Humble, Rolling, ... TODO - not tested due to broken setup_ros v3.3 action on windows 
                {
                  'include_dirs': 
                  [
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include/rcl ').replace(/\\\/g, '/') + '/include/rcl')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include/rcutils ').replace(/\\\/g, '/') + '/include/rcutils')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include/rmw ').replace(/\\\/g, '/') + '/include/rmw')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include/rcl_yaml_param_parser ').replace(/\\\/g, '/') + '/include/rcl_yaml_param_parser')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include/rosidl_runtime_c ').replace(/\\\/g, '/') + '/include/rosidl_runtime_c')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include/rosidl_typesupport_interface ').replace(/\\\/g, '/') + '/include/rosidl_typesupport_interface')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include/rcl_action ').replace(/\\\/g, '/') + '/include/rcl_action')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include/action_msgs ').replace(/\\\/g, '/') + '/include/action_msgs')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include/unique_identifier_msgs ').replace(/\\\/g, '/') + '/include/unique_identifier_msgs')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include/builtin_interfaces ').replace(/\\\/g, '/') + '/include/builtin_interfaces')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include/rcl_lifecycle ').replace(/\\\/g, '/') + '/include/rcl_lifecycle')\")",
                    "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '/include/lifecycle_msgs ').replace(/\\\/g, '/') + '/include/lifecycle_msgs')\")",
                  ],
                }
              ]
            ]
          }
        ],
        [
          'OS=="mac"',
          # TODO - macos is no longer a tier-1 ROS platform and we have no binary ROS builds to test for Humble & Rolling
          {
            'defines': [
              'OS_MACOS'
            ],
            'include_dirs': [
              "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/ ') + '/include/')\")",
            ],
            'library_dirs': [
                "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/lib/ ') + '/lib/')\")",
            ],
            'xcode_settings': {
              'GCC_ENABLE_CPP_EXCEPTIONS': 'YES',
              'CLANG_CXX_LIBRARY': 'libc++',
              'MACOS_DEPLOYMENT_TARGET': '10.12',
              'CLANG_CXX_LANGUAGE_STANDARD': 'c++14'
            }
          }
        ],
        [
          'ros_version<=1911',
          {
            'libraries': [
              '-lrosidl_generator_c'
            ],
            'libraries!': [
              '-lrosidl_runtime_c'
            ]
          }
        ]
      ]
    }
  ]
}
