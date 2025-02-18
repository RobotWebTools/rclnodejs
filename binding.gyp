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
    'runtime%': 'node',
    'ros_lib_dir': "<!(node -p \"require('./scripts/config.js').getROSLibPath()\")",
    'ros_include_root': "<!(node -p \"require('./scripts/config.js').getROSIncludeRootPath()\")",
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
        '<(ros_include_root)',
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
              '-std=c++20'
            ]
          }
        ],
        [
          'OS=="win"', {
            'defines': [
              'OS_WINDOWS'
            ],
            'cflags_cc': [
              '-std=c++20'
            ],
            'include_dirs': [
              './src/third_party/dlfcn-win32/',
            ],
            'msvs_settings': {
              'VCCLCompilerTool': {
                'ExceptionHandling': '2', # /EHsc
              },
              'VCLinkerTool': {
                'AdditionalDependencies': ['psapi.lib'],
                'AdditionalLibraryDirectories': ['<(ros_lib_dir)'],
              }
            }
          }
        ],
        [
          # TODO - macos is no longer a tier-1 ROS platform and we have no binary ROS builds to test for Humble & Rolling
          'OS=="mac"', {
            'defines': [
              'OS_MACOS'
            ],
            'xcode_settings': {
              'GCC_ENABLE_CPP_EXCEPTIONS': 'YES',
              'CLANG_CXX_LIBRARY': 'libc++',
              'MACOS_DEPLOYMENT_TARGET': '10.12',
              'CLANG_CXX_LANGUAGE_STANDARD': 'c++17'
            }
          }
        ],
        [
          'ros_version <= 1911', {
            'libraries': [
              '-lrosidl_generator_c'
            ],
            'libraries!': [
              '-lrosidl_runtime_c'
            ]
          }
        ],
        [
          # After Galactic, e.g., Humble, Jazzy, Rolling.
          'ros_version > 2105', {
            'include_dirs': [
              "<!@(node -p \"require('./scripts/config.js').getIncludePaths().forEach(p => console.log(JSON.stringify(p)))\")"
            ],
            'library_dirs': [
              '<(ros_lib_dir)',
            ]
          }
        ],
        [
          'runtime=="electron"', {
            "defines": ["NODE_RUNTIME_ELECTRON=1"]
          }
        ],
      ]
    }
  ]
}
