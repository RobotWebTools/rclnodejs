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
    'node_major_version': '<!(node -p \"process.versions.node.split(\'.\')[0]\")',
  },
  'targets': [
    {
      'target_name': 'rclnodejs',
      'sources': [
        './src/addon.cpp',
        './src/executor.cpp',
        './src/handle_manager.cpp',
        './src/rcl_action_client_bindings.cpp',
        './src/rcl_action_goal_bindings.cpp',
        './src/rcl_action_server_bindings.cpp',
        './src/rcl_bindings.cpp',
        './src/rcl_client_bindings.cpp',
        './src/rcl_context_bindings.cpp',
        './src/rcl_graph_bindings.cpp',
        './src/rcl_guard_condition_bindings.cpp',
        './src/rcl_handle.cpp',
        './src/rcl_lifecycle_bindings.cpp',
        './src/rcl_logging_bindings.cpp',
        './src/rcl_names_bindings.cpp',
        './src/rcl_node_bindings.cpp',
        './src/rcl_publisher_bindings.cpp',
        './src/rcl_serialization_bindings.cpp',
        './src/rcl_service_bindings.cpp',
        './src/rcl_subscription_bindings.cpp',
        './src/rcl_time_point_bindings.cpp',
        './src/rcl_timer_bindings.cpp',
        './src/rcl_utilities.cpp',
        './src/shadow_node.cpp',
        './third_party/ref-napi/src/ref_napi_bindings.cpp',
      ],
      'include_dirs': [
        '.',
        './third_party/ref-napi/src',
        '<(ros_include_root)',
        "<!@(node -p \"require('node-addon-api').include\")",
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
        '-lrcpputils',
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
            'conditions': [
              [
                'node_major_version >= 23', {
                  'cflags_cc': [
                    '-std=c++20'
                  ]
                }
              ],
              [
                'node_major_version < 23', {
                  'cflags_cc': [
                    '-std=c++17'
                  ]
                }
              ]
            ]
          }
        ],
        [
          'OS=="win"', {
            'defines': [
              'OS_WINDOWS'
            ],
            'conditions': [
              [
                'node_major_version >= 23', {
                  'cflags_cc': [
                    '-std=c++20'
                  ]
                }
              ],
              [
                'node_major_version < 23', {
                  'cflags_cc': [
                    '-std=c++17'
                  ]
                }
              ]
            ],
            'include_dirs': [
              './src/third_party/dlfcn-win32/',
            ],
            'libraries': [
              '-lrmw_fastrtps_cpp',
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
          # After Humble, e.g., Jazzy, Kilted.
          'ros_version > 2205', {
            'sources': [
              './src/rcl_event_handle_bindings.cpp',
              './src/rcl_type_description_service_bindings.cpp',
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
