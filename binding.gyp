{
  'target_defaults': {
    'default_configuration': 'Release',
    'configurations': {
      'Debug': {
        'defines': ['SPDLOG_DEBUG_ON'],
      },
    }
  },
  'targets': [
    {
      'target_name': 'rclnodejs',
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
        'src/third_party/spdlog/include/',
        "<!(node -e \"require('nan')\")",
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
        '-lrcl_yaml_param_parser',
        '-lrmw',
        '-lrosidl_generator_c',
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
            'include_dirs': [
              "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/include/ ') + '/include/')\")",
            ],
            'library_dirs': [
              "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/:/g, '/lib/ ') + '/lib/')\")",
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
              "<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '\\\include ').replace(/\\\/g, '/') + '/include')\")",
            ],
            'sources': [
              './src/third_party/dlfcn-win32/dlfcn.c',
            ],
            'msvs_settings': {
              'VCCLCompilerTool': {
                'ExceptionHandling': '2', # /EHsc
              },
              'VCLinkerTool': {
                'AdditionalDependencies': ['psapi.lib'],
                'AdditionalLibraryDirectories': ["<!@(node -e \"console.log(process.env.AMENT_PREFIX_PATH.replace(/;/g, '\\\lib ').replace(/\\\/g, '/') + '/lib')\")",],
              }
            }
          }
        ],
        [
          'OS=="mac"',
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
        ]
      ]
    }
  ]
}
