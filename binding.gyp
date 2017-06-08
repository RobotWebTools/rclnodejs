{
  "targets": [
    {
      "target_name": "rclnodejs",
      "variables": {
        "ROS2_INSTALL_PATH": "<!(echo $AMENT_PREFIX_PATH)",
      },
      "sources": [
        "addon.cpp",
        "./src/executor.cpp",
        "./src/handle_manager.cpp",
        "./src/rcl_bindings.cpp",
        "./src/rcl_bindings_publisher.cpp",
        "./src/rcl_handle.cpp",
        "./src/shadow_node.cpp",
      ],
      "include_dirs": [
        ".",
        "./include",
         "<!(node -e \"require('nan')\")",
         "<(ROS2_INSTALL_PATH)/include/",
      ],
      "cflags!": [
        "-fno-exceptions"
      ],
      "cflags": [
        "-std=c++11",
        "-fstack-protector-strong",
        "-fPIE -fPIC",
        "-O2 -D_FORTIFY_SOURCE=2",
        "-Wformat -Wformat-security"
      ],
      "cflags_cc!": [
        "-fno-exceptions"
      ],
      "libraries": [
        "-lrcl",
        "-lrcutils",
        "-lstd_msgs__rosidl_typesupport_c",
        "-L<(ROS2_INSTALL_PATH)/lib"
      ],
      "xcode_settings": {
        "OTHER_CFLAGS": [
          "-std=c++11"
        ]
      },
      "conditions": [
        [
          "OS!=\"win\"",
          {
            "cflags+": [
              "-std=c++14"
            ],
            "cflags_c+": [
              "-std=c++14"
            ],
            "cflags_cc+": [
              "-std=c++14"
            ]
          }
        ],
        [
          "OS==\"mac\"",
          {
            "xcode_settings": {
              "OTHER_CPLUSPLUSFLAGS": [
                "-std=c++11",
                "-stdlib=libc++"
              ],
              "OTHER_LDFLAGS": [
                "-stdlib=libc++"
              ],
              "GCC_ENABLE_CPP_EXCEPTIONS": "YES",
              "MACOSX_DEPLOYMENT_TARGET": "10.8"
            }
          }
        ]
      ]
    }
  ]
}
