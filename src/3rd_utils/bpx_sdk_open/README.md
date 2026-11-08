# BPX SDK 运行时依赖

从本机已有 `/home/yuling/utils/bpx_sdk_open` 搬入驱动使用的公开头文件、
Linux aarch64 / x86_64 共享库及原始许可证/NOTICE。未修改厂商 API 与二进制。
`dog_driver` 默认直接使用此目录，不需要其他 ROS 工作空间或用户主目录 SDK。
可用 `BPX_SDK_ROOT` CMake 参数显式覆盖。SDK 版本见 `include/bpx_sdk_version.h`。

本目录没有 SDK 实现源码；共享库为厂商预编译产物，须与目标 CPU 架构匹配。
