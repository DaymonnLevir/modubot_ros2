# Upstream provenance

This package is vendored from `ros-navigation/navigation2`, tag `1.1.20`,
commit `a097086719c88f781aa59788eca29ac6ca5e56db`.

It is built inside the ModuBot workspace because the ROS 2 Humble ARM64 binary
of `nav2_mppi_controller` can terminate with `SIGILL` while initializing the
MPPI noise generator on NVIDIA Jetson systems. A native build uses the
instruction set supported by the target processor and overrides the binary
package through the workspace overlay.

The local CMake file disables the upstream test targets by default to keep
on-robot builds small. Set `-DMODUBOT_BUILD_VENDOR_TESTS=ON` when those tests
are explicitly required.

Upstream repository: <https://github.com/ros-navigation/navigation2>
