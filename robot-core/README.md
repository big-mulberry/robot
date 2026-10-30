# robot-core

## 独立编译 examples

在 robot-core 目录运行：

```bash
cmake -S examples -B examples/build
cmake --build examples/build
./examples/build/vector_basics
```

也可以在 examples 目录运行 `cmake -S . -B build`。

examples 使用独立的 CMake 项目和 C++17，直接引用 ../include 下的公共头文件，无需先配置根目录项目或 tests。
当前 vector_basics 是可运行的占位示例；Vector3d 实现后再添加向量演示。
