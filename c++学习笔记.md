随记，不成体系

对于构建一个类的对象，我一般有两种方式
std::unique_ptr:当需要堆分配、明确所有权转移时使用
std::shared_ptr:当需要共享所有权时使用
默认用 unique_ptr，因为它开销小、逻辑简单。
只有当真的需要多个地方共享同一个对象时，才用 shared_ptr。

使用Eigen 库
#include <Eigen/Dense>
Eigen::Vector4d x;
x << 0.1, 0.1, -0.1, -0.1;

#include <array>
std::array<double, 4> x = {0.1, 0.1, -0.1, -0.1};

#include <algorithm>  
rc_vx = std::clamp(rc_vx, -0.2, 0.2);