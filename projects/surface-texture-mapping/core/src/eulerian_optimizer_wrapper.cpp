/**
 * Eulerian Shape Optimizer Wrapper
 *
 * 这个文件解决跨编译单元的typedef模板链接问题
 *
 * 问题根源：
 * - typedef Vector3 Euclidean;
 * - 链接器在处理 Geometry<Euclidean>* 和 Geometry<Vector3>* 时可能生成不同的符号
 * - 虽然C++标准保证它们是同一类型，但模板名称修饰（name mangling）可能不一致
 *
 * 解决方案：
 * - 在这个编译单元中显式实例化所需的构造函数调用
 * - 确保链接器能找到正确的符号
 */

#include "geometry.h"
#include "eulerian_shape_optimizer.h"
#include <memory>

namespace SurfaceTextureMapping {
namespace EulerianOptimizerWrapper {

/**
 * 包装函数：创建EulerianShapeOptimizer
 *
 * 这个函数在同一个编译单元中完成以下操作：
 * 1. 接受 void* 参数（来自 Core_Geometry* typedef）
 * 2. 转换为 Geometry<Euclidean>*
 * 3. 调用 EulerianShapeOptimizer 构造函数
 *
 * 由于所有操作都在同一个编译单元，编译器会正确地实例化模板
 */
EulerianShapeOptimizer* createEulerianOptimizer(void* coreGeometry) {
    // 确保 typedef Euclidean = Vector3 在当前作用域可见
    // geometry.h 已经包含了这个 typedef

    // 显式转换 void* -> Geometry<Euclidean>*
    Geometry<Euclidean>* geom = static_cast<Geometry<Euclidean>*>(coreGeometry);

    // 调用构造函数 - 这里会生成正确的符号引用
    // 由于我们在同一个编译单元中，编译器知道 Euclidean 就是 Vector3
    EulerianShapeOptimizer* optimizer = new EulerianShapeOptimizer(geom);

    return optimizer;
}

/**
 * 包装函数：销毁EulerianShapeOptimizer
 */
void destroyEulerianOptimizer(EulerianShapeOptimizer* optimizer) {
    delete optimizer;
}

} // namespace EulerianOptimizerWrapper
} // namespace SurfaceTextureMapping
