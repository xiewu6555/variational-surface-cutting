/**
 * Eulerian Shape Optimizer Wrapper
 *
 * 提供类型安全的包装函数来创建/销毁 EulerianShapeOptimizer
 * 解决跨编译单元的typedef模板链接问题
 */

#pragma once

// 前向声明（避免在头文件中包含完整定义）
class EulerianShapeOptimizer;

namespace SurfaceTextureMapping {
namespace EulerianOptimizerWrapper {

/**
 * 创建 EulerianShapeOptimizer 实例
 *
 * @param coreGeometry Core几何对象指针（实际类型是 Geometry<Euclidean>*）
 * @return 优化器指针
 *
 * 注意：调用者负责通过 destroyEulerianOptimizer 销毁返回的对象
 */
EulerianShapeOptimizer* createEulerianOptimizer(void* coreGeometry);

/**
 * 销毁 EulerianShapeOptimizer 实例
 *
 * @param optimizer 要销毁的优化器指针
 */
void destroyEulerianOptimizer(EulerianShapeOptimizer* optimizer);

} // namespace EulerianOptimizerWrapper
} // namespace SurfaceTextureMapping
