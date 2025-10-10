// Clipper2 Offset填充实现
// 使用Clipper2库在UV空间进行路径offset和填充操作

#include "../include/surface_filling.h"
#include <clipper2/clipper.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>

namespace SurfaceTextureMapping {

using namespace Clipper2Lib;
using namespace geometrycentral;

// 实现Clipper2 offset填充功能
class Clipper2OffsetFiller {
public:
    // 将Vector2路径转换为Clipper格式
    static Paths64 ConvertToPaths64(const std::vector<std::vector<Vector2>>& paths, double scale = 1000.0) {
        Paths64 result;
        for (const auto& path : paths) {
            Path64 clipperPath;
            for (const auto& pt : path) {
                clipperPath.push_back(Point64(
                    static_cast<int64_t>(pt.x * scale),
                    static_cast<int64_t>(pt.y * scale)
                ));
            }
            result.push_back(clipperPath);
        }
        return result;
    }

    // 将Clipper格式转换回Vector2路径
    static std::vector<std::vector<Vector2>> ConvertFromPaths64(const Paths64& paths, double scale = 1000.0) {
        std::vector<std::vector<Vector2>> result;
        for (const auto& path : paths) {
            std::vector<Vector2> vecPath;
            for (const auto& pt : path) {
                vecPath.push_back(Vector2{
                    static_cast<double>(pt.x) / scale,
                    static_cast<double>(pt.y) / scale
                });
            }
            result.push_back(vecPath);
        }
        return result;
    }

    // 执行路径offset操作
    static std::vector<std::vector<Vector2>> PerformOffset(
        const std::vector<std::vector<Vector2>>& inputPaths,
        double offset,
        JoinType joinType = JoinType::Round,
        EndType endType = EndType::Polygon) {

        const double scale = 10000.0; // 缩放因子，提高精度

        // 转换为Clipper格式
        Paths64 paths = ConvertToPaths64(inputPaths, scale);

        // 创建ClipperOffset对象
        ClipperOffset co;
        co.AddPaths(paths, joinType, endType);

        // 执行offset
        Paths64 solution;
        co.Execute(offset * scale, solution);

        // 转换回Vector2格式
        return ConvertFromPaths64(solution, scale);
    }

    // 生成offset填充图案
    static std::vector<std::vector<Vector2>> GenerateOffsetFilling(
        const std::vector<Vector2>& boundary,
        double spacing,
        int numOffsets = -1) {

        std::vector<std::vector<Vector2>> result;
        std::vector<std::vector<Vector2>> currentBoundary = {boundary};

        // 如果未指定offset数量，根据边界大小自动计算
        if (numOffsets < 0) {
            double minX = std::numeric_limits<double>::max();
            double maxX = std::numeric_limits<double>::lowest();
            double minY = std::numeric_limits<double>::max();
            double maxY = std::numeric_limits<double>::lowest();

            for (const auto& pt : boundary) {
                minX = std::min(minX, pt.x);
                maxX = std::max(maxX, pt.x);
                minY = std::min(minY, pt.y);
                maxY = std::max(maxY, pt.y);
            }

            double boundarySize = std::min(maxX - minX, maxY - minY);
            numOffsets = static_cast<int>(boundarySize / (2 * spacing));
        }

        // 逐层向内offset
        for (int i = 0; i < numOffsets; i++) {
            double offsetDistance = -spacing * (i + 1); // 负值表示向内offset

            auto offsetPaths = PerformOffset(currentBoundary, offsetDistance);

            if (offsetPaths.empty()) {
                break; // 无法继续offset，已到达中心
            }

            result.insert(result.end(), offsetPaths.begin(), offsetPaths.end());
            currentBoundary = offsetPaths;
        }

        return result;
    }

    // 生成网格线并使用Clipper裁剪到边界内
    static std::vector<std::vector<Vector2>> GenerateClippedGrid(
        const std::vector<Vector2>& boundary,
        double spacing) {

        // 计算边界框
        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::lowest();

        for (const auto& pt : boundary) {
            minX = std::min(minX, pt.x);
            maxX = std::max(maxX, pt.x);
            minY = std::min(minY, pt.y);
            maxY = std::max(maxY, pt.y);
        }

        // 生成网格线
        std::vector<std::vector<Vector2>> gridLines;

        // 垂直线
        for (double x = minX; x <= maxX; x += spacing) {
            gridLines.push_back({
                Vector2{x, minY - spacing},
                Vector2{x, maxY + spacing}
            });
        }

        // 水平线
        for (double y = minY; y <= maxY; y += spacing) {
            gridLines.push_back({
                Vector2{minX - spacing, y},
                Vector2{maxX + spacing, y}
            });
        }

        // 使用Clipper裁剪到边界内
        return ClipLinesToBoundary(gridLines, boundary);
    }

    // 将线段裁剪到边界内
    static std::vector<std::vector<Vector2>> ClipLinesToBoundary(
        const std::vector<std::vector<Vector2>>& lines,
        const std::vector<Vector2>& boundary) {

        const double scale = 10000.0;

        // 转换边界为Clipper格式
        Paths64 clipPaths = ConvertToPaths64({boundary}, scale);

        std::vector<std::vector<Vector2>> result;

        for (const auto& line : lines) {
            if (line.size() < 2) continue;

            // 创建一个很细的矩形来表示线段
            double dx = line[1].x - line[0].x;
            double dy = line[1].y - line[0].y;
            double len = std::sqrt(dx * dx + dy * dy);

            if (len < 1e-6) continue;

            // 法向量
            double nx = -dy / len * 0.0001; // 很小的宽度
            double ny = dx / len * 0.0001;

            // 构建细矩形
            std::vector<Vector2> rect = {
                Vector2{line[0].x + nx, line[0].y + ny},
                Vector2{line[1].x + nx, line[1].y + ny},
                Vector2{line[1].x - nx, line[1].y - ny},
                Vector2{line[0].x - nx, line[0].y - ny}
            };

            Paths64 subjectPaths = ConvertToPaths64({rect}, scale);

            // 执行裁剪
            Clipper64 clipper;
            clipper.AddSubject(subjectPaths);
            clipper.AddClip(clipPaths);

            Paths64 solution;
            clipper.Execute(ClipType::Intersection, FillRule::NonZero, solution);

            if (!solution.empty()) {
                // 简化结果，提取中心线
                for (const auto& path : solution) {
                    if (path.size() >= 2) {
                        std::vector<Vector2> clippedLine;
                        clippedLine.push_back(Vector2{
                            static_cast<double>(path[0].x) / scale,
                            static_cast<double>(path[0].y) / scale
                        });
                        clippedLine.push_back(Vector2{
                            static_cast<double>(path[path.size() - 1].x) / scale,
                            static_cast<double>(path[path.size() - 1].y) / scale
                        });
                        result.push_back(clippedLine);
                    }
                }
            }
        }

        return result;
    }

    // 生成螺旋填充
    static std::vector<Vector2> GenerateSpiralFilling(
        const std::vector<Vector2>& boundary,
        double spacing) {

        std::vector<Vector2> spiral;

        // 计算边界中心
        Vector2 center{0, 0};
        for (const auto& pt : boundary) {
            center += pt;
        }
        center /= static_cast<double>(boundary.size());

        // 生成阿基米德螺旋
        double a = spacing / (2 * M_PI);
        double maxRadius = 0;

        // 找到最大半径
        for (const auto& pt : boundary) {
            double dist = (pt - center).norm();
            maxRadius = std::max(maxRadius, dist);
        }

        // 生成螺旋点
        for (double t = 0; t < maxRadius / a; t += 0.1) {
            double r = a * t;
            if (r > maxRadius) break;

            Vector2 pt{
                center.x + r * std::cos(t),
                center.y + r * std::sin(t)
            };

            // 检查点是否在边界内（简化检查）
            if (IsPointInPolygon(pt, boundary)) {
                spiral.push_back(pt);
            }
        }

        return spiral;
    }

    // 点在多边形内部测试
    static bool IsPointInPolygon(const Vector2& point, const std::vector<Vector2>& polygon) {
        int n = polygon.size();
        bool inside = false;

        Vector2 p1 = polygon[0];
        for (int i = 1; i <= n; i++) {
            Vector2 p2 = polygon[i % n];

            if (point.y > std::min(p1.y, p2.y)) {
                if (point.y <= std::max(p1.y, p2.y)) {
                    if (point.x <= std::max(p1.x, p2.x)) {
                        if (p1.y != p2.y) {
                            double xinters = (point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                            if (p1.x == p2.x || point.x <= xinters) {
                                inside = !inside;
                            }
                        }
                    }
                }
            }

            p1 = p2;
        }

        return inside;
    }

    // 生成六边形填充图案
    static std::vector<std::vector<Vector2>> GenerateHexagonalFilling(
        const std::vector<Vector2>& boundary,
        double hexSize) {

        std::vector<std::vector<Vector2>> hexagons;

        // 计算边界框
        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::lowest();

        for (const auto& pt : boundary) {
            minX = std::min(minX, pt.x);
            maxX = std::max(maxX, pt.x);
            minY = std::min(minY, pt.y);
            maxY = std::max(maxY, pt.y);
        }

        // 六边形参数
        double hexWidth = hexSize * 2;
        double hexHeight = hexSize * std::sqrt(3);
        double hexVertSpacing = hexHeight * 0.75;
        double hexHorzSpacing = hexWidth * 1.5;

        // 生成六边形网格
        int row = 0;
        for (double y = minY; y <= maxY + hexHeight; y += hexVertSpacing) {
            double xOffset = (row % 2) * (hexWidth * 0.75);

            for (double x = minX + xOffset; x <= maxX + hexWidth; x += hexHorzSpacing) {
                // 生成单个六边形
                std::vector<Vector2> hex;
                for (int i = 0; i < 6; i++) {
                    double angle = i * M_PI / 3;
                    hex.push_back(Vector2{
                        x + hexSize * std::cos(angle),
                        y + hexSize * std::sin(angle)
                    });
                }
                hex.push_back(hex[0]); // 闭合六边形

                // 检查六边形是否与边界相交
                if (HexagonIntersectsBoundary(hex, boundary)) {
                    hexagons.push_back(hex);
                }
            }
            row++;
        }

        // 使用Clipper裁剪六边形到边界内
        return ClipPathsToBoundary(hexagons, boundary);
    }

    // 检查六边形是否与边界相交
    static bool HexagonIntersectsBoundary(
        const std::vector<Vector2>& hex,
        const std::vector<Vector2>& boundary) {

        // 简单检查：六边形中心是否在边界内
        Vector2 center{0, 0};
        for (size_t i = 0; i < hex.size() - 1; i++) { // 排除重复的最后一个点
            center += hex[i];
        }
        center /= static_cast<double>(hex.size() - 1);

        return IsPointInPolygon(center, boundary);
    }

    // 裁剪路径到边界内
    static std::vector<std::vector<Vector2>> ClipPathsToBoundary(
        const std::vector<std::vector<Vector2>>& paths,
        const std::vector<Vector2>& boundary) {

        const double scale = 10000.0;

        // 转换为Clipper格式
        Paths64 subjectPaths = ConvertToPaths64(paths, scale);
        Paths64 clipPaths = ConvertToPaths64({boundary}, scale);

        // 执行裁剪
        Clipper64 clipper;
        clipper.AddSubject(subjectPaths);
        clipper.AddClip(clipPaths);

        Paths64 solution;
        clipper.Execute(ClipType::Intersection, FillRule::NonZero, solution);

        // 转换回Vector2格式
        return ConvertFromPaths64(solution, scale);
    }
};

// 扩展SurfaceFiller的Clipper功能
std::vector<std::vector<Vector2>> SurfaceFiller::clipPaths(
    const std::vector<std::vector<Vector2>>& paths,
    const std::vector<std::vector<Vector2>>& clipPaths,
    Clipper2Lib::ClipType operation) const {

    return Clipper2OffsetFiller::ClipPathsToBoundary(paths, clipPaths[0]);
}

// 实现网格图案生成（使用Clipper2）
std::vector<std::vector<Vector2>> SurfaceFiller::generateGridPattern(
    double spacing, const std::vector<Vector2>& uvBounds) const {

    return Clipper2OffsetFiller::GenerateClippedGrid(uvBounds, spacing);
}

// 实现六边形图案生成（使用Clipper2）
std::vector<std::vector<Vector2>> SurfaceFiller::generateHexagonalPattern(
    double spacing, const std::vector<Vector2>& uvBounds) const {

    return Clipper2OffsetFiller::GenerateHexagonalFilling(uvBounds, spacing);
}

// 实现螺旋图案生成
std::vector<Vector2> SurfaceFiller::generateSpiralPattern(
    double spacing, const Vector2& center, double maxRadius) const {

    // 创建圆形边界
    std::vector<Vector2> circleBoundary;
    int numPoints = 64;
    for (int i = 0; i < numPoints; i++) {
        double angle = 2 * M_PI * i / numPoints;
        circleBoundary.push_back(Vector2{
            center.x + maxRadius * std::cos(angle),
            center.y + maxRadius * std::sin(angle)
        });
    }

    return Clipper2OffsetFiller::GenerateSpiralFilling(circleBoundary, spacing);
}

// 创建专门的Offset填充函数
std::vector<std::vector<Vector2>> GenerateOffsetFillingPattern(
    const std::vector<Vector2>& uvBoundary,
    double spacing,
    int maxOffsets) {

    return Clipper2OffsetFiller::GenerateOffsetFilling(uvBoundary, spacing, maxOffsets);
}

// 导出到全局命名空间供其他模块使用
namespace Clipper2Filling {

std::vector<std::vector<Vector2>> CreateOffsetPattern(
    const std::vector<Vector2>& boundary,
    double spacing) {
    return Clipper2OffsetFiller::GenerateOffsetFilling(boundary, spacing);
}

std::vector<std::vector<Vector2>> CreateGridPattern(
    const std::vector<Vector2>& boundary,
    double spacing) {
    return Clipper2OffsetFiller::GenerateClippedGrid(boundary, spacing);
}

std::vector<std::vector<Vector2>> CreateHexPattern(
    const std::vector<Vector2>& boundary,
    double hexSize) {
    return Clipper2OffsetFiller::GenerateHexagonalFilling(boundary, hexSize);
}

std::vector<Vector2> CreateSpiralPattern(
    const std::vector<Vector2>& boundary,
    double spacing) {
    return Clipper2OffsetFiller::GenerateSpiralFilling(boundary, spacing);
}

} // namespace Clipper2Filling

} // namespace SurfaceTextureMapping