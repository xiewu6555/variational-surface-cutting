#!/usr/bin/env python3
"""
生成测试网格数据
用于单元测试和端到端测试

版本: 1.0
日期: 2025-09-30
"""

import math
import sys

def generate_cylinder(radius=10.0, height=50.0, segments=32, rings=16):
    """
    生成圆柱体网格 (OBJ格式)

    参数:
        radius: 半径 (mm)
        height: 高度 (mm)
        segments: 周向分段数
        rings: 轴向分段数

    返回:
        vertices, faces列表
    """
    vertices = []
    faces = []

    # 生成顶点
    for i in range(rings + 1):
        z = -height/2 + (height * i / rings)
        for j in range(segments):
            theta = 2 * math.pi * j / segments
            x = radius * math.cos(theta)
            y = radius * math.sin(theta)
            vertices.append((x, y, z))

    # 生成面片 (三角形)
    for i in range(rings):
        for j in range(segments):
            # 当前环的顶点索引
            v0 = i * segments + j
            v1 = i * segments + (j + 1) % segments
            # 下一环的顶点索引
            v2 = (i + 1) * segments + (j + 1) % segments
            v3 = (i + 1) * segments + j

            # OBJ索引从1开始
            faces.append((v0 + 1, v1 + 1, v2 + 1))
            faces.append((v0 + 1, v2 + 1, v3 + 1))

    return vertices, faces

def generate_sphere(radius=15.0, slices=32, stacks=16):
    """
    生成球体网格

    参数:
        radius: 半径 (mm)
        slices: 经度分段数
        stacks: 纬度分段数
    """
    vertices = []
    faces = []

    # 顶点
    for i in range(stacks + 1):
        phi = math.pi * i / stacks  # 纬度角
        for j in range(slices):
            theta = 2 * math.pi * j / slices  # 经度角
            x = radius * math.sin(phi) * math.cos(theta)
            y = radius * math.sin(phi) * math.sin(theta)
            z = radius * math.cos(phi)
            vertices.append((x, y, z))

    # 面片
    for i in range(stacks):
        for j in range(slices):
            v0 = i * slices + j
            v1 = i * slices + (j + 1) % slices
            v2 = (i + 1) * slices + (j + 1) % slices
            v3 = (i + 1) * slices + j

            if i > 0:  # 不在顶部极点
                faces.append((v0 + 1, v1 + 1, v2 + 1))
            if i < stacks - 1:  # 不在底部极点
                faces.append((v0 + 1, v2 + 1, v3 + 1))

    return vertices, faces

def generate_plane(width=100.0, height=100.0, segments_x=10, segments_y=10):
    """
    生成平面网格

    参数:
        width: 宽度 (mm)
        height: 高度 (mm)
        segments_x: X方向分段数
        segments_y: Y方向分段数
    """
    vertices = []
    faces = []

    # 顶点
    for i in range(segments_y + 1):
        y = -height/2 + (height * i / segments_y)
        for j in range(segments_x + 1):
            x = -width/2 + (width * j / segments_x)
            z = 0.0
            vertices.append((x, y, z))

    # 面片
    for i in range(segments_y):
        for j in range(segments_x):
            v0 = i * (segments_x + 1) + j
            v1 = i * (segments_x + 1) + j + 1
            v2 = (i + 1) * (segments_x + 1) + j + 1
            v3 = (i + 1) * (segments_x + 1) + j

            faces.append((v0 + 1, v1 + 1, v2 + 1))
            faces.append((v0 + 1, v2 + 1, v3 + 1))

    return vertices, faces

def save_obj(filename, vertices, faces):
    """保存OBJ文件"""
    with open(filename, 'w') as f:
        f.write("# Generated test mesh\n")
        f.write(f"# Vertices: {len(vertices)}\n")
        f.write(f"# Faces: {len(faces)}\n\n")

        # 写入顶点
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")

        f.write("\n")

        # 写入面片
        for face in faces:
            f.write(f"f {face[0]} {face[1]} {face[2]}\n")

    print(f"已生成: {filename} ({len(vertices)} 顶点, {len(faces)} 面)")

def main():
    # 生成测试网格
    print("生成测试网格...")

    # 圆柱体 (用于E2E测试)
    # R=10mm, H=50mm -> 周长 ≈ 62.83mm
    vertices, faces = generate_cylinder(radius=10.0, height=50.0, segments=32, rings=16)
    save_obj("cylinder.obj", vertices, faces)

    # 简化的圆柱体 (用于快速测试)
    vertices, faces = generate_cylinder(radius=10.0, height=50.0, segments=16, rings=8)
    save_obj("cylinder_simple.obj", vertices, faces)

    # 球体 (用于失真测试)
    vertices, faces = generate_sphere(radius=15.0, slices=32, stacks=16)
    save_obj("sphere.obj", vertices, faces)

    # 平面 (用于完美映射测试)
    vertices, faces = generate_plane(width=100.0, height=100.0, segments_x=10, segments_y=10)
    save_obj("plane.obj", vertices, faces)

    print("\n测试网格生成完成!")
    print("使用方法:")
    print("  - cylinder.obj: E2E测试 (R=10mm, H=50mm, 周长≈62.83mm)")
    print("  - cylinder_simple.obj: 快速单元测试")
    print("  - sphere.obj: 失真分析测试")
    print("  - plane.obj: 完美映射测试")

if __name__ == "__main__":
    main()