import sympy as sp
import numpy as np

# 示例点位
loc1 = (0, 0, 0)
loc2 = (0, 0, 19)
loc3 = (2, 7, 6)

# 连杆长度示例
l1 = 10
l2 = 10


def solve(loca, locb, l1, l2, eps=1e-9):
    """
    求解逆运动学中间点坐标
    :param loca: 起点坐标 (x1,y1,z1)
    :param locb: 终点坐标 (x2,y2,z2)
    :param l1: 起点到中间点的距离
    :param l2: 中间点到终点的距离
    :param eps: 浮点数精度阈值
    :return: 中间点坐标（唯一有效解）或提示信息
    """
    x1, y1, z1 = loca
    x2, y2, z2 = locb

    # 计算两点间向量及距离
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    d_sq = dx ** 2 + dy ** 2 + dz ** 2  # 距离平方
    d = sp.sqrt(d_sq)  # 实际距离

    # 三角形不等式判断（无解条件）
    if (l1 + l2 < d - eps) or (abs(l1 - l2) > d + eps):
        return f"无解：不满足三角形不等式（距离={d:.2f}, l1={l1}, l2={l2}）"

    # 定义符号变量
    x, y, z = sp.symbols('x y z', real=True)

    # 构建方程组：共线条件（叉积为零确保三点共线）+ 距离条件
    eq1 = sp.Eq((y2 - y1) * (z - z1) - (z2 - z1) * (y - y1), 0)  # 叉积x分量
    eq2 = sp.Eq((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2, l1 ** 2)  # 到起点距离
    eq3 = sp.Eq((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2, l2 ** 2)  # 到终点距离

    # 求解非线性方程组
    solutions = sp.nonlinsolve([eq1, eq2, eq3], [x, y, z])

    # 转换为数值解并去重
    numeric_sols = []
    seen = set()
    for sol in solutions:
        # 计算数值解（保留6位小数）
        x_val = round(float(sol[0].evalf()), 6)
        y_val = round(float(sol[1].evalf()), 6)
        z_val = round(float(sol[2].evalf()), 6)
        # 去重处理
        key = (x_val, y_val, z_val)
        if key not in seen:
            seen.add(key)
            numeric_sols.append(key)

    if not numeric_sols:
        return "无有效解（计算精度问题）"

    # 筛选x大于0的解
    x_positive_sols = [sol for sol in numeric_sols if sol[0] > 0]

    if len(x_positive_sols) == 0:
        return "无满足x>0的解"
    elif len(x_positive_sols) > 1:
        return "错误：满足x>0的解不止一个"
    else:
        return x_positive_sols[0]  # 返回唯一满足条件的解


# 示例调用
if __name__ == "__main__":
    # 测试1：两点在z轴上（应有唯一解）
    print("测试1：loc1到loc2的解：")
    print(solve(loc1, loc2, l1, l2))

    # 测试2：任意两点（可能有0/1/2个解）
    print("\n测试2：loc1到loc3的解：")
    print(solve(loc1, loc3, 15, 10))  # 根据距离判断是否有解

    # 测试3：无解情况（不满足三角形不等式）
    print("\n测试3：无解案例：")
    print(solve(loc1, loc2, 5, 5))  # 距离20，l1+l2=10<20，无解