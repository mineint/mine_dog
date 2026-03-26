在四足机器人（机器狗）的控制中，动态y轴算法的核心目标是：消除足端摩擦（Scrubbing）、抵消离心力以及维持动态平衡。
按复杂度从低到高排列：
1. 几何运动学算法（基于 ICR 瞬时旋转中心）

2. Raibert 启发式算法（Raibert Heuristic）

3. 离心力补偿算法（Centrifugal Compensation）

4. 捕获点算法（Capture Point / CMP）

5. 基于模型预测控制（MPC）的动态分配
