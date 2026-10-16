# Drone Traffic Research：基础仿真器

这是从 RobotSim 迁移的二维、多无人机、固定步长仿真基础。采用 SI 单位，保留旧速度位置积分、速度松弛、加速度模长限幅和周期控制。当前控制器仅输出预设速度，用于验证底层环境；尚未实现论文交通管理算法。

## 运行

在项目根目录执行：

~~~bash
conda activate drone-traffic
python -c "import sys; print(sys.executable)"
python -m pip install -r requirements.txt
python -m pytest -q --junitxml=results/pytest.xml
python -m experiments.run_basic --scenario all
~~~

解释器应是 /home/gag47/miniconda3/envs/drone-traffic/bin/python。依赖本轮已安装；直接运行测试和实验即可。若终端仍激活 f5-tts，可明确调用目标解释器：

~~~bash
/home/gag47/miniconda3/envs/drone-traffic/bin/python -m pytest -q
/home/gag47/miniconda3/envs/drone-traffic/bin/python -m experiments.run_basic --scenario all
~~~

单独运行场景：

~~~bash
python -m experiments.run_basic --scenario single
python -m experiments.run_basic --scenario head_on
python -m experiments.run_basic --scenario five_agents
~~~

程序使用 Matplotlib Agg 后端，不需要 OpenGL 或实时 GUI。可用 --output 指定结果根目录；重复运行会覆盖该根目录下对应场景的同名产物。--dt 可改变步长，但必须整除 0.2 s 控制周期和场景时长，且不大于速度松弛时间；例如：

~~~bash
python -m experiments.run_basic --scenario all --dt 0.02 --output results/dt_002
~~~

## 实验与输出

| 场景 | 配置 | 本轮结果 |
|---|---|---|
| single | 1 机从静止开始，期望速度 (8, 0) m/s，10 s | 最终速度 7.999638 m/s；最大加速度 6 m/s² |
| head_on | 2 机从 (-10, 0)/(10, 0) m 出发，期望速度 (±2, 0) m/s，12 s | 对称接近后穿越；两机与各自独立仿真结果一致 |
| five_agents | 5 个不同初始位置与速度指令，10 s | 状态、邻居变化和轨迹记录验证通过 |

全部使用 dt=0.01 s、tau=1 s、a_max=6 m/s²、控制周期 0.2 s。场景没有随机输入、噪声、延迟、墙壁或避碰；相遇与穿越属于预期行为。通信半径分别为 80、8、6 m，后两个较小值用于显式验证邻居进入／离开。

每个 results/basic/场景名/ 下包含：

- trajectories.png：二维轨迹，圆点为起点、叉号为终点。
- speed.png：速度和加速度曲线；点线为给定速度大小，黑虚线为加速度上限。
- neighbors.png：配对距离与邻居数量。
- trajectory.npz / trajectory.csv：完整状态轨迹。
- speed.csv：逐时刻、逐机速度数据。
- config.json：初态、控制器、动力学参数、单位、环境版本及参考提交。
- summary.json：数值摘要和基础不变量检查。

查看：[单机速度响应](results/basic/single/speed.png)、[相向运动距离](results/basic/head_on/neighbors.png)、[五机轨迹](results/basic/five_agents/trajectories.png)。

NPZ 不含 Python 对象，可安全读取：

~~~python
import numpy as np
with np.load("results/basic/five_agents/trajectory.npz", allow_pickle=False) as data:
    times = data["times"]               # (K+1,)
    positions = data["positions"]       # (K+1, N, 2), m
    velocities = data["velocities"]     # (K+1, N, 2), m/s
    commands = data["applied_desired_velocities"]  # (K, N, 2), m/s
~~~

状态行 k 对应 t_k；区间命令和 accelerations[k] 对应 [t_k, t_(k+1))。状态字段 desired_velocities 是最近已应用／保持的命令，初始行是用户输入的初值；分析控制刷新时刻请使用 applied_desired_velocities。CSV 使用 held_desired_* 列名明确这一点。

## 模块与验证

| 文件 | 用途 |
|---|---|
| simulator/state.py | 独立、只读的状态数组与稳定 ID |
| simulator/dynamics.py | 纯函数形式的速度跟踪、限幅、旧速度位置积分 |
| simulator/neighbors.py | 通信半径内邻居、AgentObservation、可替换 ObservationProvider |
| simulator/engine.py | 同步快照、整数步计时、周期控制与命令保持 |
| simulator/logging.py | 状态／区间记录及 CSV、NPZ 输出 |
| controllers/basic.py | 与底层分开的固定期望速度测试控制器 |
| scenarios/basic_scenarios.py | 确定性 1／2／5 机配置 |
| experiments/run_basic.py | 命令行运行、摘要和静态绘图 |
| tests/ | 解析解、原 C 数值片段对照、状态／邻居／引擎／输出验证 |

本轮实测 **47 passed**。C 对照测试从只读原仓库提取原函数／代码片段，在 pytest 临时目录用 GCC 编译，绕过 GUI 依赖；若机器没有 GCC，只跳过这 4 组对照，解析解测试仍执行。原版 RobotSim 整个程序没有构建或运行。

详见 [迁移说明与实测误差](docs/simulator_migration.md) 和 [论文审计](docs/paper_audit.md)。后续控制器实现 Controller 的调用接口，观测误差／延迟通过 ObservationProvider 接入。目标状态、噪声过程和延迟历史缓存仍需下一阶段实现。
# drone-traffic-research
