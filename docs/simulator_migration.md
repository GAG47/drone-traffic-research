# RobotSim 基础仿真框架迁移说明

日期：2026-10-15。参考提交：ea54b97e7e2659449e721ebd4a2285beb9b0e9ce。该提交在 references/robotsim 中保持不变。本轮交付的是二维基础仿真器与验证实验，固定速度测试控制器不代表论文交通算法。

## 1. 已迁移的内容与源码对应

| 原源码／函数 | Python 文件／接口 | 保留的行为 |
|---|---|---|
| utilities/dynamics_utils.h:36，phase_t；dynamics_utils.c，AllocatePhase 等 | simulator/state.py，DroneState | 按机组织位置、速度和身份；额外把原全局 PreferredVelocities 纳入状态 |
| robotmodel.c:183，RealCoptForceLaw | simulator/dynamics.py，advance | 实际速度加上由期望速度减测量速度得到的松弛增量 |
| robotmodel.c:279–294，Step 的位置更新块 | advance | 用旧实际速度积分位置 |
| robotmodel.c:350–375，Step 的加速度约束块 | advance | 对速度变化的向量模长限幅，保留方向 |
| robotmodel.c:206–223，控制周期判断与 PreferredVelocities 缓存 | simulator/engine.py，Simulation.step | 控制周期到来时刷新期望速度，期间保持命令；物理速度每步继续跟踪 |
| robotmodel.c:25，CreatePhase；dynamics_utils.c:1491，SelectNearbyVisibleAgents | simulator/neighbors.py，PerfectObservation、neighbors_within | 通过通信范围获得局部邻居状态；当前无测量误差、无延迟、无丢包 |
| robotsim_main.c:1920 起的无界面推进循环与状态存储 | Simulation.run、TrajectoryRecorder | 统一推进、保存初态及每个步末的状态 |
| math_utils.c:200，UnitVect 的归一化思想 | advance 的速度增量归一化 | 饱和时保持原增量方向；没有迁移与当前任务无关的几何工具库 |

原仓库内部使用 cm、cm/s、cm/s²；本实现统一为 m、m/s、m/s²，时间均为秒。例如 a_max=600 cm/s² 对应 6 m/s²。所有 Python 结果文件已使用 SI 单位，不需额外乘 0.01。

## 2. 数学计算与执行顺序

令第 k 步开始时的真实位置、真实速度为 x_k、v_k，测量速度为 v_obs，当前保持的期望速度为 u_k。核心计算为：

$$
x_{k+1}=x_k+\Delta t\,v_k,
\qquad
v_{raw}=v_k+\frac{\Delta t}{\tau}(u_k-v_{obs}),
$$

$$
\Delta v=v_{raw}-v_k,\qquad
\Delta v_{limited}=
\begin{cases}
\Delta v,&\|\Delta v\|/\Delta t\le a_{max},\\
a_{max}\Delta t\frac{\Delta v}{\|\Delta v\|},&\text{其他情况},
\end{cases}
\qquad
v_{k+1}=v_k+\Delta v_{limited}.
$$

当前完美观测取 v_obs=v_k。物理层未添加速度上限，也未把期望速度直接用于位置积分。

一次 Python step 的顺序：

1. 取共同的只读状态快照，构建全部 agent 的观测。
2. 若整数步编号达到控制刷新周期，计算并收集所有 agent 的输出；否则保持上次命令。
3. 从同一旧快照计算全部下一位置、下一速度，执行加速度模长限幅。
4. 整体替换状态，用整数步编号计算新时间，记录状态、区间命令与加速度。

原 C 先写下一位置，再计算控制输出；Python 为保持纯函数结构先收集控制输出，再在 advance 中计算位置。两者控制器均读取旧快照，位置也均使用旧速度，因而在当前无噪声模型中数学顺序一致。逐机调用控制器不会让后调用者看到已更新的其他机状态。

工程上要求控制器自身的跨步记忆按 ID 管理；不要用一个与调用次数绑定的共享计数器代表每架机的状态。基础控制器是无状态的，ID 不依赖数组行号。

## 3. 保留、简化和边界

### 保留的原始计算

- 水平速度松弛时间 tau，以及对“期望减测量速度”的响应。
- 旧实际速度的显式 Euler 位置积分。
- 二维速度增量的欧氏模长限幅，作为原三维算法 z=0 的切片。
- 周期性刷新期望速度、周期之间保持命令，同时每个物理步继续动力学更新。
- 邻居范围采用距离 ≤ 通信半径。

### 明确的工程简化

| 简化／补全 | 原因及限制 |
|---|---|
| 仅二维，状态形状 (N,2) | 满足本轮任务；核心计算按空间轴处理。扩展三维需放宽状态校验并加入 tau_Z，不代表已经实现三维交通 |
| 无传感器噪声、外部噪声、延迟、丢包或风 | 先把确定性数值行为验证清楚。已有 ObservationProvider 接口，但尚无这些过程的实际实现 |
| 通过 ID／行索引排除自身 | 原筛选函数用 Dist==0 识别自身，不适合处理不同 agent 完全重合。Python 保留重合但不同 ID 的邻居，属于有意修正 |
| 保留稳定数组顺序，不把自身交换到局部索引 0 | 每份 AgentObservation 显式区分自身和 neighbors，不需原 C 的指针交换 |
| 全状态存入内存而不建立循环历史缓存 | 1／2／5 机运行简单可核查。时间／内存成本不适合直接外推到长时间大规模实验 |
| 邻居每步按当前位置重新计算 | 便于正确获取和记录当前邻居；只在控制刷新时影响期望命令。没有旧通信包缓存 |
| 控制周期必须是 dt 的整数倍 | 避免原 C 强制取整周期的隐性偏差；默认 0.2/0.01=20 步 |
| 0 < dt ≤ tau | 显式 Euler 在此范围保持无约束速度响应单调；原 C 未做同样校验，此处主动拒绝更大的步长 |
| 时间取 step_index × dt | 避免反复累加产生终止时间漂移；运行时长也必须为整数步 |
| 无墙壁、障碍、边界限制 | 五机是在二维平面中从给定区域出发，后续可离开该区域 |
| 不继承 UnitVect 的极小向量死区 | 原 C 将长度小于 1e-10 cm 的向量归零；Python 只对非零且确需限幅的增量归一化。极端微小尺度不承诺逐位一致 |
| 参数和场景选择明确分开 | dt=0.01、tau=1、a_max=6、控制周期=0.2 来自旧框架默认；单机通信范围80 m参考 S1，双机8 m／五机6 m为邻居测试自定值，均非本文交通实验复现 |

本轮未迁移 SPP、Self-drive、Repulsion、Friction、Queueing、场地／障碍 shill、GUI、旧 Flocking 统计、进化优化、路径规划或城市资源容量。

## 4. 测试方法与实际结果

实际使用解释器：

~~~text
/home/gag47/miniconda3/envs/drone-traffic/bin/python
Python 3.11.17
NumPy 2.4.6
Matplotlib 3.11.2
pytest 9.1.1
GCC 13.3.0
~~~

NumPy、Matplotlib、pytest 安装在 drone-traffic 环境；requirements.txt 固定本轮使用的直接依赖版本。pip check 未发现依赖冲突。未安装 OpenGL、ROS、Gazebo 或 PyTorch。

实际执行：

~~~bash
/home/gag47/miniconda3/envs/drone-traffic/bin/python -m pytest -q --junitxml=results/pytest.xml
MPLCONFIGDIR=/tmp/drone-traffic-matplotlib /home/gag47/miniconda3/envs/drone-traffic/bin/python -m experiments.run_basic --scenario all
/home/gag47/miniconda3/envs/drone-traffic/bin/python -m pip check
~~~

测试结果：**47 passed in 2.91s**，无跳过。机器可读记录见 [results/pytest.xml](../results/pytest.xml)。

### 独立数学基准

无加速度饱和、完美观测、恒定命令 u 时，q=1-dt/tau：

$$
v_n=u+(v_0-u)q^n,\qquad
x_n=x_0+n\Delta t\,u+\tau(v_0-u)(1-q^n).
$$

测试使用这些解析表达式核对整段轨迹，覆盖 dt=0.01、0.05、0.1、0.2、0.5 s。存在饱和时，根据初始速度误差求饱和阶段步数，以等差数列求该阶段位移，再接几何级数尾段。包括运动中将期望速度设为零的制动案例。

其余测试覆盖：静止零指令、模长而非分量限幅、旧速度积分、测量／真实速度区别、双机与单独运行一致、同快照控制、行顺序置换、指令保持、时钟连续、轨迹初态和区间对齐、重复运行逐元素相同、ID 与状态隔离、零距离不同 agent、范围边界、文件和 CLI 输出、非法输入。

### 原 C 数值对照

tests/test_c_reference.py 从原文件直接读取并提取：

- 完整 RealCoptForceLaw 函数；
- Step 中的位置更新代码块；
- Step 中的加速度限幅代码块；
- math_utils.c 中的 UnitVect 函数。

被测片段不重写；测试仅补足小型类型、向量读写和固定命令的 CalculatePreferredVelocity 桩。源文件与共享库仅写在 pytest 临时目录，使用 gcc -std=c99 -shared -fPIC -O0 ... -lm 编译。因此没有 OpenGL 仍可执行这部分验证。

C 端使用原生厘米单位，Python 端使用 SI；比较前转换单位。每组 500 步，包含命令突变、转向、减速和其中一组人为测量速度偏差。每步控制刷新，较慢控制周期保持由独立引擎测试覆盖。

| dt / tau / a_max | 测量速度偏差 (m/s) | 最大位置分量差 (m) | 最大速度分量差 (m/s) |
|---|---|---:|---:|
| 0.01 / 1 / 6 | (0,0) | 3.020e-14 | 7.105e-15 |
| 0.05 / 0.5 / 2 | (0,0) | 1.705e-13 | 2.665e-14 |
| 0.1 / 1 / 100 | (0,0) | 1.705e-13 | 2.665e-15 |
| 0.01 / 1 / 6 | (0.25,-0.1) | 2.665e-14 | 8.882e-15 |

这些差异位于浮点运算和单位转换的舍入尺度。结论限定为被提取的确定性动力学片段；**没有构建或运行原完整 RobotSim，也没有证明噪声、通信或论文算法等效**。C 测试在无 GCC 的机器上会明确跳过；本轮 GCC 存在，4 组均实际执行。

## 5. 三组实验结果

| 场景 | 时长／步数／状态样本 | 最大加速度 (m/s²) | 末态最大速度误差 (m/s) | 结果 |
|---|---|---:|---:|---|
| single | 10 s / 1000 / 1001 | 6.000000 | 3.621e-4 | 达到加速度上限后转入渐近跟踪，最终 x=71.656962 m、vx=7.999638 m/s |
| head_on | 12 s / 1200 / 1201 | 2.000000 | 1.157e-5 | 两机独立、镜像运动，最终 x≈±12.000012 m，顺序已交换 |
| five_agents | 10 s / 1000 / 1001 | 2.236068 | 9.653e-5 | 五组轨迹均正常记录，ID 与状态互不混用 |

双机从20 m间距开始，t=6.00 s采样到最小间距约0.009620 m；通信半径8 m，邻居数从0变1再变0。采样最小距离不代表连续时间最小距离，轨迹实际发生穿越。五机也会接近／穿越，其采样最小间距约0.003768 m；这些结果符合当前没有避碰控制的点模型。

五机初始邻居数为 [1,2,2,1,0]，最终为 [0,1,0,0,1]，过程中每架机最多有4个邻居。位置积分残差的全部实验最大值约7.04e-15 m。所有摘要中的基础检查均通过。

实验目录：

- [单机](../results/basic/single/)：[轨迹](../results/basic/single/trajectories.png)、[速度／加速度](../results/basic/single/speed.png)、[摘要](../results/basic/single/summary.json)。
- [双机](../results/basic/head_on/)：[轨迹](../results/basic/head_on/trajectories.png)、[距离／邻居](../results/basic/head_on/neighbors.png)、[摘要](../results/basic/head_on/summary.json)。
- [五机](../results/basic/five_agents/)：[轨迹](../results/basic/five_agents/trajectories.png)、[距离／邻居](../results/basic/five_agents/neighbors.png)、[摘要](../results/basic/five_agents/summary.json)。

各目录还包含 config.json、trajectory.npz、trajectory.csv、speed.csv 及其余图像，合计每组8个文件。已目视检查五机轨迹、单机响应、双机距离／邻居图，标签、单位和运动趋势正常。

## 6. 记录时间与后续接入接口

K 个物理步产生 K+1 行状态，包含 t=0 初态和最终状态：

| NPZ 字段 | 形状 | 时间语义 |
|---|---|---|
| times | (K+1,) | 状态时刻 |
| ids | (N,) | 稳定身份 |
| positions / velocities | (K+1,N,2) | 当前真实状态 |
| desired_velocities | (K+1,N,2) | 最近已应用／保持的命令；第0行是初始化输入 |
| applied_desired_velocities | (K,N,2) | 从状态 k 推进到 k+1 时实际使用的命令 |
| accelerations | (K,N,2) | 同一区间的确定性速度增量除以 dt |
| neighbor_counts | (K+1,N) | 该状态下真实通信范围内邻居数量 |

后续交通控制器接入 Controller(observation, time) → 2D desired velocity，每个 agent 接收自身及局部邻居快照。AgentObservation 目前有 agent_id、position、velocity、neighbors；NeighborStates 有 ids、positions、velocities、distances。目标可首先放在按 ID 管理的任务状态中，再显式扩展需要广播的观测字段。

ObservationProvider.observe(snapshot,time,radius) 每个物理步统一调用，返回按同一 ID 顺序排列的全部局部观测；未来可以内部维护 GPS 误差和带时间戳的邻居缓存。advance 的 measured_velocities 参数已经进入实际速度跟踪计算，测试验证了这一通路，避免将来误用测量速度作为真实状态。

复现论文前仍需补齐：

1. 目标／到达事件、上一与下一目标、邻居目标广播。
2. 原 GPS 相关误差过程、通信延迟历史、丢包和时间戳处理；若加入外部噪声，应按原 C 在确定性限幅之后叠加。
3. 论文规定的有效速度、危险配对时间比例和到达吞吐量；当前摘要仅用于验证基础环境。
4. 已审计的公式／参数歧义的实现约定，然后才是论文控制器各分项。
5. 更大规模时的空间索引和分块／流式记录；当前邻居计算为 O(N²)，全轨迹存储为 O(KN)，未做大规模性能保证。

上述内容均为下一阶段工作。本轮没有提前实现交通算法、城市模型或复杂资源调度。
