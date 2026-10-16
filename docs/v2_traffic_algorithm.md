# V2：论文核心交通算法实现与小规模验证

实现日期：2026-10-16。基于项目 V1 提交 `88db18a`，复用原有 `Simulation`、`PerfectObservation`、`advance`、轨迹记录；**没有修改 `simulator/` 和原作者 RobotSim 仓库**。

已实现论文 §3 的动态 Self-drive、各向异性 Repulsion、选择性 Friction，完成测试和实际实验。当前是**带明确工程补全的论文参考实现**：不能等同于作者未公开的实现，也不代表已复现论文大规模统计结果。Queueing、噪声、通信延迟及城市环境未实现。

## 1. 实现位置与论文对应

页码均为期刊印刷页码；PDF 第 1 页为 p.29。依据本地论文原文以及 [既有审计](paper_audit.md) 中已核对的官方 S2。

| 入口 | 内容 | 依据 |
|---|---|---|
| [geometry.py](../algorithms/geometry.py)：`braking_speed` | 分段制动曲线 D | p.35，式 (6) |
| 同文件：`constrained_candidate`、`maximize_on_disk` | C1–C5 的二维几何求解 | pp.37–38，Fig.1、C1–C5 |
| [self_drive.py](../algorithms/self_drive.py)：`danger_conditions`、`self_drive` | D1–D4、最紧迫邻居、迭代、方向恢复、目标减速 | §3.4，pp.36–38 |
| [interactions.py](../algorithms/interactions.py)：`repulsion` | 线性幅值和各向异性方向 | §3.2，式 (2)/(3)，pp.34–35 |
| 同文件：`friction_selection`、`friction` | 三条件筛选、允许速度差、超限速度协调 | §3.3，式 (4)–(6)，pp.35–36；幅值是假设 |
| [traffic.py](../controllers/traffic.py)：`TrafficController` | 按 ID 管理目标，输出三个速度分量之和 | p.34，式 (1) |
| [traffic_scenarios.py](../scenarios/traffic_scenarios.py) | 本项目设计的确定性场景 | 工程验证场景，不是论文实验配置 |
| [run_traffic.py](../experiments/run_traffic.py) | 对照实验、指标、诊断、绘图、离线重放 | 危险占用率参考 p.39 式 (7)；其他完成性指标自定义 |

控制器每 0.2 s 接收同步、只读的自身及通信半径内邻居快照，按自身 ID 查目标。依次计算 Self-drive、Repulsion、Friction，并相加交回 V1 速度跟踪。期望速度不是加速度；没有额外归一化到 `vSPP`，也没有擅加合成速度硬限幅。三个模式 `navigation`、`self_drive`、`full` 分别为不避碰目标导航、仅 Self-drive、完整三项合成。

## 2. Self-drive 的确定实现

**论文明确描述：** 令 `n=(r_j-r_i)/d`，初候选 `u=vSPP*g`，相对速度 `w=u-v_j`。D1 为 w 严格位于半角 `asin(min(Ravoid/d,1))` 的前向碰撞锥内；D2 为 `w·n>D(d,Ravoid,pavoid,aavoid)`；D3 为 `u·n>0`；D4 为 `(d-Ravoid)/|w|<t_plan`。必须四者同时成立。每轮处理最紧迫威胁，之后重新检查全部邻居。

**工程补全：** 每次控制调用固定 `t_plan=d_goal/vSPP`，内部迭代不更新；用 D4 的距离差／相对速率排序，平局按邻居 ID。它不是精确二次方程碰撞时间。负距离差保留；零相对速度不构成 D1/D4 威胁。默认数值容差 `1e-9`。

**C1–C5 求解推导：** 选取 `s=sign(cross(n,u_old-v_j))`，令 `t_s=rotate(n,s*theta)`，求最大化 `u·u_old`，满足：

```text
s * cross(t_s, u-v_j) >= 0     # C1，所选侧的碰撞锥切线外
s * cross(n,   u-v_j) >= 0     # C2，不换相对绕行侧
|u| <= |u_old|                 # C3，速度圆
n·u <= D(d,Ravoid,pavoid,aavoid) # C4，本机速度投影（不是相对速度）
```

C1 与 C2 联合将锥外区域缩为半平面交集。线性目标在闭圆盘与半平面交集上的最优点，必在无约束圆支撑点、直线／圆交点或直线／直线顶点中；实现枚举这些有限几何候选并检查全部约束。另加入圆心和边界垂足处理退化情形。**没有角度离散搜索，没有 ORCA 替换。** 最优性测试独立采用夹角判据、解析迎面最优解以及随机可行点的目标值上界检查；随机采样仅用于测试。

**退化与失败规则均属工程补全：**

- 切线边界可取；正好沿两机连线时选 `s=+1`。这是相对速度空间的逆时针侧，不是“绝对向左”。两机迎面时形成相反的世界坐标横移。C2 只约束本轮前后候选，不锁定跨控制周期的绕行侧。
- `d<=Ravoid` 时半角取 π/2；重合位置的 n 按 ID 选互为反向的 x 轴。该扩展只保证定义明确，不能消除已发生的距离违例。
- 候选重复、可行集为空或达到 32 次上限，记录 `cycle`／`infeasible`／`iteration_limit`，Self-drive 输出零作为制动请求。**不是瞬间停车，也不是安全保证；完整控制器仍会叠加 Repulsion/Friction。**
- 解出无威胁候选后，按 p.38 的条件，在速度已降低且背离目标时尝试保持模长转向目标，并重新检验 D1–D4。
- 最后将模长限制到 `D(d_goal,0,pavoid,aavoid)`。减速可能新生威胁，因此再查全部邻居，必要时继续求解，共享同一迭代预算。这一步重检是工程补全，已有实际几何用例测试。
- 目标距离 ≤0.5 m 且实际速度 ≤0.2 m/s 时 Self-drive 请求零；恰在目标点时也请求零。没有吸附位置、重置实际速度或模拟 Queueing。若其他分量推离目标，下次仍会重新导航。

## 3. Repulsion、Friction 与参数

**论文明确描述：** Repulsion 对 `d<10.5 m` 的邻居叠加 `1.3*(10.5-d)` m/s，按式 (3) 三分支变换方向。平行分类比较邻居速度与本机目标方向，角阈值为 π/3。第三分支使用 `A/2`，不是 `A²`；保留印刷公式在平行分类 φ=π/2 处的不连续，不做自拟平滑。

**工程补全：** 依据 `cross(g,n)` 对无符号角作镜像；共线对向时选择与 Self-drive 平局侧一致的横向排斥。静止邻居归到非平行类；目标方向未定义或位置重合时使用各向同性分离，重合时采用互反 ID 轴。多邻居按 ID 排序求和以免行顺序改变结果。

**论文筛选＋待验证幅值假设：** Friction 的 F1/F2/F3 分别用邻居朝本机 ±45°、邻居相对目标方向 ±120°、邻居速度相对目标方向 ±90°。F1、F2 都真则应用，都假则不应用，恰一个为真时要求 F3。F1 明确使用 `-q` 作为朝本机方向，消解原文相对位置符号疑点。静止邻居 F1 为假、F3 为真；重合位置例外视为 F1/F2 为真；无目标方向时普通移动邻居的 F2/F3 为假。

采用 `friction_gain * max(|v_j-v_i|-allowed(d),0) * unit(v_j-v_i)`，其中 `allowed=max(0.5,D(d,10.7,1,3))`。旧 C 的 `src/utilities/interactions.c::FrictionLinSqrt` 支持“超限幅值”形式，**不能证实这就是本文隐藏实现**。默认增益 1 是本项目假设，不是 S2 参数，也没有照搬旧增益 0.023802。

| 参数组 | 默认 SI 值 | 来源 |
|---|---|---|
| Repulsion | 半径 10.5 m，增益 1.3 s⁻¹，A=0.5 | 官方补充 p.3，S2；正文 p.35 的 A=0.42 另作敏感性检查 |
| Friction | 半径 10.7 m，允许小速度差 0.5 m/s，p=1 s⁻¹，a=3 m/s² | S2 |
| Self-drive | vSPP=8 m/s，Ravoid=13 m，pavoid=0.5 s⁻¹，aavoid=3 m/s² | S2 |
| 动力学／通信 | τ=1 s，amax=6 m/s²，通信 80 m，控制周期 0.2 s | S1 对应值；保持 V1 实现 |
| 积分 | dt=0.01 s，旧速度积分位置 | V1／原代码；不是已确认的论文步长 |
| 观测 | 完美即时观测，无噪声、无延迟 | 本轮验证简化；不等同 S1 全环境 |

统一参数在 `TrafficParameters`；追越场景的 4 m/s 前机、近距同向的双机 4 m/s 通过按 ID 的速度覆盖配置，明确区别于 S2 同质机队。

## 4. 实际实验结果

运行 **9 个场景 × 3 种控制器 = 27 组**，另有 5 组参数敏感性实验。所有结果有限，实际加速度满足 V1 的 6 m/s² 模长约束。初态与完整目标均在各场景 `config.json` 中；无随机扰动。

下表为默认完整控制器，时间为最后一次进入并持续满足“距离≤0.5 m、速度≤0.2 m/s”的时刻。每机详细航程、初次到达与最终误差见 JSON。

| 场景／初态 | 时长 | 最小机间距 m | 完成数 | 稳定到达 s | 主要观察 |
|---|---:|---:|---:|---|---|
| A `single`：静止，直达 80 m | 30 s | — | 1/1 | 17.35 | 航程 82.90 m，存在末端超调后修正 |
| B `head_on`：间隔 120 m，静止 | 50 s | 14.129 | 2/2 | 23.31 / 23.31 | 提前侧向避让，恢复目标方向 |
| C1 `crossing`：两机均距交点 60 m，静止 | 50 s | 12.992 | 0/2 | 未到达 | 持续运动却无目标进展，活锁候选 |
| C2 `crossing_offset`：距交点 30/100 m，静止 | 50 s | 49.497 | 2/2 | 22.35 / 27.35 | Self-drive 全程零修正；能区分错时交叉 |
| D `overtaking`：间隔 45 m，后机8、前机4 m/s | 60 s | 16.971 | 2/2 | 42.62 / 33.72 | 后机航程198.36 m，对比直线180 m，有效率代价 |
| E `five_agents`：半径70 m正五边形，静止 | 70 s | 15.521 | 0/5 | 未到达 | 1545次循环、8次不可行，制动后死锁 |
| C1敏感性 `crossing_cruise`：相同几何，初速8 m/s | 50 s | 14.312 | 2/2 | 22.82 / 23.99 | 通过交叉，零回退 |
| E敏感性 `five_agents_cruise`：相同几何，初速8 m/s | 70 s | 13.665 | 5/5 | 均27.61 | 最大3轮修正，零回退 |
| D2 `close_parallel`：间隔8.54 m，同速4 m/s | 50 s | 8.544 | 2/2 | 26.62 / 27.78 | 有安全间隔、零初始相对速度，激活近距分量 |

迎面对照：不避碰导航穿越对方，连续步内最小距离约 **0 m**；仅 Self-drive 的最小距离 **14.973 m**、到达 **22.71 s**；完整控制器为 **14.129 m / 23.31 s**。加入 Friction 并不保证距离或效率单调改善。

**指标定义：** `pair_metrics` 沿 V1 每个积分区间的线性位置段计算真实步内最小距离和 `d<3 m` 占用时长，避免仅看采样端点漏检穿越。输出危险事件数、危险对·秒，以及式 (7) 的时间平均配对占用率；3 m 取自论文 Fig.7 p.47，和 13 m 避让半径不同。全部 18 组含避碰模式的主实验均无 3 m 违例，**其中4组仍未完成任务**。近距同向场景从 Ravoid 内出发，但相对速度为零、距离大于危险阈值，不属于初始物理碰撞。

死锁／活锁仅是有限时间诊断：未完成、末10 s目标净进展不足0.5 m时，若该窗口最大速度<0.2 m/s则标死锁候选，否则标活锁候选，不声称证明无限时间死锁。静止初态交叉的每机最终目标误差48.922 m，末10 s进展约9×10⁻⁶ m；这组不能以“求解无回退”或“无碰撞”算成功。

**三个分量的实际作用：** Self-drive 负责远距离预测，正常避让时 Repulsion 可以完全不触发，符合 p.36 的设计说明。迎面场景 Friction 最大3.510 m/s；近距同向场景 Repulsion最大2.543 m/s、Friction最大1.128 m/s，证实二者确实接入。单机仅由 Self-drive 驱动，因 τ=1 s 跟踪和0.2 s控制保持出现轻微目标超调，没有隐式吸附。五机巡航初态的最大合成指令8.309 m/s，说明 vSPP 并非合成速度硬上限。

**参数敏感性：** Friction 增益0.5／1／2的迎面最小距离分别14.972／14.129／14.157 m，到达23.15／23.31／23.59 s，均完成且无危险违例。近距同向改变A至0.42仍完成，排斥方向与瞬时合成指令有变化，不能据此消除正文／S2差异。5组完整产物在 `results/traffic_sensitivity/`。未用这些结果回调默认参数。

## 5. 验证、问题定位与剩余限制

**实测 102 passed：原有 V1 47项＋V2 55项。** V1 的原 C 动力学片段对照仍通过。V2 覆盖 D 分段与连续性、各危险条件、C1–C5独立约束／目标验证、对称／静止／重合／零相对速度、多个威胁重查、循环与不可行及上限回退、方向恢复、减速后新威胁、排斥三分支、Friction筛选、合成、快照不变、ID行重排、运行输出与GIF重放。JUnit记录在 `results/pytest_v2.xml`。

另对27组主实验逐一核验源码SHA256、有限状态和加速度上限；从保存的实际位置／速度独立重建11,421个标为 `optimal` 的求解步骤，全部通过C1角度、C2侧向和C3/C4约束检查，C3/C4最大正残差为1.78×10⁻¹⁵ m/s。该检查验证单步约束，不把循环回退解释成全局收敛。

实际问题与归因边界：

- **实现中已处理：** 目标减速后原先安全的相对速度可能再次危险，加入显式重检并用独立几何状态回归测试；在多个邻居之间循环不能当作收敛，加入候选重复检测和可见的失败日志；连续步内检测避免基线穿越被采样漏掉；区分运动活锁和停车死锁。
- **局部协调限制／工程补全相关：** 完全同步从静止起步时，自身候选已取8 m/s，邻居观测速度仍在加速；与巡航初态相比，相对速度绕行侧和决策会改变。交叉会出现协调振荡，五机反复处理不同威胁可回到旧候选。C1–C5只约束当前选中邻居，没有要求累计先前所有约束；当前实现忠实保留重查流程，并显式回退。独立单步约束与最优性测试通过，未发现能解释这些结果的单步求解错误，但这不是全局正确性证明。
- **尚未验证：** 作者是否使用不同规划时域更新、跨时刻侧向记忆、特定对称破缺、Friction幅值或其他失败恢复。初始间距足够且巡航初态能通过，不能将静止初态失败归咎于“初始已经物理不可避免碰撞”。也不能由这些小规模反例直接否定作者完整系统。
- **性能限制：** JSON提供控制调用平均／p95耗时，不含观测构造、记录和绘图；本机主要为亚毫秒量级，五机循环案例较慢。这是小样本运行测量，不是实时性或大规模性能保证。诊断默认保存在内存，后续大规模运行应关闭 `record` 或改流式输出。

## 6. 运行与检查产物

```bash
conda activate drone-traffic
export MPLCONFIGDIR=/tmp/drone-traffic-matplotlib
python -m pytest -q --junitxml=results/pytest_v2.xml
python -m experiments.run_traffic --scenario all --mode all --gif
python -m experiments.run_traffic --scenario head_on --mode full --friction-gain 0.5 --output results/gain05
python -m experiments.run_traffic --scenario close_parallel --mode full --anisotropy 0.42 --output results/a042
python -m experiments.run_traffic --replay results/traffic/crossing_cruise/full
```

无需新增项目依赖。`pypdf`仅临时安装到 `/tmp/v2-pdf` 供核对本地原文，未加入运行环境或 requirements。项目输出受现有 `.gitignore` 的 `/results/` 规则排除，保留在本地，可用上述命令重建。

每个 `results/traffic/<场景>/<模式>/` 包含 `trajectories.png`、`speed.png`、`distances.png`、`trajectory.npz`、`trajectory.csv`、`speed.csv`、`config.json`、`summary.json`、`diagnostics.jsonl`。迎面与两种同步交叉的完整控制器另有 `replay.gif`。NPZ可 `allow_pickle=False` 读取；重放仅读取保存状态，不重新执行算法。配置包括源码SHA256、运行时环境和当时基础Git版本；未提交实现期间生成的结果应以文件哈希核验代码，不能仅依赖基础提交号。

诊断每条对应一次控制调用，记录时间、ID、三分量、合成、迭代数、每轮全部危险邻居及选中ID、修正前后速度、绕行侧、求解状态、退出原因和耗时。记录位于控制时刻，区间指令使用 V1 `applied_desired_velocities`；未改动原有数据对齐。

直接查看：[汇总指标](../results/traffic/summary.csv)、[迎面GIF](../results/traffic/head_on/full/replay.gif)、[交叉失败GIF](../results/traffic/crossing/full/replay.gif)、[巡航交叉GIF](../results/traffic/crossing_cruise/full/replay.gif)、[五机巡航轨迹](../results/traffic/five_agents_cruise/full/trajectories.png)。

下一阶段应先冻结本版为参考基线，围绕**时域更新、对称破缺／跨周期侧向稳定性、循环恢复、Friction幅值**做独立可切换的规则实验，保留本轮失败回归。随后接入 V1 的延迟／噪声观测接口，核验 S1 条件下的完成率与安全性；Queueing和共享资源容量应作为后续独立模块，当前不宜直接扩展城市环境或宣称大规模论文复现完成。
