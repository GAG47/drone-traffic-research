# 《Decentralized Traffic Management of Autonomous Drones》论文与开源代码审计

审计日期：2026-10-15（Asia/Shanghai）。范围：本地论文、官方补充材料、作者公开 RobotSim 仓库及现有工具链；本轮没有修改作者源码，没有实现交通算法，没有运行论文实验。

## 1. 审计结论与证据口径

**当前公开仓库是可复用的通用无人机动力学／Flocking 仿真基础，不是本文交通算法的完整开源实现。** 本地版本与审计日远端 HEAD 一致；实际算法入口沿当前速度方向生成 SPP 项，没有目标驱动的 D1–D4／C1–C5 预测迭代，也没有本文选择性 Friction、各向异性 Repulsion 和 Queueing 的完整调用链。不能用现有程序编译成功或出现 Repulsion／Friction 名称作为“本文已经复现”的证据。

**论文给出了可以工程化的核心规则，但尚不足以在所有状态下唯一确定实现。** 阻碍严格复现的主要问题是式 (4) 的幅值／量纲问题、Self-drive 规划时域更新缺失、退化几何与迭代保护未定义、Queueing 距离规则不完整，以及正文、图注和补充材料的若干不一致。S1/S2 已取得，不存在“全部实验参数不可获取”的问题。

建议先采用 **B：Python + NumPy 参考实现**，用小规模确定性案例验证每一项公式，再加入按 C 框架核对的动力学、延迟和噪声，复查 Fig. 2；把原 C 仓库保留为只读对照。若以后性能不足，再测量瓶颈并迁移数值内核。这个建议是工程判断，不意味着 Python 已具备论文等效性。

全文使用四类证据标记：

- **[论文明确描述]**：正文或官方 Supplementary Materials 中可直接定位的内容；若不同位置相互矛盾，分别保留。
- **[开源代码证实]**：当前固定提交中的实际代码行为；静态阅读与运行验证分开说明。
- **[工程推断]**：由公式、量纲或源码导出的判断和开发建议，不冒充作者实现。
- **[尚未验证]**：未公开、未说明、缺少运行证据或仍存在多种解释的内容。

页码统一用期刊印刷页码；本地 PDF 第 1–25 页对应期刊 pp. 29–53，因此“p. 35”就是 PDF 第 7 页。补充材料另以“补充 p. 3”等引用。源码路径均相对于项目根目录，行号对应下述固定提交。

## 2. 资料完整性、版本和工具环境

### 2.1 已找到的资料

| 项目 | 审计结果 |
|---|---|
| 工作目录 | /home/gag47/projects/drone-traffic-research |
| 正文 | [papers/Decentralized traffic management of autonomous drones.pdf](../papers/Decentralized%20traffic%20management%20of%20autonomous%20drones.pdf)，25 页，元数据 DOI 与目标论文一致，逐页提取无缺页 |
| 正文版本 | Swarm Intelligence 19:29–53 (2025)；首页记载 2024-07-11 在线发表。这两种年份并不矛盾 |
| 原仓库 | [references/robotsim/](../references/robotsim/)，58 个 Git 跟踪文件，工作树干净；git fsck --full 成功，6 个历史提交 |
| 固定提交 | ea54b97e7e2659449e721ebd4a2285beb9b0e9ce；提交时间 2019-07-28，提交说明 Delete temp |
| 远端核对 | origin = https://github.com/csviragh/robotsim.git；git ls-remote 查到 HEAD 和唯一公开分支 master 均为上述 hash，没有返回标签 |
| README | 本地 HEAD 的跟踪文件中没有 README；GitHub 根目录也没有 README。替代阅读了 doc/usersdoc.pdf（7 页，2019 年手册） |
| 补充材料 | 已从 Springer 官方链接取得 11721_2024_241_MOESM1_ESM.pdf，855,369 字节、7 页；包含 Tables S1/S2、Figures S1/S2、Text S1 和视频链接 |
| 实验专用资料 | 未发现本文的 traffic 源码、运行配置全集、随机种子清单、批量实验脚本、优化器脚本或原始统计数据 |

**[开源代码证实]** “仓库完整”在这里表示本地已取得当前公开版本的全部跟踪文件，不能推导为“论文实验代码完整”。**[论文明确描述]** 论文 Data availability／p. 39 脚注只把链接称为 generic robotic simulation framework；[作者项目页](https://hal.elte.hu/drones/swarmint2024.html)也称 code basis，未承诺公开本文完整算法。

正文 SHA-256：

~~~text
d8318f732ea72aa49eb03f679b7d4658e731fd78ecf0c90fe9f543eafe1c7cd7
~~~

官方补充 PDF SHA-256：

~~~text
b14a277d83c0a169422bddbd2c7797bd257f73ab0880f2cadad50e8bc88a590b
~~~

补充 PDF 来自[期刊页面的 Supplementary Information](https://link.springer.com/article/10.1007/s11721-024-00241-y)中的[直接下载地址](https://media.springernature.com/original/springer-static/esm/art%3A10.1007%2Fs11721-024-00241-y/MediaObjects/11721_2024_241_MOESM1_ESM.pdf)。浏览工具下载超时后，通过只读 HTTP 下载成功，临时保存于 /tmp/drone-supplement.pdf；本报告的参数取自正式版补充材料。

### 2.2 构建、依赖和本机检查

**[开源代码证实]** 主体为 C，使用 GCC + GNU Make；Makefile 只有 spp_evol 算法目标，输出 robotflocksim_main。没有 CMake、Python 仿真入口或外部物理引擎。默认链接 libm、OpenGL、GLU、GLUT；pngout=true 额外链接 DevIL 的 IL、ILU、ILUT，debug=true 增加调试宏，默认构建还带 -pg。许可证为 MIT。

tools/install.sh 调用 apt 安装 freeglut3-dev、libdevil-dev 和旧的 python 包；doc/usersdoc.pdf p. 3 将 Python 2.x 描述为统计辅助工具。Python 不是该 C 可执行文件的必要语言运行时；旧安装脚本也不能作为 Ubuntu 24.04 上已验证的安装方案。

| 检查 | 实测结果及含义 |
|---|---|
| GCC / Make | GCC 13.3.0、GNU Make 4.3 已存在 |
| Conda | drone-traffic 环境存在，解释器为 Python 3.11.17；其中 numpy、scipy 和常用 PDF 库未安装 |
| 当前 shell 的 Python | 实际落在 f5-tts 环境，而非 drone-traffic；后续命令应显式选择目标解释器 |
| 构建命令检查 | make -n spp_evol 成功展开 GCC 命令；仅证明目标存在 |
| 只读编译探测 | gcc -fsyntax-only src/robotsim_main.c 失败：GL/gl.h 缺失；尚未达到链接和运行阶段 |
| 图形库 | 发现 libGL.so.1 运行库；未发现所需 GL 头文件及 GLU/GLUT 开发环境，不能声称可直接构建 |
| PDF 工具 | 系统没有 pdftotext/pdfinfo；仅在 /tmp/drone-audit-pdf 放入 pypdf 6.20.1 和 pypdfium2 5.14.0，用于提取和目视核对，未改 Conda 环境 |

**[尚未验证]** 完整 C 构建、无界面运行、WSL 图形显示、现代编译器下更后续的兼容问题及仿真性能。本轮没有运行安装脚本，也没有安装图形开发包。

## 3. 代码结构、动力学和可复用范围

### 3.1 主要模块

| 路径／关键函数 | 实际职责 | 本文复用价值与限制 |
|---|---|---|
| src/robotsim_main.c，main、各运行循环 | 参数读取、初始化、GUI／-novis 分支、历史状态、统计输出 | 通用主循环可参考；-novis 是运行选项，当前 Makefile 仍编译链接 GUI |
| src/algo.h | InitializePhase、CalculatePreferredVelocity、HandleOuterVariables、DestroyPhase 等算法接口 | 可以接入新 traffic 模块，但现有接口实现不是本文算法 |
| src/algo_spp_evol.c，InitializeFlockingParams、CalculatePreferredVelocity | SPP、自推进、全邻居排斥／摩擦、场地和障碍物作用 | 旧 Flocking 基线；不能原样当 traffic controller |
| src/robotmodel.c，CreatePhase、RealCoptForceLaw、Step | 局部观测、延迟、速度松弛、加速度上限、积分、外噪声 | 最有价值的动力学／观测参照；论文使用版本是否逐行一致未证实 |
| src/sensors.c，StepGPSNoises | 有恢复力和阻尼的相关 GPS 误差过程 | 可核对噪声生成机制；不能简化成每步独立位置白噪声后仍声称等效 |
| src/utilities/dynamics_utils.h:36，phase_t | Coordinates、Velocities、InnerStates、RealIDs、数量 | 通用状态容器；没有显式 target 字段，现有 SPP 设置 NumberOfInnerStates=0 |
| src/utilities/dynamics_utils.c，SelectNearbyVisibleAgents、SwapAgents、AllocatePhase | 状态管理、通信半径筛选、距离相关丢包 | 可用作局部观测语义参照；没有空间哈希／树索引 |
| src/utilities/math_utils.c，VelDecayLinSqrt、StoppingDistanceLinSqrt、SigmoidLin | 制动曲线、反函数、线性截断排斥、向量几何 | 制动曲线与式 (6) 最接近；仍须检查饱和参数和单位 |
| src/utilities/interactions.c，RepulsionLin、FrictionLinSqrt | 各向同性短程排斥和全邻居速度对齐 | 只能复用部分标量函数／计算形式 |
| src/utilities/arenas.c，ParseArenaFile、NearestArenaPoint | 圆／方形预定义场地、归一化多边形和最近边界点 | 场地工具；NearestArenaPoint 注释限定凸多边形、2D |
| src/utilities/obstacles.c，ParseObstacleFile；algo_spp_evol.c，Shill_Obstacle_LinSqrt | 2D 多边形障碍解析、虚拟 shill 障碍作用 | 有静态几何基础；不等于城市建筑／航路／起降容量模型 |
| src/stat.c，RatioOfDangerousSituations | 危险距离内配对比例及一般运动统计 | 可对应式 (7) 的瞬时部分 |
| src/algo_spp_evol_stat.c，SaveClusterDependentParams、SaveModelSpecificStats | 群体簇、速度相关性、越界统计 | 属于 Flocking 评价，不提供式 (8)/(9) 的目标到达统计 |
| src/vizualizer/、dynspecviz.c、objects_menu.c、colors.c | OpenGL 绘图、交互、参数界面 | 方便人工观察，独立复现论文数值不要求沿用 |
| config/ | 输出开关、颜色 | 没有本文实验场景；颜色中出现 TrafficCircle／Target 不证明算法存在 |
| parameters/ | initparams.dat、unitparams.dat、flockingparams.dat、arenas.default、obstacles.default | 旧默认配置；障碍示例顶点均被注释，不能视为已启用城市场景 |

### 3.2 单步仿真调用链

**[开源代码证实]** 主要路径如下，依据 src/robotmodel.c:183–385：

~~~text
主循环
  → Step
    → 取本步真实状态与 t_del 对应历史状态
    → x_next = x + v_old * DeltaT
    → 到传感器周期时 StepGPSNoises
    → 对每个 agent 构造 CreatePhase 局部观测
    → RealCoptForceLaw
      → 到 t_GPS 周期时 CalculatePreferredVelocity
      → 保持上次 preferred velocity，按 tau_PID 松弛更新速度
    → 对确定性速度增量作 a_max 饱和
    → AddNoiseToVector 再叠加外部速度扩散噪声
    → 写入下一状态与历史缓存
~~~

位置使用旧速度的显式 Euler 更新。RealCoptForceLaw 的水平分量实质为

$$
v_i^{+}=v_i^{\mathrm{true}}
+\frac{\Delta t}{\tau_{XY}}
\left(\tilde v_i-v_i^{\mathrm{observed}}\right),
$$

竖直方向使用独立的 \(\tau_Z\)。这是速度跟踪的一阶松弛与位置积分组成的简化二阶运动模型，不是六自由度刚体／电机／姿态动力学。[src/robotmodel.c:211](../references/robotsim/src/robotmodel.c)、RealCoptForceLaw。

**[开源代码证实]** Step 先将确定性 \(\Delta v\) 的三维模长限制为 \(a_{\max}\Delta t\)，再加外噪声。因此 \(a_{\max}\) 不是包含白噪声后每一步总速度增量的绝对上界；源码在噪声之前保存加速度统计。AddNoiseToVector:148–150 的速度噪声增量为各分量 \(\sqrt{2\Sigma_{\mathrm{outer}}\Delta t}\,\xi\)。

**[开源代码证实]** StepWind 的调用在 Step:304 被注释；当前 AddNoiseToVector 也未使用传入的风向量。存在 Wind 参数和函数，不代表当前主循环具有有效风动力学。

### 3.3 邻居、通信和测量误差

**[开源代码证实]** CreatePhase（src/robotmodel.c:25–130）先复制全体状态；在传感器更新时调用 SelectNearbyVisibleAgents，将通信范围内且未丢包的 agent 打包到数组前端，并使自身位于索引 0。函数名和注释中含“distance”，但当前实际调用是筛选／打包，不是逐邻居距离排序。

邻居资格依据当前真实位置筛选；邻居的位置和速度随后由延迟历史状态替换，再叠加延迟 GPS 误差；自身使用当前测量状态。非更新时刻局部数组只保留自身，而 preferred velocity 沿用上次结果。这里没有显式 UDP 网络或按时间戳外推的实现。

SelectNearbyVisibleAgents（src/utilities/dynamics_utils.c:1491）使用

$$
P_{\mathrm{loss}}(d)\ \text{对应比较阈值}\
d^2\frac{\text{packet\_loss\_ratio}}
{\text{packet\_loss\_distance}^2}
$$

与均匀随机数比较；若阈值超过 1，比较自然总为真。unitparams.dat 的丢包率为 0，但论文 S1 没有给出丢包率，不能推定本文也为 0。

StepGPSNoises（src/sensors.c:30）同时维护位置和速度误差，用恢复力、阻尼与随机驱动产生时间相关误差；\(\lambda_{GPS}=0.1\) 是硬编码。**[工程推断]** 其 /300 常数需要随原代码的厘米单位体系一起做量纲审查，移植到 SI 后不能直接照抄。**[开源代码证实]** GPSPhase 和 GPSDelayedPhase 是分别演化的误差容器，不能未经核对就假定后者严格来自前者的同一随机轨迹延迟。

**[工程推断]** 该框架的“局部决策”不等于整机仿真每步计算量只取决于局部邻居数：CreatePhase 对每个 agent 复制／扫描全体状态，基本遍历为 \(O(N^2)\)；碰撞统计也有全配对扫描。论文 p. 42 关于固定局部通信半径的机载计算可扩展性，不是对这个集中式仿真实现耗时的线性保证。

### 3.4 现有 SPP 与本文的决定性差别

**[开源代码证实]** CalculatePreferredVelocity（src/algo_spp_evol.c:417）：

1. :456–461 将本机当前观测速度归一化后乘 V_Flock，作为 SPP 自推进；没有指向个体目的地。
2. :464 调用 RepulsionLin；方向始终是远离邻居的位置方向，没有目标方向、速度方向分类或 \(A\)。
3. :468 调用 FrictionLinSqrt；遍历全部局部邻居，没有 §3.3 的三个交通筛选条件。
4. :471–486 叠加场地、障碍 shill 项。
5. :489–493 的静态 CutOffMode=false，实际把合成向量再次归一化到 V_Flock。虽然注释写“saturates at V_Max”，V_Max 截断仅在另一分支；当前默认路径并非简单限幅。

**[开源代码证实]** 没有发现：个体目标状态与随机换目标机制、动态速度障碍迭代、D1–D4／C1–C5、Queueing、成对优先级和按方向选层。结论同时基于完整跟踪文件清单、全仓搜索和上述真实入口；没有把“关键词未命中”单独当充分证据。

**[论文明确描述]** 本文 §3.4 p. 38 明确区分其动态预测与 2018 年交通方法的静态邻居避让。但当前 SPP 文件既不能证明实现了 2025 正式版方法，也不能证明完整实现了 2018 年 traffic 算法。algo_spp_evol 的 “evol” 注释和参数不等于附带了本文的进化优化程序。

## 4. 论文算法：输入、输出及可实现程度

### 4.1 问题边界与控制接口

**[论文明确描述]** §2 pp. 32–33 面向能全向运动、速度范围包含悬停的多旋翼，主要问题是开放空间中的二维交通；各机有独立目标，通过局部通信去中心协调。三维实验是分层交通扩展，不是完整自由三维规划。城市静态障碍避让在 §6 p. 49 被列为本文范围之外。

每机广播位置、速度、目的地（§3.1 p. 34）。单次控制输入至少包括自身位置／速度／目标、局部邻居位置／速度／目标，以及参数；输出是期望速度：

$$
\tilde{\mathbf v}_i
=\mathbf v_i^{rep}+\mathbf v_i^{friction}
+\mathbf v_i^{self-drive}.\tag{论文 1}
$$

这些是**速度项，不是直接施加的力或加速度**。底层跟踪和加速度约束属于动力学。正文／S2 未给出对式 (1) 合成向量统一施加硬速度上限的完整规则；\(v^{SPP}\) 是自由运动的期望速度，不能直接等同于实际速度始终不超过的 \(v_{\max}\)。

### 4.2 Repulsion：式 (2)/(3)，pp. 34–35

**[论文明确描述]**

$$
\mathbf v_i^{rep}
=\sum_{\substack{j\ne i\\d_{ij}<R_0}}
p^{rep}(R_0-d_{ij})\,\hat{\mathbf u}^{rep}_{ij},
\qquad d_{ij}=\|\mathbf r_j-\mathbf r_i\|.\tag{论文 2}
$$

参数为排斥半径 \(R_0\)、线性增益 \(p^{rep}\) 和各向异性强度 \(A\)。每对的幅值随距离线性变化；该式没有旧代码的 V_Rep 饱和参数，也没有除以邻居数。

设 \(\hat{\mathbf r}_{ti}\) 是本机指向目标的单位向量，\(\phi\) 是邻居相对位置与该目标方向的夹角，\(\rho\) 是排斥方向与目标反方向 \(-\hat{\mathbf r}_{ti}\) 的夹角：

$$
\rho=
\begin{cases}
(1-A)\phi,
&\phi\le \pi/2,\ \mathbf v_j\uparrow\uparrow\hat{\mathbf r}_{ti},\\
\pi+(1-A)(\phi-\pi),
&\phi>\pi/2,\ \mathbf v_j\uparrow\uparrow\hat{\mathbf r}_{ti},\\
(1-A/2)(\phi-\pi)+\pi,
&\mathbf v_j\uparrow\downarrow\hat{\mathbf r}_{ti}.
\end{cases}\tag{论文 3}
$$

第三行是 **\(A/2\)，不是 \(A^2\)**，已目视核对 PDF。平行分类指 \(\angle(\mathbf v_j,\hat{\mathbf r}_{ti})<\pi/3\)，其他情况归到另一类；不是直接比较两机实际速度是否平行。大意是同向交通更倾向纵向间隔和成列，对向交通更倾向侧向分离。

输入需要本机位置、目标方向、邻居位置和邻居速度方向；输出是叠加后的排斥速度向量。

**[尚未验证]** 正文没有明确给出从标量夹角恢复二维旋转符号的完整约定；正面对称、邻居静止、本机已在目标和重合位置时需要补充定义。式 (3) 在平行分类、\(\phi=\pi/2\) 的左右极限相差 \(A\pi\)，这是按印刷公式导出的不连续，不能擅自改成平滑公式。正文 p. 35 说仿真 \(A=0.42\)、实机 0.5；S2 却两者均为 0.5，见第 7 节。

### 4.3 Friction：式 (4)–(6)，pp. 35–36

**[论文明确描述]** 定义 \(\mathbf w_{ji}=\mathbf v_j-\mathbf v_i\)、\(w_{ji}=\|\mathbf w_{ji}\|\)、\(\hat{\mathbf w}_{ji}=\mathbf w_{ji}/w_{ji}\)。为便于审计，这里用 w 避免与速度向量混淆；原文标量写为 \(v_{ji}\)：

$$
\mathbf v_i^{friction}
=\sum_{j\in\mathcal A_i}
H\!\left(w_{ji}-v^{frictmax}(d_{ij})\right)
\hat{\mathbf w}_{ji}.\tag{论文 4，原样}
$$

论文明确说 \(H\) 是 Heaviside 阶跃函数。

$$
v^{frictmax}(d)=
\max\left(v^{friction},
D(d,R^{friction},p^{friction},a^{friction})\right),\tag{论文 5}
$$

$$
D(d,R,p,a)=
\begin{cases}
0,&d-R<0,\\
p(d-R),&0\le d-R\le a/p^2,\\
\sqrt{2a(d-R)-a^2/p^2},&d-R>a/p^2.
\end{cases}\tag{论文 6}
$$

\(D\) 输出允许的相对速度尺度；\(R\) 是距离偏移，\(p\) 是线性段增益，\(a\) 控制平方根制动段，短距离线性段避免平方根在零处的无穷斜率。\(v^{friction}\) 为允许的小幅速度差，防止所有相近飞机完全锁速。这里的 \(a^{friction}\) 是制动曲线参数，区别于物理 \(a_{\max}\)。

**[论文明确描述]** 邻居集合 \(\mathcal A_i\) 的逻辑可以整理为下表；这是对文字逻辑的重排，不是增加筛选条件：

| \(F_1\)：邻居朝本机运动 | \(F_2\)：邻居在本机与目标之间的方向区域 | \(F_3\)：对齐邻居仍有利于目标进展 | 应用摩擦 |
|---|---|---|---|
| 真 | 真 | 任意 | 是 |
| 假 | 假 | 任意 | 否 |
| 两者恰有一个为真 | | 真 | 是 |
| 两者恰有一个为真 | | 假 | 否 |

原文给出的三个角阈值分别为 \(\pm\pi/4\)、\(\pm2\pi/3\)、\(\pm\pi/2\)。第二个阈值确为 120°，不能自行收窄为 90°。“朝本机”的第一条件存在符号疑点：p. 34 把相对位置写为 \(\mathbf r_j-\mathbf r_i\)，p. 36 又说用 \(\mathbf r_{ij}\) 与 \(\mathbf v_j\) 的 \(\pm\pi/4\) 检验接近；若直接以零夹角检验，方向反而是远离本机。应确认其相对位置方向／角度中心，不能静默反号后称为论文原式。

输入是位置、速度、目标方向和上述制动／筛选参数；输出是降低所选邻居相对速度的速度修正向量。

**[工程推断：重要公式问题]** 式 (4) 右边按原样只有无量纲的阶跃和单位向量，不能与式 (1) 中其他 m/s 项量纲相加，也没有随速度差超限程度改变的幅值。旧代码 FrictionLinSqrt:65–68 实际使用

$$
C_{\mathrm{Frict}}\,
\max(w_{ji}-v^{frictmax}(d),0)\,\hat{\mathbf w}_{ji},
$$

但旧代码中的这一事实**不能证明本文式 (4) 应当补写该因子**，更不能直接沿用旧 C_Frict=0.023802。候选解释包括漏写速度差幅值或未写出的增益；正式实现前需明确采用哪种解释并做敏感性实验。

### 4.4 Self-drive：初始化与危险检测，pp. 36–37

为避免正文 §3.2／§3.4 相对位置下标不一致，以下固定

$$
\mathbf q_{ij}=\mathbf r_j-\mathbf r_i,\quad
d_{ij}=\|\mathbf q_{ij}\|,\quad
\mathbf n_{ij}=\mathbf q_{ij}/d_{ij},\quad
\Delta\mathbf v_{ij}^{(s)}
=\mathbf v_i^{(s)}-\mathbf v_j.
$$

这里 \(\mathbf n_{ij}\) 一律从本机指向邻居；对应 §3.4 原文的 \(\hat{\mathbf n}_{ji}\)。这只是记号统一。

**[论文明确描述]** 每次控制计算内部进行候选速度迭代：

$$
\mathbf v_i^{(0)}
=v^{SPP}\frac{\mathbf x_i^{target}-\mathbf r_i}
{\|\mathbf x_i^{target}-\mathbf r_i\|}.
$$

\(t_{\mathrm{plan}}^{(0)}\) 初始化为到达目标所需时间。若按当前直线自由速度估计，则是 \(d_{target}/v^{SPP}\)；**[工程推断]** 这是自然数值化表达，正文未说明是否将有限加速度、临近目标减速或 Queueing 时间计入。

只有四个条件**同时满足**才将邻居 j 判为本轮 threatening：

| 条件 | 论文规则及含义 |
|---|---|
| D1 | \(\angle(\Delta\mathbf v_{ij}^{(s)},\mathbf n_{ij})<\arcsin(\min(R^{avoid}/d_{ij},1))\)。相对速度位于指向邻居避让圆的碰撞锥内；使用邻居当前速度，不把邻居当静态障碍 |
| D2 | \(\Delta\mathbf v_{ij}^{(s)}\cdot\mathbf n_{ij}>D(d_{ij},R^{avoid},p^{avoid},a^{avoid})\)。闭合速度超过本机有限制动能力允许值 |
| D3 | \(\mathbf v_i^{(s)}\cdot\mathbf n_{ij}>0\)。本机对接近至少承担一部分责任 |
| D4 | 原文 \((d_{ji}-r^{avoid})/\lvert\Delta\mathbf v_{ij}\rvert<t_{\mathrm{plan}}^{(s)}\)。危险在规划时域内出现；原文此处半径用了小写 r，且相对速度漏了迭代上标 |

**[论文明确描述]** 没有危险邻居时接受当前候选速度；有多个时，选择“最早靠得太近”的邻居 k 先处理。

**[工程推断]** 如果以 D4 的距离差／相对速率作为紧迫度，应取满足 D1–D4 的邻居中该值最小者。不过该表达式不是一般斜向相对运动的精确圆边界首次接触时间。使用精确二次方程求交时间会改变排序，不能当作论文已规定的等价替换。

### 4.5 Self-drive：C1–C5 与下一候选，p. 38、Fig. 1 p. 37

对已选邻居 k，**[论文明确描述]** 下一候选 \(\mathbf u=\mathbf v_i^{(s+1)}\) 满足：

| 条件 | 含义 |
|---|---|
| C1 | \(\mathbf u-\mathbf v_k\) 离开从本机向邻居 \(R^{avoid}\) 圆作两切线界定的碰撞锥 |
| C2 | 新旧相对速度在两机连线同一侧，保持本次绕行侧 |
| C3 | \(\|\mathbf u\|\le\|\mathbf v_i^{(s)}\|\)，危险处理中不加速 |
| C4 | \(\mathbf u\cdot\mathbf n_{ik}\le D(d_{ik},R^{avoid},p^{avoid},a^{avoid})\)，限制本机朝邻居运动的分量。原文使用 \(\hat{\mathbf n}_{ki}\)，与 §3.4 定义一致 |
| C5 | 在满足 C1–C4 的候选中，最大化 \(\mathbf u\cdot\mathbf v_i^{(s)}\) |

Fig. 1(b) 画出以速度原点为中心的 C3 圆、由邻居速度平移的禁区以及 C4 半平面。C5 优化的是与**上一候选**的点积，不能替换成最大化与最初目标方向的点积，也不是随意在固定角度网格中找一条可行方向。

**[工程推断]** 在非退化二维状态下，C1–C4 可以转成速度平面的几何约束，C5 可以转成约束优化问题；没有给出闭式求解代码本身不代表数学不可实现。但论文没有给出涵盖切线边界、对称平局、重合／已在避让圆内以及空可行集的完整确定性规则，因此不能据此宣称所有情况都有唯一答案。

**[论文明确描述]** 更新后重查**全部邻居** D1–D4，直到没有危险邻居。若多轮后候选速度小于 \(v^{SPP}\)，且

$$
(\mathbf x_i^{target}-\mathbf r_i)\cdot\mathbf v_i^{(s)}<0,
$$

则尝试以**当前已降低的速度模长**重新指向目标；只有不触发 D1–D4 才采用这一恢复方向。临近目标最后使用 \(D(d_{target},0,p^{avoid},a^{avoid})\) 平滑减速。

**[尚未验证]** 最后一步是对候选速度模长取 min、如何与避让后方向组合、减速后是否重新检查危险邻居，正文未逐项写明。自然的限幅实现必须标为工程补全，尤其因为改变本机速度会改变相对运动，不能无条件认为减速永远保留原来的避让可行性。

### 4.6 终止性与确定性缺口

| 位置 | 已说明 | 尚缺的确定规则／影响 |
|---|---|---|
| §3.4 p. 36–38 | 定义 \(t_{\mathrm{plan}}^{(s)}\) 并给初值 | 未给 \(t_{\mathrm{plan}}^{(s+1)}\) 更新；固定、重算、逐轮缩短会改变 D4 与邻居集合 |
| D1 / C1 | 碰撞锥与切线 | \(d=0\)、\(d\le R^{avoid}\) 时切线退化；零相对速度夹角未定义 |
| D4 | 距离差除以相对速率 | r/R 符号、缺少 s、负时间、零速除法；是否用该式直接排序未形式化 |
| C2 | 保持绕行侧 | 旧相对速度恰在线上时两侧等价；没有固定左右或 ID 平局规则 |
| C1 / C5 | 脱离禁区且最大化点积 | 切线边界是否可取；若严格排除边界，最优值可能只有上确界。D1 使用严格小于提示切线可安全，但仍需确认实现约定 |
| C5 | 局部最大化 | 多解平局、空可行集和数值容差未规定 |
| 迭代 | 无危险邻居即停 | 未规定迭代上限、最小改善阈值、循环检测和失败回退；C3 的速度非增不能单独证明有限步终止 |
| 末端处理 | 目标附近制动 | 到达判定半径／速度阈值、到达与换目标的先后、停车后方向和时域初值未规定 |
| 所有分项 | 式 (1) 连续叠加 | 合成限幅与底层追踪／采样的确切顺序未完整给出；Self-drive 自身无威胁不代表合成速度无威胁 |

**审计判断 [工程推断]：可以制定一套有出处、有补全记录的参考实现规范；不能把未补全的自然语言规则直接视为完全确定、可逐位复现作者结果的算法。** 论文 §6 p. 48 认为位置噪声有助于正面冲突的绕行选侧，这并不替代无噪声测试中需要的明确平局规则。

### 4.7 Queueing：§3.5 pp. 38–39

**[论文明确描述]** A 排在 B 后面，当

$$
\|\mathbf x_A^{target}-\mathbf x_B^{target}\|<R^{avoid},
\qquad
\|\mathbf x_B^{target}-\mathbf r_B\|
<
\|\mathbf x_A^{target}-\mathbf r_A\|.
$$

排队意味着在距目标比 B 稍远的位置停下，是二维自组织队列，不是固定路线上的一维 FIFO 队列。输入额外需要邻居目的地和邻居到其目的地的距离；即使目标不完全重合，只要落入上述相近区域也会触发。

**[尚未验证]** 本文将进一步细节指向 2018 年工作，没有给“稍远”的停车距离公式、多个先到邻居如何选、同距离平局、排队进入／退出条件、队列状态保持，以及它如何修改 Self-drive／目标减速。S2 另列 \(R^{queue}=30\) m，却没有解释如何用于正文以上 \(R^{avoid}\) 判据；不能把两个参数直接互换。

**[工程推断]** 普通避碰解决路径冲突；Queueing 限制相近目的地的并发接近，缓解末端挤压，二者需要共同工作。但本文没有显式资源容量 \(c\)、服务时间、占用／释放事件或公平性保证；不能把 Queueing 等同为已解决“起降点容量约束”。未来共享资源研究应在基线复现后另建资源模型，单独评价等待、服务率和公平性。

### 4.8 可核对的执行流程

以下是论文规则的流程整理，问号表示本文不足以确定的部分，不是已经运行的伪实现：

~~~text
输入：自身状态、目标；局部邻居状态和目标；参数
  1. 按目标方向、邻居速度分类计算各向异性 Repulsion
  2. 通过三个方向条件选摩擦邻居，按式 (4)–(6) 计算 Friction
     [式 (4) 幅值解释待定]
  3. 初始化 Self-drive 候选和规划时域
  4. 检查全部邻居 D1–D4
       无威胁 → 接受
       有威胁 → 选最紧迫 k → C1–C5 求下一候选 → 返回第 4 步
       [时域更新、退化处理、迭代失败回退待定]
  5. 满足条件时尝试恢复目标方向
  6. 临近目标按 D 制动
  7. Queueing 修改末端接近／停车行为 [具体插入顺序和距离公式待定]
  8. 按式 (1) 合成期望速度，交给有限加速度底层模型
输出：期望速度；可能的排队／到达状态 [状态机定义待定]
~~~

## 5. 论文公式与代码的对应关系

| 论文位置 | 开源对应 | 一致程度与审计判定 |
|---|---|---|
| 式 (1)，p. 34 | algo_spp_evol.c，CalculatePreferredVelocity:482–499 | 只在“速度项相加”形式相似；代码加入 shill，并默认归一化成 V_Flock，不能当成同一个控制器 |
| 式 (2)，p. 34 | interactions.c，RepulsionLin:73；math_utils.c，SigmoidLin:596 | 有 \(p(R_0-d)\) 的局部线性形式，但代码有 V_Rep 饱和、方向各向同性 |
| 式 (3)，p. 35 | 无 | 当前排斥函数没有目标向量、速度分类、A 输入 |
| 式 (4)，p. 35 | interactions.c，FrictionLinSqrt:16 | 代码有 C_Frict × 超限幅值，论文印刷式没有；代码也没有交通邻居筛选 |
| 式 (5)，p. 35 | FrictionLinSqrt:60–68 | max(V_Frict, braking curve) 对应；传入距离偏移为 R_0+R_0_Offset_Frict，参数意义不能逐名直接照搬 |
| 式 (6)，p. 35 | math_utils.c，VelDecayLinSqrt:608 | 分段形状、连接点一致；代码还取 v_max 饱和。用于摩擦时 v_max=VelDiff，超限修正与未饱和曲线的行为可对照验证 |
| D1–D4 / C1–C5，pp. 37–38 | 无 | 当前 SPP 只维持当前运动方向；不具备本文主要创新 |
| 目标附近制动，p. 38 | 只有通用 VelDecayLinSqrt 工具 | 没有个体目标与相应调用链 |
| Queueing，p. 39 | 无 | 无目标广播状态、相近目标判断和停车规则 |
| 式 (7)，p. 39 | stat.c，RatioOfDangerousSituations:363–396；robotsim_main.c:2092–2118 | 瞬时配对比例及时间积分结构对应；代码阈值为 ≤，论文以 Heaviside 表示，边界取值需注明 |
| 式 (8)/(9)，p. 40 | 无直接实现 | 旧统计有速度模长、相关性、簇等，没有上一／下一目标投影和到达吞吐量 |
| §4.2 优先级，pp. 43–44 | 无 | 没有成对 hierarchy 状态和优先级条件分支 |
| §4.3 分层，pp. 44–46 | 无对应交通逻辑 | 框架能存三维坐标不代表已有按航向选层、基准层到达和垂直切换状态机 |

**[开源代码证实]** 可直接借鉴的最小集合是：状态／参数容器、速度松弛及积分、有限加速度饱和、通信筛选和延迟缓存、误差模型、D 的分段函数、配对风险统计。所有这些仍应与本文的实际参数、单位和更新顺序逐一核对。

## 6. 实验设置、参数来源与预期结果

### 6.1 场景与目标生成

**[论文明确描述]** §4 p. 39 和补充 Text S1 pp. 6–7：

- 仿真在边长 L 的方形区域中进行二维交通，各机独立获得边界目标。
- 到达当前目标后，从另外三条边等概率选一条，再在该边均匀选点；若新旧目标距离小于 L/3，重复整个抽样过程。
- 该规则不是在整个边界上无条件均匀重新采样；同边目的地被排除，短程路径被拒绝。
- 平均直达路径长度 \(\langle l\rangle=0.947L\)；这是补充 Text S1 基于模拟测量给的近似常数，对应 \(\Phi\simeq1.056\,v^{eff}N/L\)，不是任意目标生成机制的通用几何定理。
- 文中 MFP 定义采用 \(L/\sqrt N\) 的简化空间尺度，密度为 \(\rho=N/L^2=1/\mathrm{MFP}^2\)。不要把这里的 MFP 与动力学碰撞理论的平均自由程混为同一量。

**[尚未验证]** 正文和补充材料没有给出本文初始位置／初始速度分布、第一段的 previous target 定义、到达容差、边界是否有额外 shill 力，以及所有密度测量点的机器可读列表。旧 SPP 的随机布置与墙力不能直接补成本文设置。

### 6.2 官方 Table S1：环境参数

**[论文明确描述]** 下表逐项取自补充 p. 3；最后一列仅是旧仓库对照。

| 参数 | S1 数值（SI） | 旧仓库 parameters/unitparams.dat |
|---|---|---|
| 通信延迟 \(t^{del}\) | 0.5 s | t_del=1.0 s；与 S1 不同 |
| 速度松弛时间 \(\tau^{CTRL}\) | 1 s | tau_PID_XY=1，tau_PID_Z=1 |
| 最大确定性加速度 \(a^{max}\) | 6 m/s² | a_max=600 cm/s² |
| 通信范围 \(r^c\) | 80 m | R_C=10000 cm=100 m；与 S1 不同 |
| 传感器更新间隔 \(t^s\) | 0.2 s（5 Hz；表称 update rate，但单位是秒） | t_GPS=0.2 |
| 传感器噪声 \(\sigma^s\) | 0.005 m²/s² | Sigma_GPS_XY=50 cm²/s²=0.005 m²/s²，Z=0 |
| 外部噪声 \(\sigma\) | 0.2 m²/s³ | Sigma_outer_XY=2000 cm²/s³=0.2 m²/s³，Z=0 |

**[尚未验证]** 这些噪声参数不是“位置标准差 0.005 m”或“每步速度标准差 0.2 m/s”。S1 没有列出完整随机过程、XY/Z 分配、丢包率、风、速度硬上限和积分步长。旧 initparams.dat 的 DeltaT=0.01 s 是代码默认值，**不是已确认的本文积分步长**。旧配置 Length=600、NumberOfAgents=100 与部分论文实验相符，也不能因此假定整份配置就是论文配置。

**[论文明确描述：相互不一致]** 正文 p. 41 用 \(t_{delay}=1\) s、\(v^{SPP}=8\) m/s、\(a_{\max}=6\) m/s² 解释两机迎面停车距离约 26.6 m；S1 的通信延迟却是 0.5 s。是否正文另计了控制反应时间，本文没有建立清楚的参数对应，需分别保存。

### 6.3 官方 Table S2：算法参数

**[论文明确描述]** 补充 p. 3 的原表如下；表中 friction 的 vfrictmax 行描述的是小距离允许速度差，对应式 (5) 的常数 \(v^{friction}\)，并非整个距离函数只有一个常数。

| 参数 | 仿真 | 实机 | 单位／用途 |
|---|---:|---:|---|
| \(R_0^{rep}\) | 10.5 | 9 | m，排斥区半径 |
| \(p^{rep}\) | 1.3 | 1.1 | s⁻¹，排斥增益 |
| \(A^{rep}\) | 0.5 | 0.5 | 无量纲；正文仿真写 0.42，冲突未消解 |
| \(R^{friction}\) | 10.7 | 10.7 | m，强摩擦区距离偏移 |
| 表中 \(v^{frictmax}\)／式 (5) 的 \(v^{friction}\) | 0.5 | 0.8 | m/s，小距离速度差容限 |
| \(p^{friction}\) | 1 | 1 | s⁻¹ |
| \(a^{friction}\) | 3 | 3 | m/s² |
| \(v^{SPP}\) | 8 | 6 | m/s，自由运动期望速度 |
| \(R^{avoid}\) | 13 | 10.5 | m，预测避让范围 |
| \(p^{avoid}\) | 0.5 | 0.5 | s⁻¹ |
| \(a^{avoid}\) | 3 | 3 | m/s² |
| \(R^{queue}\) | 30 | 30 | m；正文未定义其与排队停车距离的关系 |

**[开源代码证实]** 旧 flockingparams.dat 例如 V_Flock=4 m/s、R_0=25.6668 m、Slope_Rep=0.4 s⁻¹、C_Frict=0.023802、ArenaRadius=230 m；都不能作为 S2 替代。原 C 使用 cm、cm/s、cm/s²，算法参数表的 .Mult=0.01 主要服务显示，不表示内部数值自动全部转成 SI。平方长度量的换算因子为 \(10^{-4}\)，不要一律乘 0.01。

### 6.4 三个评价指标

**Collision risk [论文明确描述]**，式 (7)，p. 39：

$$
\psi^{coll}=
\frac1T\int_0^T
\frac1{N(N-1)}
\sum_{i=1}^N\sum_{j\ne i}
H(r^{coll}-d_{ij}(t))\,dt.
$$

每一时刻统计危险配对占全体配对的比例，再做时间平均；等价于无序危险对数除以 \(\binom N2\) 后平均。Fig. 7 p. 47 明确说明仿真使用 \(r^{coll}=3\) m。它是危险距离占用率，既不是物理碰撞概率，也不是一次连续接近算一次的事件计数。

**[开源代码证实]** RatioOfDangerousSituations 与此瞬时比例一致；HowManyCollisions（dynamics_utils.c:1376）使用 AgentsInDanger 的状态变化，属于另一种事件近似，不能拿它代替式 (7) 的积分。

**[工程推断]** p. 41 将 \(\psi^{coll}=10^{-6}\)、N=100、T=600 s 解释为约 6 次 occurrences；严格按式 (7)，得到的是 \(10^{-6}\times100\times99\times600=5.94\) 个“有序危险对·秒”（无序对为 2.97 对·秒）。没有事件持续时间和计数规则，不能把它严格换成 6 次独立碰撞事件。

**Effective velocity [论文明确描述]**，式 (8) 及其后文字，p. 40：

$$
\mathbf e_i=
\frac{\mathbf x_i^{next}-\mathbf x_i^{prev}}
{\|\mathbf x_i^{next}-\mathbf x_i^{prev}\|},\qquad
v_i^{eff}(t)=
\mathbf v_i(t)\cdot\mathbf e_i\,
\operatorname{sgn}
\left[(\mathbf x_i^{next}-\mathbf r_i(t))
\cdot(\mathbf x_i^{next}-\mathbf x_i^{prev})\right].
$$

公式本体只写投影，文字要求加上述符号修正。最后对 agent 和时间平均得到 \(v^{eff}\)。投影基于上一目标到下一目标的直线，不能换成当前位置到目标的径向速度，也不能使用速度模长。越过目标、第一航段、正好目标面上的 sign(0) 需定义。

**Throughput [论文明确描述]**，式 (9)，p. 40：

$$
\Phi^{through}=\frac{Nv^{eff}}{\langle l\rangle},
\qquad
\langle l\rangle=
\begin{cases}
0.947L & \text{本文方形目标抽样},\\
4R/\pi & \text{圆周均匀目标抽样}.
\end{cases}
$$

单位 s⁻¹，解释为单位时间到达目标的数量。补充 Text S1 同时提到直接测量到达数／时间和用有效速度校准几何常数。

**[工程推断]** 后续应并行报告直接计数吞吐量和式 (9) 推导值，以暴露初始过渡、目标停留、排队、换目标和分层垂直航段的统计差异；不要先强制让两者数值相等。

### 6.5 Fig. 2：密度扫描

**[论文明确描述]** pp. 40–41、Fig. 2：

| 项目 | 设定／预期 |
|---|---|
| agent | N=100，同质 \(v^{SPP}=8\) m/s |
| 场景 | 方形边界目标，不同 L 对应不同密度 |
| 两组 | (a) 无 agent-agent 相互作用的 null model；(b) 优化后的完整交通算法 |
| 采样 | 每个测量点 100 次，每次 10 min；图示误差线为标准差 |
| 最优点 | MFP=27.5 m，L=275 m，\(\rho=100/275^2\approx13.22\times10^{-4}\) agent/m² |
| 峰值吞吐量 | 交通算法约 1.961 s⁻¹；此前 2018 年结果为 0.96 s⁻¹ |
| 风险改善 | 在该密度相对 null model 约降为 1/2500，作者称超过三个数量级 |
| 曲线趋势 | 稀疏时有效速度接近期望速度、风险趋低；更密时有效速度下降、风险升高，吞吐量在中间密度达到峰值 |

**[工程推断]** 用作者最优点代入式 (9)，\(v^{eff}\approx1.961\times0.947\times275/100=5.107\) m/s；这是由文中数值推导的校验点，不是本轮测量值。

**[尚未验证]** null model 是否仍启用 Queueing、场地边界力和全部同样的噪声／底层控制；全部扫描 L 列表、每次初始条件、种子、是否剔除暂态、优化目标函数与搜索范围均未完整公开。100 次重复不是 100 个 agent 的含义。

### 6.6 Fig. 3：规模扩展

**[论文明确描述]** §4.1 pp. 41–42：

- 固定 MFP=27.5 m 和同质期望速度，增加 agent 数到 5000；保持密度时应令 \(L=27.5\sqrt N\) m。
- 每点 100 次 10 min 仿真。图实际绘制有效速度与 collision risk；有效速度约维持在 4.5–4.7 m/s（仅目视估读，未数字化），配对风险随 N 增大而下降。
- 作者同时说明从 N=100 到 5000 时，按其 collision 定义累计的危险量约增加 270 倍，因此风险比例下降不等于危险总量下降。
- Supplementary Movie S1 展示的是 5、50、500、5000 机；这不是正文列出的 Fig. 3 全部测量点列表。

**[工程推断：图注问题]** Fig. 3 图注称固定密度下“相同有效速度，因此相同吞吐量”，与式 (9) 不相容：

$$
\Phi=\frac{Nv^{eff}}{0.947L}
=\frac{v^{eff}}{0.947\times27.5}\sqrt N.
$$

若使用正文定义，总到达吞吐量应近似随 \(\sqrt N\) 增长，而不是保持常数。除非另有未说明的归一化方式，否则不能把图注当作总吞吐量验收目标。Fig. 3 也没有画 throughput 曲线。

**[尚未验证]** 固定 MFP 下 Fig. 2 最优点推导速度约 5.1 m/s，而 Fig. 3 的曲线约 4.6 m/s，是否来自参数／统计差异，现有资料不足以解释；不应要求一套未经确认的配置同时精确命中两图。

### 6.7 其他实验与实机验证

| 实验 | 论文明确描述 | 保留的问题 |
|---|---|---|
| Fig. S1，补充 p. 4 | N=100，\(v^{SPP}=16\) m/s，每点 100 次 10 min；峰值吞吐量更高，但峰值出现在更低密度 | 未提供高速情形是否改通信范围等完整配置 |
| Fig. S2，补充 p. 5 | N=30，agent 0→29 的期望速度从 32→2 m/s 线性变化；100 次 10 min；快机有效速度被拥堵压至约 12 m/s | 图注同时写 L=27.5 m 和 MFP=50 m，二者与 N=30 不相容 |
| Fig. 4，pp. 43–44 | 等权与完全无环优先级 FullCF 比较；低编号优先，期望速度 8 m/s，100 次重复；boss 效率高、下级效率低 | 该图专用 N、L、完整运行时长未在图注全部列明 |
| 优先级机制，§4.2.2 | 上级对下级缩小 Self-drive 避让距离至约 5 m，停止摩擦，保留排斥；只有下级速度朝向上级时才允许此放宽 | 这些约值不取代 S2 基础参数；没有代码实现 |
| Fig. 5，pp. 45–46 | N=100、8 m/s，L=150–600 m，比较 1/3/5 层，每密度 100 次；按航向分扇区选层，先垂直换层、再水平交通、再回基准层完成目标 | 不能把增加层数当成末端容量同步增加 |
| 分层参数，p. 46 | 层间距 10 m，overlap=0.5，垂直距离小于两者乘积时继续相互作用；目标均在基准层 | 文中把层距单位印为“10 ms”，按语境是距离单位笔误，需保留注明；仿真竖直速度未完整给出 |

**[工程推断]** Fig. S2 的明确矛盾：若 L=27.5 m，则 MFP=27.5/√30≈5.02 m；若 MFP=50 m，则 L≈273.86 m。“可能漏了一个 0”只是猜测，不能替作者把 L 改成 275 m 后当作已确认配置。补充 Text S1 还把用于几何常数校准的 7000 次仿真称为主文 Fig. 1，而主文 Fig. 1 实为算法示意图，引用编号也有疑点。

**[论文明确描述]** §5 pp. 46–48 的实机证据与仿真须分开：

- 100 架真实多旋翼参加，97 架飞行日志可取得；Fig. 7 分析 400 s 稳态片段。
- Odroid C1+ 运行算法，向 PixHawk1／定制 ArduCopter 发送速度指令 20 Hz；U-blox M8P 支持 RTK。
- ad-hoc IBSS WiFi 以 10 Hz 广播带时间戳、ID、位置、速度、目标和状态的 UDP 包，地面站提供 RTK 修正；时间戳用于线性外推邻居位置。
- 圆形边界目标，半径 125 m；两层相隔 14 m，水平期望速度 6 m/s、竖直速度 1.5 m/s。
- Fig. 5 图注的匹配仿真采用 97 机、6 m/s、两层，\(v^{eff}=3.52\pm0.02\) m/s，实机为 3.57 m/s；不能拿默认 100 机／8 m/s／奇数层仿真直接比较现实差距。
- 作者报告飞行无物理碰撞，但 3 m 风险距离偶有违反，最小间距未低于 1.5 m。因此“无碰撞”不等于 \(\psi^{coll}=0\)。

**[尚未验证]** 本轮没有下载、解析或重算真实飞行日志，没有验证视频、硬件或作者“无碰撞”主张。20 Hz 是速度指令频率，10 Hz 是状态广播，S1 的 5 Hz 是仿真传感器周期，三者不可混用。

## 7. 尚待确认的公式、参数和实现细节

以下问题应作为复现记录和向作者求证的清单；本轮未向作者发送消息。

| 优先级 | 未决项 | 影响与建议处理 |
|---|---|---|
| 阻断严格等效 | 式 (4) 缺速度幅值／增益，且量纲不闭合 | 先取得作者解释／实现；如无法取得，显式建立候选解释分支，不把旧摩擦函数认作答案 |
| 阻断严格等效 | Self-drive 的 \(t_{\mathrm{plan}}^{(s)}\) 更新缺失 | 固定、重估或收缩会改变预测；必须冻结选择并标为工程补全 |
| 阻断严格等效 | C1–C5 的退化、平局、边界、不可行与迭代失败 | 在实现前定义确定性行为、数值容差和日志；增加最大迭代数只是工程保护，不是论文规则 |
| 阻断 Queueing 等效 | \(R^{queue}=30\) 的用途及停车距离不明 | 先追溯作者引用的 2018 年 Queueing 说明，仍不足则求证；不能直接添加服务容量机制冒充原算法 |
| 高 | A：正文 0.42，S2 0.5 | 保留两个参数分支，尚不能判断哪一个生成主文各图 |
| 高 | 延迟：S1 0.5 s，正文停车示例 1 s | 确认是否为不同层次延迟；不要无说明地把两者相加 |
| 高 | 相对位置方向和 Friction 第一角条件 | 用明确向量定义审查；同时核对原文／2018 图示，不凭常识静默修正 |
| 高 | Fig. 3 吞吐量图注与式 (9) 冲突 | 使用明确定义的总吞吐量和必要的归一化量分别报告 |
| 高 | Fig. S2：场地尺寸与 MFP 冲突 | 不以任何一个数作为唯一已确认复现实验配置 |
| 高 | 积分步长、仿真控制更新、初态、到达／换目标容差、暂态截断 | 旧框架默认只能作为候选参考，参数来源需逐项标注 |
| 中 | 传感器噪声精确定义、XY/Z 分配、丢包／风、历史／外推策略 | 先按旧框架单独验证，再说明与论文未建立等效的部分 |
| 中 | 速度合成限幅、制动后复查、Queueing 插入顺序 | 会改变碰撞风险与慢机行为；不得沿用 SPP 的定速归一化 |
| 中 | 密度扫描点、各图精确 N 列表、进化优化目标／搜索配置、原始曲线数据、种子 | 无法做精确曲线／参数优化复现；可先做定性趋势与关键公开点验证 |

**[工程推断]** 这些问题影响“严格复现”的程度，不等于论文方法无效。应分别区分：规则级参考实现、论文数值趋势复现、作者实验逐项等效三种目标。

## 8. 技术路线比较

| 评价维度 | A：修改原 C RobotSim | B：Python + NumPy 轻量参考实现 |
|---|---|---|
| 论文一致性可验证程度 | 保留现有动力学／噪声时有优势，但公开代码并非论文 traffic 版本，关键算法仍需新增和解释 | 每条公式、输入、约束和补全决策容易暴露并核查；必须另证动力学、采样、延迟、噪声的等效性 |
| 实现工作量 | 省去部分底层模型；仍须加目标系统、选择性 Friction、各向异性 Repulsion、Self-drive、Queueing、指标和批量配置 | 需实现小型状态／积分／观测框架，但可以围绕论文接口组织；不要求 GUI 或复杂物理引擎 |
| 当前环境成本 | 缺 GL 开发依赖；无界面模式也仍依赖其构建；可能存在更后续的旧 C 兼容问题，尚未验证 | 目标 Python 环境存在但 NumPy 尚未安装；后续按需安装小范围数值依赖即可，本轮未安装 |
| 调试难度 | 全局变量、静态临时缓存和 GUI／动力学耦合较多，几何候选和状态追踪更费力 | 容易输出中间向量、可行域、危险邻居及逐步轨迹，适合先定位公式解释差异 |
| 大规模性能 | 编译型循环有优势；但当前全局复制、扫描和配对统计保留 \(O(N^2)\) 成本 | 小规模／中等规模适合验证；逐机迭代若用纯 Python 循环会慢，不能承诺 5000 机实时 |
| 内存与扩展 | 当前 PhaseData 历史存储与全局状态布局需核对，扩展目标／资源状态有接口侵入 | 可独立设计快照、延迟缓冲、日志和邻居索引；必须避免生成整段 T×N×N 轨迹矩阵 |
| 城市／航路／共享容量 | 有 2D 障碍多边形，但缺城市拓扑、资源服务和拥堵状态；仍需新建模型 | 便于将开放空间交通、几何环境和资源占用分层组织，适合后续研究变量与消融 |
| 风险 | 误把“作者原框架”当成“作者原算法”；移植时沿用定速 SPP 改变结果 | 过度简化噪声／控制、过早优化、用任意求解器或 ORCA 替换 C1–C5，均会偏离原文 |

**推荐 [工程推断]：优先 B，保留 A 的只读对照价值。** 本项目最先需要解决的是规则是否可核查和参数来源是否清楚，C 语言并不能消除公开算法缺口。Python 能较低成本暴露这些问题，也适合未来显式资源容量与服务过程的研究。

性能策略应后置：先批量化简单向量计算，再基于测量选择均匀网格／空间哈希、稀疏配对统计；确有瓶颈后再考虑编译内核。NumPy 全 N×N 广播并不自动具备良好大规模性能，C 旧版也没有自动获得局部邻居数量级复杂度。

## 9. 推荐的下一阶段开发顺序与验收

本节只是后续工作建议，本轮未创建这些模块。

1. **先冻结复现规范和参数来源表。** 明确向量符号、单位、式 (4) 解释、时域更新、几何边界、平局、迭代保护；把 S1/S2、正文冲突值和工程自定值分开记录。对阻断项优先寻找作者专用代码或解释。未解决时只称“有补全记录的参考实现”。
2. **先实现几何与制动纯函数。** 包括向量安全归一化、式 (6)、切线／碰撞锥、D1–D4、紧迫邻居选择和 C1–C5 候选求解。用迎面、交叉、追越、远离、静止、完全对称、圆内／重合、同时间多威胁及空可行集案例验证；记录每次候选是否满足约束、何时终止。这些测试验证真实数学边界，不是复制实现逻辑。
3. **实现最小状态、目标生成和指标模块。** 维护真实／观测状态、上一／下一目标和可重复随机流；实现三边抽样及 L/3 拒绝、危险配对时间比例、带符号目标投影、到达事件计数与式 (9)。先无相互作用核对目标分布和指标定义。
4. **实现与 C 对照的动力学／观测模块。** 速度松弛、旧速度位置积分、确定性加速度限幅、传感器采样、通信范围、延迟缓冲和两类噪声逐层开启；先比较无噪声单机响应，再对随机过程做统计核对。特别保留“限幅后叠加外噪声”的顺序。
5. **合成 Repulsion、选择性 Friction、Self-drive；再接 Queueing。** 先做两机／少量多机轨迹和分项日志，避免把末端停车问题混入最初几何求解。若 Queueing 仍无完整规范，明确标注该阶段为不完整基线，不以关闭它的结果声称完整复现。
6. **优先验证 Fig. 2 的公开关键点，再做密度扫描。** N=100、8 m/s、L=275 m；A 和延迟的冲突值分别运行，先做少量调试，规则和配置冻结后再按 100×600 s 协议运行。报告风险、有效速度、直接／推导吞吐量及重复方差；不能只以曲线形状相似验收。
7. **再验证 Fig. 3、异质性和分层。** 固定 MFP 扩 N 时分开报告配对风险和危险总量、总吞吐量与归一化量；性能和数值行为均做测量。
8. **基线证据充分后再扩展城市与共享资源容量。** 再引入建筑几何、航路／交叉口／起降资源、容量、服务时间、占用释放及拥堵评价，并与原始开放空间／Queueing 基线分别比较。

下一阶段最先动手的具体组件应是 **D 制动函数、二维碰撞锥／D1–D4、C1–C5 候选求解与迭代追踪**，同时建立 **单位／参数来源和规则补全登记**。不应先搭建城市画面，也不应先包装原 SPP 可执行文件为本文基线。

## 10. 本轮验证记录与可重查入口

**实际完成：** 本地清单和完整性检查；58 个跟踪文件分类与关键源码调用链阅读；正文 25 页及用户手册 7 页提取；官方补充 7 页获取和解析；式 (3)–(6)、Fig. 1–3 和 S2 表／图的页面目视核对；只读远端版本核对；工具链与语法编译探测；本报告生成。未修改 references/robotsim。

可重查的只读命令：

~~~bash
git -C references/robotsim rev-parse HEAD
git -C references/robotsim status --short
git -C references/robotsim ls-tree -r --name-only HEAD
git -C references/robotsim log --all --oneline
git -C references/robotsim fsck --full
git ls-remote https://github.com/csviragh/robotsim.git HEAD 'refs/heads/*' 'refs/tags/*'
make -C references/robotsim -n spp_evol
gcc -fsyntax-only references/robotsim/src/robotsim_main.c
conda env list
/home/gag47/miniconda3/envs/drone-traffic/bin/python --version
sha256sum 'papers/Decentralized traffic management of autonomous drones.pdf'
~~~

最后一条 GCC 检查预期目前因 GL/gl.h 缺失而失败；它不是论文仿真验收。外部 GitHub API 分支／标签请求曾遇到 403 限流，随后已用 git ls-remote 完成版本核对。全文未声称实际复现成功。

主要资料：

- [论文正式版 DOI／期刊页面](https://doi.org/10.1007/s11721-024-00241-y)：本地全文为主要算法与实验依据，特别是 §2–5、pp. 32–48。
- [官方补充材料 PDF](https://media.springernature.com/original/springer-static/esm/art%3A10.1007%2Fs11721-024-00241-y/MediaObjects/11721_2024_241_MOESM1_ESM.pdf)：S1/S2 参数见 p. 3，Fig. S1/S2 见 pp. 4–5，Text S1 见 pp. 6–7。
- [作者项目页](https://hal.elte.hu/drones/swarmint2024.html)：代码基础、补充材料及飞行可视化入口；网页陈述不替代代码审计。
- [固定提交源码](https://github.com/csviragh/robotsim/tree/ea54b97e7e2659449e721ebd4a2285beb9b0e9ce)：对应本地 references/robotsim；特别是 [algo_spp_evol.c](https://github.com/csviragh/robotsim/blob/ea54b97e7e2659449e721ebd4a2285beb9b0e9ce/src/algo_spp_evol.c#L417)、[robotmodel.c](https://github.com/csviragh/robotsim/blob/ea54b97e7e2659449e721ebd4a2285beb9b0e9ce/src/robotmodel.c#L243)、[interactions.c](https://github.com/csviragh/robotsim/blob/ea54b97e7e2659449e721ebd4a2285beb9b0e9ce/src/utilities/interactions.c#L16)、[math_utils.c](https://github.com/csviragh/robotsim/blob/ea54b97e7e2659449e721ebd4a2285beb9b0e9ce/src/utilities/math_utils.c#L608)、[stat.c](https://github.com/csviragh/robotsim/blob/ea54b97e7e2659449e721ebd4a2285beb9b0e9ce/src/stat.c#L363)。
