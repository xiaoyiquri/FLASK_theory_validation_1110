# FLASK-RRTConnect 代码大纲与参数修改指南

本文对应2026-10-19修复版的version-2测量实现。下文路径相对工程根目录
`FLASK_theory_validation`。这是独立DF + RRTConnect工程；DynoBench文件用于
输入场景、模型对照和绘图，不是在官方DynoBench求解器内新增的插件。

## 1. 运行主线

```text
examples/dynobench_*.yaml：指定场景、规划模型、参数
  → main.cpp：读取配置，覆盖seed/预算/后端
  → io.cpp：读取DynoBench场景与模型，将物理起终点lift到flat状态
  → run_*.cpp / io.h：实例化对应机器人、n、r
  → planner.h：两棵树交替扩展、Connect、提取首解
       → bvp.h：正时间LQMT边界连接
       → model.h：平坦映射、物理限制、奇异性
       → validation.h：边/整条路径验证
            → batch*.cpp：同一轨迹8个时刻的SIMD求值/物理检查
            → geometry*.cpp：原始矩形/球本体与盒/球障碍碰撞
  → planner.h：一次Algorithm 4 shortcut，记录每次cost下降事件
  → diagnostics.h / result_io.h / io.h：路径、指标、误差、快照输出
  → benchmark.py：多场景、多seed、固定预算记录
  → summarize_runs.py / plot_six.py：统计与图
```

搜索与shortcut共用预算。标准流程找到首解并完成一次shortcut后即可停止；
并不会为了填满10秒自动重启或持续优化。事件后没有新路径时，预算查询保持
最后已知路径。该流程不自动具有渐近最优性。

## 2. C++生产代码

### include/flask：模板实现与公共接口

| 文件 | 实现内容 | 要改变什么时查看 |
| --- | --- | --- |
| `bvp.h` | 完整flat端点的固定时间/最优时间LQMT连接；r2三次多项式；正时间驻点筛选；代价计算 | 边连接和目标函数数学实现 |
| `planner.h` | `PlannerConfig`、`Plan`、不可变incumbent历史；归一化flat距离；线性扫描最近邻；正向/目标树有向扩展；Connect、回溯；Algorithm 4 shortcut；首解/预算时钟 | 树扩展、NN、shortcut、事件记录 |
| `model.h` | 机器人类型和`ModelConfig`；flat节点/边界检查；由曲线导数恢复物理状态与控制；模型限制；Unicycle r2区间速度极值检查 | 动力学检查、分支与奇异性处理 |
| `validation.h` | `ValidationConfig`、统计计数；有限时间网格边验证；整条路径与端点/拼接检查；scalar/SIMD选择；可选连续球几何检查 | 验证策略、失败分类、物理连续性契约 |
| `batch.h` | `PackedCurve`、`BatchContext`、`BatchResult`；导数系数打包；8个时刻的SoA数据接口 | SIMD数据布局/接口 |
| `geometry.h` | 原始球/盒障碍、球/二维矩形机器人、8点姿态批次、碰撞与AVX2接口 | 几何数据结构 |
| `diagnostics.h` | flat位置/速度/加速度/jerk跳变，物理姿态/角速度跳变，完整物理起终点误差 | reduced-r2诊断，不是额外优化 |
| `result_io.h` | YAML辅助、路径多项式与物理样本、长度/诊断、DynoBench格式states/actions/times导出 | 单条路径结果格式 |
| `io.h` | `Scene`、`runScene<N,R>`；组装模型/BVP/验证器/规划器；组织首解、最终与中间文件、汇总与配置快照 | 一次完整运行和结果组织 |
| `profile.h` | 可选RAII分阶段计时`ProfileScope` | NN/BVP/物理/碰撞耗时诊断 |

### src：入口、配置解析和编译后端

| 文件 | 实现内容 |
| --- | --- |
| `main.cpp` | CLI参数检查、配置加载、seed/预算/backend覆盖、机器人分发、异常退出 |
| `io.cpp` | 场景/模型文件解析、相对路径解析、物理参数读取、端点lift、环境范围合并、原始机器人外形与盒障碍物加载、YAML选项装配 |
| `run_integrator.cpp` | 积分链CLI实例化：`(n,r)=(1,1),(2,2),(7,2)` |
| `run_unicycle.cpp` | Unicycle CLI实例化：`n=2, r=2/3` |
| `run_quad2.cpp` | Quad2D CLI实例化：`n=2, r=2/4/5` |
| `run_quad3.cpp` | Quad3D CLI实例化：`n=4, r=2/4/5` |
| `batch.cpp` | 批次模型参数/执行器映射与原始几何、参数有效性、AVX2批次调用入口 |
| `batch_avx2.cpp` | 同一条曲线8个时刻的double SIMD导数求值、物理映射、约束掩码和碰撞；AVX2每向量4个double，分组处理8点 |
| `geometry.cpp` | CPU AVX2能力检测、矩形SAT/球盒/球球碰撞、后端分派 |
| `geometry_avx2.cpp` | AVX2原始几何碰撞内核 |

`CMakeLists.txt`控制Eigen/yaml-cpp、AVX2独立编译单元、统一Eigen对齐ABI、
可执行文件和测试。AVX2不表示同时搜索8棵树或同时求8条BVP。

## 3. Python工具

| 文件 | 职责 | 常改入口 |
| --- | --- | --- |
| `tools/configure_six.py` | 重置六份根场景配置到当前显式r2/R=I/rho=1/shortcut profile | 批量配置生成规则；会覆盖手动参数 |
| `tools/benchmark.py` | 启动六场景×seed×repeat；进程限时；保存输入/源码/二进制哈希；收集原始首解/最终/事件/预算表；可选native checker | `--seeds`、`--budget`、`--backend`、`--checkpoints`、`--scenarios`、`--repeats` |
| `tools/experiment_io.py` | 共享YAML加载、源文件快照、模型差异、历史合法性与预算时刻前驱事件查询 | 测量语义，默认时刻常量`CHECKPOINTS` |
| `tools/summarize_runs.py` | 检查记录完整性；按首解/最终/预算统计成功率、条件均值、样本SD、诊断及拒绝分类 | 统计/表格组织 |
| `tools/plot_six.py` | 从运行快照绘制真实环境、原始起终点、模型姿态、首解/最终/预算路径及cost–time图 | `draw_environment`、`draw_robot`、`draw_path`、`draw_cost_time` |
| `tools/audit_measurements.py` | 独立求积复算保存多项式的cost/长度，核对端点/拼接、预算路径来源与C++源码哈希 | 结果独立审计 |
| `tools/native_check.py` | 模型对照后调用真实官方DynoBench checker，记录命令、日志、状态；不匹配/未请求不是通过 | 外部checker适配 |
| `tools/requirements.txt` | numpy、matplotlib、PyYAML依赖声明 | Python依赖 |

## 4. 测试代码

| 文件 | 验证内容 |
| --- | --- |
| `test_controllability.cpp` | 积分链可控性矩阵、rank=rn、各输入链可控性指数为r |
| `test_gramian_scaling.cpp` | Gramian及小时间det阶数r²n，含非对角SPD权重和数值归一化 |
| `test_theory_assumptions.cpp` | 平坦映射、奇异性、约束状态化反例及理论尺度/假设检查 |
| `test_bvp.cpp` | 完整边界、固定/最优时间、正时间驻点与独立代价核对 |
| `test_planner.cpp` | 有向双树、range插值/重连接、端点、首解与shortcut历史、预算约束 |
| `test_validation.cpp` | 有限网格薄障碍漏检、可选几何检查、网格端点、采样预算与失败计数 |
| `test_models.cpp` | 物理映射、边界、ODE残差、奇异性和reduced-r2拼接/端点例外 |
| `test_batch.cpp` | scalar/SIMD差分、活动lane、时间边界、物理约束边界与失败分类 |
| `test_geometry.cpp` | 原始外形、SAT、球盒、接触边界、随机scalar/SIMD差分 |
| `test_io.cpp` | 首解/最终路径分开保存，指标与路径一致、导出时间/状态及认证标记 |
| `test_measurements.py` | 预算前驱查询、失败空值与统计分母、不可变输入快照、模型差异、四元数姿态、native适配mock |
| `fixtures.h` | 运行时测试模型夹具 |
| `theory_test_support.h` | 理论测试的矩阵、求积/断言等公共辅助 |

CMake组合10个C++测试程序、2项自定义n/r用例及3项CLI测试，形成15个启用测试项，另有6项禁用的场景基准；
Python测试另行运行。测试通过不等于六环境每个seed有解或原生动力学认证。

## 5. 冻结内核与参考源码

`vendor/rrtpff`是带`SOURCE_MANIFEST.json`哈希的既有内核快照，正常调参不修改。
PFF是内部求解器的来源名称，当前`bvp.h`使用完整末端状态约束，不能据此把
生产规划器称作RRT*/PFF。

| 文件（相对vendor/rrtpff） | 职责 |
| --- | --- |
| `pff/pff_integrator.h` | 积分链A/B、状态转移、Gramian等线性系统数学 |
| `pff/pff_cost.h` | 基础二次型运行代价权重 |
| `pff/pff.h` | 固定时间边界求解、终端约束、能量系数与最优时间相关数学 |
| `df/unicycle_df.h` | Unicycle平坦映射、速度分支与速度/角速度等限制 |
| `df/quad2d_df.h` | Quad2D平坦映射、姿态/推力/转矩恢复与限制 |
| `df/quad3d_df.h` | Quad3D姿态、角速度、推力/转矩/执行器映射与限制 |
| `df/paper_r2_df.h` | Quad reduced-r2的物理状态到flat边界lift |
| `df/differential_flatness.h` | 通用平坦映射模板接口 |
| `df/df_types.h` | 平坦映射使用的元数据、分支和配置等公共类型 |
| `common/flat_state.h` | 按导数块排列的flat状态 |
| `common/flat_control.h` | flat控制向量 |
| `common/flat_trajectory.h` | 多项式曲线、状态和任意阶导数求值 |
| `common/physical_trajectory.h` | 物理状态/控制序列容器 |
| `common/boundary_diagnostics.h` | 完整物理边界误差与姿态误差函数 |
| `common/types.h` | 标量、系统/环境、失败原因枚举与通用记录 |
| `common/math_utils.h` | 数学辅助函数 |

`references/ompl_fe44ef0/FlatStateSpace.h/.cpp`、`RRTConnect.cpp`是公开参考
源码，`SOURCE.md`记录来源。它们不编译进本工程可执行文件；生产实现位于
`include/flask/planner.h`，不应通过修改references改变实际算法。

## 6. 六环境配置与三个数据入口

| 根配置（examples下） | 原始环境（dynobench_data下） | 规划模型（dynobench_data下） | 可视模型（dynobench_original_models下） |
| --- | --- | --- | --- |
| `dynobench_uni_bugtrap.yaml` | `uni_bugtrap.yaml` | `unicycle1_v0_model.yaml` | `unicycle1_v0.yaml` |
| `dynobench_uni_wall.yaml` | `uni_wall.yaml` | `unicycle1_v2_model.yaml` | `unicycle1_v2.yaml` |
| `dynobench_quad2d_bugtrap.yaml` | `quad2d_bugtrap.yaml` | `quad2d_v0_model.yaml` | `quad2d_v0.yaml` |
| `dynobench_quad2d_hole.yaml` | `quad2d_hole.yaml` | `quad2d_v0_model.yaml` | `quad2d_v0.yaml` |
| `dynobench_quad3d_block.yaml` | `quad3d_block.yaml` | `quad3d_v0_model.yaml` | `quad3d_v0.yaml` |
| `dynobench_quad3d_window.yaml` | `quad3d_window.yaml` | `quad3d_v0_model.yaml` | `quad3d_v0.yaml` |

- `dynobench.environment`决定障碍物、工作空间和物理起终点。
- `dynobench.model`决定规划动力学；本项目Unicycle/Quad3D为论文参数版本。
- `dynobench.visual_model`决定实际碰撞和绘图外形与原生模型差异对照，不覆盖规划动力学。

模型文件是共享的：改Quad3D规划模型会同时影响Block和Window。
要只改一个场景，应复制模型文件并修改该场景的`dynobench.model`路径。
自定义环境也应复制，保留原始文件及来源哈希，避免把改图后的结果称作原始基准。

`integrator_obstacle.yaml`、`unicycle_straight.yaml`、`quad3d_full.yaml`是CLI测试
示例，不是六环境批处理默认输入。`examples/time_budget/`是保留的另一套配置，
属于历史输入，当前benchmark默认不读取；修复版使用根配置或review_20261019副本，不混用旧sphere字段。

## 7. 常用参数在哪里改

### 搜索与代价：六份根配置

| 键 | 当前六环境值 | 含义 |
| --- | --- | --- |
| `planner.seed` | 41 | 单次不带覆盖参数运行时的seed；benchmark默认覆盖为20～39 |
| `planner.timeout` | 10 | 搜索与shortcut总计算预算，秒 |
| `planner.iterations` | 1000000 | 外层采样循环上限，不等于总BVP次数或Connect次数 |
| `planner.max_nodes` | 20000 | 两棵树合计节点上限；可能在10秒前触发 |
| `planner.range` | 0.25 | 归一化flat度量下的扩展控制；不是米制距离或Eq.33邻域半径 |
| `planner.shortcut` | true | 是否对首解执行Algorithm 4；false可保留未shortcut路径 |
| `planner.shortcut_attempts` | 100 | shortcut候选尝试上限，不是接受次数 |
| `planner.goal_bias` | 0 | 采样物理端点lift所得flat目标的概率；修改会偏离本轮配置 |
| `planner.connect_steps` | 0 | 0表示Connect不设额外步数帽，仍受预算/节点/Trapped/Reached终止条件限制 |
| `planner.metric_scale` | 未配置 | 长度n*r的正向量；默认按各导数块的采样范围extent计算归一化尺度 |
| `rho` | 1 | 正的执行时长惩罚系数；不是计算时间预算 |
| `weight` | I_n | 数学中的R，必须对称正定；n×n，不能把所有元素都设为1 |
| `n`、`r` | Uni/Q2:2,2；Q3:4,2 | 输出维数、积分链阶数；更改涉及维度、lift与物理契约，不是普通调参 |
| `bounds.lower/upper` | 各场景显式给定 | derivative-major的flat均匀采样范围；位置部分与原始环境范围取交集 |
| `lifting.start.speed` / `goal.speed` | Uni为0.3 | 非零端点lift速度；不代表全路径恒速或pose-speed采样 |

目标函数为`J=rho*T+integral(w^T R w dt)`。r2时`w=y''`。
增大R会更重视flat控制能量，增大rho会更重视执行时长；二者都可能改变边、
搜索过程与成功率，不能保证几何长度单调变化。

r2的flat变量排列：

```text
Unicycle: [x, y, vx, vy]
Quad2D:   [x, z, vx, vz]
Quad3D:   [x, y, z, yaw, vx, vy, vz, yaw_rate]
```

默认`constrain_sampling_bounds=false`时，非Integrator的高阶采样范围不自动
作为沿边物理约束；位置范围沿边检查，速度/角速度等由模型限制检查。
Q3当前yaw/yaw_rate采样上下界均为0。

### 验证、几何和输出

| 键 | 当前值 | 说明 |
| --- | --- | --- |
| `validation.step` | 0.01 | 沿轨迹执行时间的检查步长，秒；不是规划计算时间 |
| `validation.max_samples` | 20000 | 单边检查点数上限；需要更多点时拒绝，不自动稀疏化 |
| `validation.simd` | true | 请求SIMD；实际还依赖编译、CPU与模板维度，结果记录实际后端 |
| `validation.continuous_sphere_collision` | false | 仅支持球机器人+球障碍的额外检查；原始盒场景开启会明确报错 |
| `validation.subdivision_depth` | 默认16 | 上述额外球几何检查的细分上限 |
| `validation.enforce_physical_continuity` | Uni true，Quad r2 false | 是否强制物理拼接连续；Quad r2仅位置/速度连续不足以满足完整物理连续 |
| `validation.enforce_physical_endpoints` | Uni true，Quad r2 false | 是否强制完整物理起终点；不能用开关掩盖reduced契约差异 |
| `validation.junction_tolerance` | 默认1e-6 | 物理拼接/端点检查容差 |
| `robot_radius` | 仅非DynoBench球模型 | 原生模型从visual_model的shape/size读取，native配置不再允许此字段 |
| `sphere_cell_size` | native配置已移除 | 原始盒保留为盒，不再通过此参数做球覆盖 |
| `output_step` | 未显式设置 | 原生格式states/actions/times导出间隔；默认来自规划模型dt，不控制验证网格 |

保存路径中的`samples`来自验证步长采样；`output_step`主要控制另外导出的
DynoBench格式文件。`plot_six.py::draw_path`当前沿路径画约7个模型姿态，
修改`np.linspace(...,7,...)`可改变姿态示意密度，不影响规划。

### 规划模型文件中的动力学参数

| 机器人 | 修改键 | 注意 |
| --- | --- | --- |
| Unicycle | `max_vel/min_vel`、`max_angular_vel/min_angular_vel` | 速度和角速度限制；零速度处存在flat逆映射奇异性 |
| Quad2D | `m`、`I`、`l`、`max_f`、`max_vel`、`max_angular_vel` | 质量、惯量、力臂、归一化电机控制限幅与速度限制 |
| Quad3D | `m`、`J_v`、`arm_length`、`t2t`、`motor_control`、`u_lb/u_ub`、`max_f`、速度限制 | 当前motor_control=false使用wrench映射；u与物理推力/转矩有缩放，不能把max_f当作该模式全部控制约束 |

DynoBench数据分支中，`max_acc`、`max_angular_acc`被视为启发式界，
没有直接作为规划硬约束。模型文件`distance_weights`也不作为本工程NN度量；
实际NN由`planner.metric_scale`控制。有`dynobench`块时不要在顶层加`model`
期待覆盖动力学；该分支读取的是`dynobench.model`指定文件。

## 8. 修改与运行示例

### 单场景：不编辑配置，覆盖seed/预算/backend

```bash
mkdir -p results/manual_seed20
./build-measurements/flask_plan \
    examples/dynobench_quad3d_window.yaml \
    results/manual_seed20/window.yaml 20 10 simd
```

位置参数顺序是`场景 输出 [seed [预算秒 [scalar|simd]]]`。
CLI覆盖优先于YAML。单次CLI输出的是路径文件，六图/统计工具需要benchmark
产生的实验目录及manifest，不能把这个单文件目录直接传给plot_six。

### 修改某个场景的参数

可以直接编辑配置；当前六份根配置实际是合法JSON写法，JSON也是YAML子集。
在Ubuntu也可用标准Python安全修改，不需要sed文本替换：

```bash
python3 - <<'PY'
import json
from pathlib import Path
p=Path('examples/dynobench_uni_bugtrap.yaml')
c=json.loads(p.read_text())
c['planner']['range']=0.25
c['planner']['shortcut']=True
c['planner']['shortcut_attempts']=100
c['rho']=1
n=c['n']
c['weight']=[[int(i==j) for j in range(n)] for i in range(n)]
p.write_text(json.dumps(c,indent=2)+'\n')
PY
```

改JSON/YAML不需要重新编译。手动改完不要再运行`configure_six.py`，
否则range、预算、R、rho、r、shortcut等会被重置。

### 六环境20个不同seed与指定预算记录时刻

```bash
RUN_DIR="results/custom_$(date +%Y%m%d_%H%M%S)"
/usr/bin/python3 -I tools/benchmark.py \
    --executable build-measurements/flask_plan \
    --output "$RUN_DIR" --budget 10 --seeds $(seq 20 39) \
    --backend simd \
    --checkpoints .001 .005 .01 .05 .1 .5 1 5 10
/usr/bin/python3 -I tools/summarize_runs.py "$RUN_DIR"
/usr/bin/python3 -I tools/plot_six.py "$RUN_DIR" --seed 20 \
    --at-budget .01 .1 1 10
xdg-open "$RUN_DIR/cost_time.png"
```

`--seeds 20`是一个seed；`--seeds $(seq 20 39)`是20个不同seed；
`--seeds 20 --repeats 20`是同seed重复20次，统计含义不同。
输出目录必须是新目录。只运行单个环境可追加：

```bash
--scenarios examples/dynobench_quad3d_window.yaml
```

`--budget`覆盖所有场景的YAML timeout；`--checkpoints`优先控制记录时刻，
显式时刻不得超过预算，工具会自动加入总预算时刻。`recording.checkpoints_seconds`
当前只保留在配置作说明，benchmark不从该字段取时刻。
`--at-budget`只选择绘图时刻，不改变运行预算或重新规划。

## 9. 修改后是否需要编译/重跑

| 修改位置 | 编译 | 重跑规划 |
| --- | --- | --- |
| YAML搜索参数、动力学、环境、起终点 | 不需要 | 需要；旧结果保存的是旧输入快照 |
| include/src中的算法或验证实现 | 需要 | 需要，并做相应回归检查 |
| CMake/AVX2编译选项 | 重新配置并编译 | 性能对比需要重跑 |
| plot_six.py颜色、模型姿态个数、图标题 | 不需要 | 不需要，可从旧运行快照重画 |
| summarize_runs.py报表排版 | 不需要 | 不需要，可从旧原始记录重算 |
| 新增更早的预算查询时刻 | 不需要 | version-2已有完整事件历史时可事后查询；修改benchmark CLI只影响下一次运行 |

`goal_tolerance`虽可读取但当前RRTConnect没有用它判定Reached，改它不产生
期望的目标区域变化。`unicycle_sample_speed`是被拒绝的旧选项，不要恢复。
切换r需要同步bounds维度、lift、高阶边界与验证契约，并受上表CLI实例化限制。
理论测试可接受的动态n/r不等于CLI已经编译所有n/r组合。

完整测量语义见`measurement_protocol.md`；理论假设和范围见
`theoretical_guarantees.md`；已有实验差距见`measurement_results_20261018.md`。

2026-10-19：已加入Connect重复节点拒绝与duplicate_rejections计数；实际原始几何与SIMD说明见fix_20261019.md。
