# 从 Insta360 ERP 全景视频到 3DGS-SLAM：北京胡同场景的迁移路径、基线选择与改进路线

> 文档定位：基于对 `S3PO-GS`（本仓库）、`MCGS-SLAM`（arXiv 2509.14191 / ICRA 2026）、`ODGS-SLAM`（CVPR 2026）等公开代码与论文的实际核对，
> 回答"全景视频怎么喂进 3DGS-SLAM"、"MCGS-SLAM 能不能用 3~4 个前向切图跑"、"哪条路更可行"、"改进方向与相关工作怎么写"四个问题。
> 代码结论均标注了核对位置，可直接对照仓库源码验证。

---

## 0. 结论先行（TL;DR）

1. **你的 8 张切图不是 8 个相机，而是同一个相机的 8 个观测方向。** 全景图在几何上是**单视点（共心）球面投影**：切出来的 8 个透视视角共享同一个光心，两两之间基线为 0，相对位姿是**纯旋转**，而这个旋转在你切图的时候就已经精确已知了。

2. **由此直接判定：把 3~4 个前向切图喂给 MCGS-SLAM 是"能跑但退化"的用法。** 结论可以精确到残差层面：
   - MCGS-SLAM 的 MCBA 在跨相机（同一时刻不同相机）对上最小化稠密光度/几何残差。**共心 rig 下，两视图之间的诱导光流只依赖已知的固定旋转，与深度无关**（纯旋转的无穷远单应），因此跨相机项对深度的雅可比恒为 0，对位姿只剩"旋转约束"——而旋转已被标定固定。跨相机视差这一 MCGS-SLAM 的核心增益来源直接消失。
   - 叠加一个工程事实：其多相机模式在代码里是**按 3~4 相机 Waymo rig 写死的**（见 §3.2），塞 8 路需要动手术。
   - 你付出 K 倍算力，换回的是一个"宽视场单目系统"。

3. **可行的两条主线（推荐组合使用）：**
   - **路线 B（先跑通）**：ERP → **直接重投影成单个透视相机**（HFoV 约 90°~100°），喂 S3PO-GS / MonoGS / Splat-SLAM 等单目基线。注意：是**重新投影**，**不是把两张透视图左右"物理拼接"**（原因见 §4.1）。
   - **路线 C（推荐作为你的方法主线）**：把 8 张切图当作 **1 个位姿 + K 个观测方向** 喂给 S3PO-GS —— 每个时间步只估计 1 个 6-DoF 位姿，光度损失在 K 个切片上求和。这在数学上等价于"用 K 个针孔相机实现全景渲染"，既拿到 360° 覆盖，又避免宽视场投影畸变和 3DGS 光栅器在大 FoV 下的退化。**这是把"共心"这一物理事实显式建模进 SLAM 的写法，也是相对 MCGS-SLAM 这类"真多相机"方法的正确对照。**

4. 你的胡同数据恰好命中室外 3DGS-SLAM 最痛的三个点，而且它们可以组成**一篇完整论文的因果链**，而不是三个孤立 trick：
   **窄街巷（平移/尺度弱可观 + 侧墙在窄视场外）→ 用 360° 共心覆盖解；跨时段光照变化 → 外观解耦；行人/自行车/电动车 → 动态鲁棒。**

5. **信息量最高、成本最低的第一个实验**：`K ∈ {1, 3, 5, 8}` 的**覆盖率消融** + `中心视角 vs 侧向视角` 的**方向消融**。这张图能同时回答"多视图到底带来了什么"和"MCGS-SLAM 那套为什么在共心 rig 上不成立"。

---

## 1. 室外 RGB-only 3DGS-SLAM 还存在哪些问题

分四层列，并标注"在你的胡同场景会怎么表现"。

### 1.1 前端（跟踪）层

| 问题 | 机制 | 胡同场景的具体表现 | 现有思路 |
|---|---|---|---|
| 单目尺度漂移 / 尺度歧义 | 单目几何存在尺度-深度耦合 | 沿巷走 50~100 m 后全局尺度偏 5%~15% | S3PO-GS 用 MASt3R 点图锚定；Splat-SLAM 用全局 BA；MCGS-SLAM 用 rig 固定基线 |
| **走廊式运动导致的平移退化** | 沿巷轴前进时，前向视角几乎无侧向视差 | 侧向平移/横滚近乎不可观，位姿在弱方向漂 | 少有人正面解决，见 §6.2 第 7 条（这是你的机会） |
| 窄视场导致的约束方向性不足 | 单目一次只看 <90° | 两侧墙体（真正的尺度与几何约束来源）完全在视野外 | 多相机 / 全景 |
| 低纹理与重复纹理 | 青砖墙、灰瓦、门楣、"千篇一律"的院门 | 单目 3DGS 的光度跟踪在低纹理区间失败 | 深度先验、语义/结构先验 |
| 动态瞬态物 | 光度一致性假设破裂 | 行人、自行车、电动车、临时摊位 | WildGS-SLAM 式不确定性、语义掩码 |
| 光照/曝光变化 | 同一点亮度不一致 | 跨时段采集、日照斑驳（窄巷+高墙的阴影边界）、逆光、夜景 | per-image 仿射曝光 a/b、外观解耦 |
| 频繁纯旋转 | 走路时不停转头 | 单目在此退化（无法三角化） | 全景/宽 FoV 天然抗纯旋转 |
| 卷帘快门 + 运动模糊 | 消费级 X 系列为卷帘传感器 | 快走/骑行时 RS 畸变与拖影 | IMU 去卷帘、提高帧率、限速采集 |

### 1.2 后端（建图）层

| 问题 | 胡同表现 |
|---|---|
| 幽灵高斯 / 拖影 | 行人被"固化"成半透明高斯串，且污染会反噬跟踪（正反馈） |
| 外观被"平均" | 同一点在不同曝光下被观测，高斯颜色趋灰/发白；跨时段更严重 |
| 显存与算力随轨迹爆炸 | 一条 200~300 m 胡同，高斯数可达数百万级 |
| 累积漂移 + 回环代价高 | 现有 `global_BA` 在全图上的求解成本高 |
| 深度先验误差累积 | MASt3R / 单目深度网络在低照度、动态、宽 FoV 下退化，误差进入初始化并累积 |

### 1.3 表示层

- 各向异性高斯在**弱约束区域过拟合**（天空、树叶、玻璃、水面、挂晒的衣物）。
- 城市级内存管理（子图 / 八叉树 / 剪枝）在室外长序列上是必需的。
- **宽 FoV 与 3DGS 光栅器的冲突**：标准 3DGS 用屏幕空间**局部仿射近似**（对投影雅可比在中心处展开）计算 2D 协方差，FoV 很大时图像边缘误差显著；这也是全向 3DGS 需要专门光栅器（OmniGS / ODGS / 360-GS）的原因之一。**这条直接约束了你能不能把 ERP 原图直接喂给针孔光栅器——答案是不能，这是 §3.5 里 ODGS-SLAM 对比数据的来源。**

### 1.4 评测层

- 缺乏室外**长序列 + 动态 + 跨时段**的标准基准：KITTI/Waymo 是车载前后向、稀疏 GT、侧向覆盖少，与"人拿着全景相机走街巷"差别很大。
- 3DGS-SLAM 对随机种子与超参敏感，**单次跑出来的数字不可信**（见 §7 协议）。
- 室外几何精度缺少 GT（胡同里 GNSS 完全不可用）。

---

## 2. 你的数据的几何本质（全文的地基，务必先想清楚这一节）

### 2.1 ERP 全景 = 单视点球面投影

Insta360 X 系列（及同类"背靠背双鱼眼"全景相机）输出的等距柱状（ERP）全景图，是被当作**单一投影中心**的图像来处理的：所有像素对应"从同一点出发、方向各异的射线"。
像素 (u,v) 与射线方向 (θ,φ) 是一一映射，画面里**不含任何视差信息**。

### 2.2 所以"8 切图"= 共心零基线 rig，不是 8 个相机

你在切图时做的变换，是把球面图像按 8 个方向重采样成 8 张针孔图像。这个操作用一个虚拟针孔相机 (K_k, R_k) 描述，其中：

- 8 个虚拟相机的**平移完全相同（= 0，相对全景光心）**；
- 8 个虚拟相机之间只差一个**固定的纯旋转 R_k**（k=0..7，绕竖直轴 45° 步进）；
- 因此任意两片之间的关系是 **homography at infinity**：`p_j = K_j R_ji K_i^{-1} p_i`，**与场景深度无关**。

一句话：**这 8 张图在信息上等价于"一个视点的 360° 观测"，不等于"8 个视点的多视角观测"。**

### 2.3 与 Waymo rig 的本质差异（对照表）

| 维度 | Waymo 车载 rig（MCGS-SLAM 的设计场景） | Insta360 ERP 切 8 片 |
|---|---|---|
| 相机间基线 | 0.4 ~ 1.0 m（论文原话：wide-baseline，strong parallax） | ≈ 0 |
| 相机间视差 | 有，可三角化 → 直接约束深度与尺度 | 无，纯旋转 |
| 多相机的作用 | **提供视差** → 深度/尺度/尺度一致性 | 只提供**方向覆盖**，不提供视差 |
| 位姿自由度 | 每相机 6-DoF（受 rig 外参约束） | 整个 rig 只有 **1 个 6-DoF 位姿** |
| 历元间约束 | 每相机时间维 + 跨相机空间维 | **只有时间维**（相邻时间步的位移） |
| 对光栅器的要求 | 针孔 | 针孔（切片）/ 全向（ERP 原生） |
| 与单目基线的关系 | 多视图冗余 = 强约束 | **等价于"宽视场/全向单目"** |

### 2.4 但"零基线 ≠ 没用"：真正的收益是**可观性（observability）**，不是视差

这是把你这套数据讲成一个**方法贡献**的关键论证：

- 跟踪的信息矩阵形如 `Σ_k J_kᵀ J_k`（K 个方向的光度残差雅可比之和）。共心 rig 下所有切片共享同一组 6-DoF 未知量，但残差方向在球面上**互补**。
- 在**走廊/窄巷**里，前向相机的残差对"沿巷轴平移"敏感，对"侧向平移 / 横滚 / 尺度"很弱；而**侧向切片观测墙面与地面**，恰恰补上这些弱可观方向。
- 因此共心多切片的收益可以精确表述为：**不是用三角化增加深度约束，而是把单目在特定方向上退化的可观性"各向同性化"**（isotropic conditioning）。
- 这在窄街巷、长直走廊、拱廊、隧道这类场景里是**结构性优势**，而室外 3DGS-SLAM 的失败案例恰好集中在这里。

⚠️ 这个论证必须讲清楚，否则审稿人会问："你基线是 0，多视图凭什么有用？" 你的答案就是上面这句：**约束方向覆盖 ≠ 视差增益**。

### 2.5 一个必须处理的现实细节：真实 Insta360 ERP 不是理想共心等距柱状投影

- 双镜头光心存在几厘米差异，机内拼接（stitching）假设共心，**近距离（胡同里墙离你只有 1~3 m）会出现鬼影/错缝**；
- 拼接接缝的方位是固定的，可能与你的关键方向重合；
- 全向相机的实际投影并非严格等距柱状（存在标定残差）。

建议做法（按代价从低到高）：
1. 采集时把**接缝朝低纹理方向**（天空/地面），或在不同时间步轻微改变相机 roll，让接缝不出现在同一位置；
2. 用棋盘格/标定场标定，量化 ERP → 切片的**重投影误差**，作为数据集质量指标写进论文；
3. 有条件时**绕过机内拼接**，直接用原始双鱼眼 + 全向相机模型（或 SC-OmniGS 那类可学习畸变校正）——这本身可以成为论文中"数据质量讨论"的一节。

---

## 3. MCGS-SLAM 能不能这么用？（明确判定 + 具体做法）

### 3.1 先说结论

**能跑，但不该这么跑。** 把 3~4 个前向切图喂给 MCGS-SLAM，你会得到一个"宽视场单目系统 + K 倍算力开销"，而它的两个核心卖点在共心 rig 上同时失效或无事可做：

| MCGS-SLAM 模块 | 前提 | 在共心切片 rig 上 |
|---|---|---|
| **MCBA**（多相机 BA，跨相机稠密光度+几何残差） | 相机间有**非零基线/视差** | 跨相机残差**与深度无关** → ∂r/∂d ≡ 0；对位姿只剩旋转约束（已被外参固定）。**核心增益消失** |
| **JDSA**（跨视图深度-尺度对齐，低秩先验） | 各相机的单目深度预测存在**视图相关的尺度/增益误差** | 这一条**仍然成立**（对 8 个切片分别跑深度网络，尺度确实会不一致），但它退化成一个"冗余多深度图一致性"问题；而你完全可以在 ERP 上跑一次深度网络、或在切片间做重叠区一致性约束，用更小的代价解决 |
| 宽 FoV 带来侧向覆盖 | — | **你已经免费拥有**（8 片就是 360°），不需要靠多相机实现 |

### 3.2 工程事实：它的多相机模式是按 3~4 相机写死的

核对 `mcgs-slam` 仓库源码（`mcgs_slam/options.py`、`mcgs_slam/depth_video.py`、`mcgs_slam/streams.py`）：

```python
# options.py
args.multi = len(args.imagedir) if len(args.imagedir) > 2 else False   # >2 路才算多相机
args.calib = np.array(params['intrinsic'])                             # 每相机 [fx,fy,cx,cy, k1,k2,p1,p2]
args.T_cami_cam0 = torch.as_tensor(params['T_cami_cam0'], ...)          # 每相机相对 cam0 的外参
args.timescale = float(params['timescale'])

# depth_video.py
self.multi = args.multi if args.multi > 2 else False
self.images_list = [self.images] + [zeros(...) for _ in range(self.multi-2)]   # 缓冲区按 multi-2 分配
... for ic in range(1, self.multi-1):  # 循环范围同样是 multi-1
```

- README 的示例命令是 `--imagedir ${seq}/front ${seq}/front_right ${seq}/front_left ${seq}/front_right`（4 个目录，且 `front_right` 重复出现），说明其默认配置就是 **Waymo 的 front / front_left / front_right** 三向 rig；
- 内部缓冲与循环范围按 `multi-1 / multi-2` 记账，**塞 8 路需要改代码**，不是改 yaml 就行；
- `image_stream()` 要求**每个相机一个目录**，文件名末尾必须带时间戳（正则取**文件名中最后一个数字**），并断言各相机同帧时间戳差 < 20 ms。

### 3.3 如果一定要跑，yaml 应该怎么写（共心切片版）

标定文件 `calib/xxx.yml` 的关键字段（外参是 lietorch `SE3` 的 7 元组 `[tx,ty,tz,qx,qy,qz,qw]`；已用 `calib/100613.yml` 的数值验证 `|q|=1`）：

```yaml
intrinsic: [                      # 每路相机 [fx, fy, cx, cy, k1, k2, p1, p2]
  [fx0, fy0, cx0, cy0, 0, 0, 0, 0],   # 中心切片（参考）
  [fx1, fy1, cx1, cy1, 0, 0, 0, 0],   # 左切片（HFoV 内切，无畸变）
  [fx2, fy2, cx2, cy2, 0, 0, 0, 0],   # 右切片
]
camera: 'pinhole'
baseline: [0, 0, 0, 0, 0, 0, 1]       # 共心 → 平移为 0
T_cami_cam0: [                        # 纯 yaw 旋转，平移全为 0
  [0, 0, 0,   0, 0, 0, 1],                                  # 0°
  [0, 0, 0,   0, sin(+30°/2), 0, cos(+30°/2)],              # +30° yaw
  [0, 0, 0,   0, sin(-30°/2), 0, cos(-30°/2)],              # -30° yaw
]
timescale: 1
ht: 480
wd: 640
```

**能跑起来 ≠ 结果有意义**：此时跨相机项贡献的只有"已知的旋转约束"，你花 3 倍算力得到的是一个宽 FoV 单目系统。

### 3.4 三种"公平使用 MCGS-SLAM"的方式（推荐）

既然 MCGS-SLAM 是很好的对比方法，别把它废掉，而是**用在它被设计出来的工况上**：

1. **复现验证**：用官方 Waymo demo + 单序列数据把它跑通，确认你的评测管线（ATE、PSNR/LPIPS、可视化）能对接。这是"我复现了它"的硬证据。
2. **零基线退化消融（这是能写进论文的实验）**：在**同一段胡同数据**上，构造两种输入给 MCGS-SLAM——
   (a) 3~4 片共心切片（零基线）；(b) 后续 §3.5 的"虚拟时移 rig"（非零基线）。
   预期结果：**(a) 相比单相机没有实质提升、但耗时线性增长；(b) 有提升但引入新的失败模式**。这条实验直接支撑你论文的动机段落。
3. **跨方法基准**：Waymo 数据是 S3PO-GS 与 MCGS-SLAM 的**共同测试场**（S3PO-GS 用 Waymo `FRONT`，MCGS-SLAM 用 Waymo 前向三相机）。在 Waymo 上做"单目 vs 多相机"的交叉验证，再把你自采的胡同数据作为**新的困难基准**引入。故事线非常干净。

### 3.5 附带的一个强证据：不要把 ERP 直接喂给针孔 3DGS-SLAM

`ODGS-SLAM`（CVPR 2026，代码与数据集已公开）在实拍全景数据上报告：把全景图直接送进针孔 3DGS-SLAM（如 MonoGS）的 ATE 明显劣于全向方法（具体数值以其论文表格为准）。
这与 1.3 节的"局部仿射近似在大 FoV 下退化"是同一件事的两面。**结论：全景图的处理必须在投影层做对（切切片 / 用全向光栅器），不能指望网络自己学会。**

### 3.6 进阶选项：虚拟时移 rig（想让 MCGS-SLAM 发挥实力的唯一正路）

思路：人为制造基线。构造一个 K 相机"虚拟 rig"，第 k 个相机的图像取**时间步 t_k 的全景切片**，相邻相机之间间隔 Δt，则相机间基线 ≈ 该时段的位移（走路 1.2 m/s、Δt=0.2 s → 约 0.24 m，与车载 rig 同量级）。

- 做法：先用单目 VO（路线 B 的产物）给出粗略轨迹，据此设定虚拟 rig 的外参初值，再交给 MCBA 精化。
- 代价与风险：**违反"rig 刚体 + 场景静止"假设**，只在运动近似匀速、且 Δt 很小时成立；胡同里行人/电动车会同时破坏这个假设。**只建议作为探索性实验，不作为主线。**

---

## 4. 路线对比与最终推荐

| 路线 | 输入形态 | 需要的工作 | 科学价值 | 风险 | 建议 |
|---|---|---|---|---|---|
| **A** 直接喂 MCGS-SLAM 3~4 片 | K 个共心切片 + 标定 | 造 calib yaml + 目录组织（改代码以支持 K>4） | 低（模块退化） | 中 | 仅作退化消融/对比项 |
| **A′** 虚拟时移 rig 喂 MCGS-SLAM | K 个时间错位切片 | 需要先有 VO 轨迹 | 中（可发表为"全景→多相机"桥接） | 高 | 探索性 |
| **B** 单透视重投影 + 单目基线 | **1 张**（直接 ERP→透视） | 只需数据适配 | 中（拿到第一条基线） | **低** | **先做，1~2 周** |
| **C** 共心多切片 + 单一位姿（扩展 S3PO-GS） | K 张切片共享 1 个位姿 | 中等（改 tracking/mapping 循环 + 可见性并集） | **高（主贡献）** | 中 | **主线** |
| **D** ERP 原生（ODGS-SLAM / OmniGS 光栅器） | 整张 ERP | 换光栅器 + 改 SLAM | 高（最"正统"） | 中高（生态、依赖、显存） | 并行预研，中期评估 |

### 4.1 "拼接成一个视角"的正确姿势：**重投影，不是物理拼接**

这一点很重要，容易被做错：

- ❌ **错误做法**：把左中右三张 90° 透视图**左右并排拼**成一张宽图。这产生的是一个**复合投影（非单一针孔）**图像：
  - 3DGS 光栅器假设**单一 K**，复合投影下它的投影函数不成立，图像边缘会出现无法解释的几何错位；
  - 拼接处产生**不连续的一阶导（Jacobian 断裂）**，光度损失在接缝处不可微，优化会在接缝附近产生"假特征"；
  - MASt3R / DROID / 深度网络等先验全部假设针孔模型，复合投影会让它们失真。
- ✅ **正确做法**：**直接从 ERP 重采样到一张目标透视相机**（给定 K，对每个像素求射线方向 → 球面采样）。这在数学上就是"模拟一台广角相机"，唯一代价是边缘像素的拉伸（角分辨率非均匀）。

### 4.2 视场角预算（关键参数取舍）

| 目标 HFoV | 评价 | 用途 |
|---|---|---|
| ≈ 60°~75° | MASt3R / DROID 等先验最舒服 | 与官方 Waymo 前视配置可比 |
| **≈ 90°~100°** | **推荐工作点**：覆盖大部分几何收益，光栅器与先验都还能工作 | 路线 B 主配置 |
| ≈ 110°~120° | 边缘拉伸明显；3DGS 光栅器与单目先验开始退化 | 仅作对比 |
| > 130° | 不建议走针孔 | 应改用切片（路线 C）或全向光栅器（路线 D） |

切片参数建议（以 8K ERP = 7680×3840 为例，角分辨率 21.3 px/°）：

| 项 | 建议值 | 说明 |
|---|---|---|
| 水平步长 | 45°（你已有） | 8 方向 |
| 单切片 HFoV | 80°~100° | 与 45° 步长配合 → 相邻重叠 35°~55°，足够做重叠一致性 |
| 单切片 VFoV | 按 4:3 或 16:9 取 | 竖直方向可裁掉部分天空/地面 |
| 喂给 SLAM 的分辨率 | 640×480 ~ 800×600 / slice | 3DGS-SLAM 显存与像素数成正比，别用 4K |
| 各切片光照 | 同一历元同曝光（ERP 已经是） | 避免切片间伪光照差 |

**别忘了 8 片的算力账**：K=8 时前端渲染 8 次/帧。务实做法是**跟踪用 K=3~4（前向 + 两侧），建图用 K=8**——这本身又是一个可写的消融实验。

---

## 5. 把 S3PO-GS 迁移到胡同数据：可执行改造清单

### 5.1 它现在到底吃什么（源码核对结论）

| 环节 | 事实 | 源码位置 |
|---|---|---|
| 目录结构 | `<dataset_path>/rgb/*.png`、`depth/*.png`、`mono_depth/*.png`、`gt/*.txt` | `utils/dataset.py: WaymoParser` |
| GT 格式 | 每帧一个 txt，**4×4 行主序空格分隔**；代码 `np.loadtxt(...).reshape(4,4)` 后**取逆** → **文件里存的是 c2w（相机到世界）** | `WaymoParser.load_poses` → `poses.append(np.linalg.inv(pose))` |
| GT 用在哪 | **只用在两处**：(1) 初始化第 0 帧位姿 `viewpoint.update_RT(viewpoint.R_gt, viewpoint.T_gt)`；(2) `eval_ate` 中的 `kf.R_gt/T_gt` | `slam_frontend.py:initialize` / `eval_utils.py:89` |
| 后续帧位姿怎么来 | MASt3R 估计与最近关键帧的**相对位姿** `get_pose()` + 光度优化 → **没有 GT 也能跑，只是没有 ATE** | `slam_frontend.py:tracking` |
| 深度怎么用 | `MonocularDataset.load_image`：三通道图只取 `[:, :, 0]`（深度可复用 RGB 占位，`dl3dvParser` 就是让 `depth_paths = color_paths`）；`mono_depth` 实际由 MASt3R 在线生成 | `utils/dataset.py:load_image`、`slam_frontend.py:145,170` |
| 尺度对齐 | `process_depth()` 用 MASt3R patch 匹配把渲染深度与 MASt3R 深度做**分块尺度拟合**（pointmap replacement） | `utils/depth_utils.py`、`add_new_keyframe` |
| 光照 | 每相机有 `exposure_a/exposure_b` 仿射参数，**在 tracking 里被优化**（`color_refinement: True`） | `camera_utils.py`、`slam_frontend.py:tracking` |
| 关键帧判据 | `dist > kf_translation × median_depth` 或（共视比 < `kf_overlap` 且 `dist > kf_min_translation × median_depth`）；Waymo 配置 0.08 / 0.05 / 0.90 | `slam_frontend.py:is_keyframe` |

**结论：迁移成本主要在"数据格式 + 一个新 Parser/Config"，不在算法。**

### 5.2 数据组织（路线 B：单透视图，最快跑通）

```
datasets/hutong/hutong_01/
├── rgb/            000000.png 000001.png ...     # 从 ERP 重投影出的中心透视视图（HFoV 90~100°）
├── depth/          -> symlink 到 rgb/            # 占位（单目模式不用）
├── mono_depth/     -> symlink 到 rgb/            # 占位，MASt3R 在线生成
└── gt/             000000.txt 000001.txt ...     # c2w 4×4（没有 GT 就放单位阵，放弃 ATE）
```

配置文件 `configs/mono/hutong/01.yaml`：

```yaml
inherit_from: "configs/mono/hutong/base_config.yaml"   # 从 waymo/base_config.yaml 复制
Dataset:
  type: 'hutong'
  dataset_path: "datasets/hutong/hutong_01"
  Calibration:
    fx: <f>, fy: <f>, cx: <cx>, cy: <cy>      # 重投影目标透视相机的内参
    k1: 0.0, k2: 0.0, p1: 0.0, p2: 0.0, k3: 0.0
    width: 640, height: 480
    distorted: False                            # 切片后已无镜头畸变
```

同时在 `utils/dataset.py` 的 `load_dataset()` 里加一个分支，并按 `dl3dvParser` 的写法加 `HutongParser`（它是最简的参考实现：直接复用 rgb 路径当深度）。

**要调的参数（相对于 Waymo 配置）**：
- `Training.kf_translation / kf_min_translation`：这两个阈值是**相对 `median_depth` 的比例**（Waymo 配置 0.08 / 0.05），实质对应"相邻关键帧位移与该帧中位深度的比值"。换了抽帧率、步速、分辨率与场景尺度都必须重调：步行 1.2 m/s、2 fps 抽帧 → 每帧位移 0.6 m，与车载配置的量级完全不同。
- `Dataset.pcd_downsample_init`：户外尺度不同，注意高斯初始化点数。
- `depth.*` 阈值（MASt3R patch 匹配的容差）：换场景/换曝光条件后需要重新标定。

### 5.3 路线 C 的改造：共心多切片 + 单一位姿（推荐主线）

**核心改动只有一处：把"每帧一个观测"改成"每时间步 K 个观测、共享一组位姿参数"。**

```python
# 概念伪代码（slam_frontend.py: tracking / mapping 的最小改动示意）

# 1) Camera 侧：为每个切片保留 K 张图 + K 个内参 + 该切片的固定相对旋转 R_k（由切图参数给定）
#    位姿参数只有一份：cam_rot_delta / cam_trans_delta（挂在"共享位姿"上）

# 2) tracking：损失在 K 个切片上求和
for itr in range(tracking_itr_num):
    loss_tracking = 0.0
    for k in range(K):
        viewpoint_k.R = R_k_c2c.T @ viewpoint_shared.R        # w2c 形式：P_k = R_k · P_0
        viewpoint_k.T = R_k_c2c.T @ viewpoint_shared.T
        render_pkg = render(viewpoint_k, self.gaussians, self.pipeline_params, self.background)
        loss_tracking += get_loss_tracking(config, render_pkg["render"],
                                           render_pkg["depth"], render_pkg["opacity"], viewpoint_k)
    loss_tracking /= K
    loss_tracking.backward()          # 梯度全部回到同一组 6-DoF 参数
    pose_optimizer.step(); update_pose(viewpoint_shared)

# 3) 可见性账本：occ_aware_visibility[t] = 该时间步 K 个切片渲染可见性的并集
# 4) 关键帧 & 滑动窗口：以"时间步"为粒度（不变量，沿用现有逻辑）
# 5) 后端 mapping：从 (t, k) 里采样，densification 的梯度用 K 个渲染的平均
```

**公式（务必在论文里写清楚）**：设全景相机 t 时刻的位姿为 `C(t) ∈ SE(3)`（c2w），切片 k 相对全景参考系的固定旋转为 `R_k`，则切片 k 的 c2w 位姿为 `C_k(t) = C(t) · R_k`，w2c 为 `C_k(t)^{-1}`。**未知量只有 C(t) 的 6 个自由度。**

这个改动带来的直接好处：
1. **保证 rig 一致性**：绝不会出现"8 个切片各估一个位姿、彼此矛盾"的退化解；
2. **8 倍的残差约束同一组未知量** → 尤其在侧向平移/横滚/尺度方向上显著改善条件数；
3. **地图天然 360° 覆盖**，把"单目 3DGS-SLAM 视场受限"这一公认弱点转为优势（窄街巷两侧的立面才是几何主体）；
4. 与 `exposure_a/b` 兼容：光照鲁棒性模块可以继续在这里扩展（§6）。

**建议的落地顺序**：
`K=1（中心）跑通` → `K=3（前 + 左 + 右）` → `K=5` → `K=8`，每一步都记录 ATE / PSNR / 显存 / 耗时，直接产出 §0 第 5 条所说的那张核心图。

### 5.4 坑清单

1. **MASt3R 的 FoV 敏感性**：MASt3R/DUSt3R 训练于常规透视图像，喂 HFoV > 100° 的图会退化。方案：(a) 跟踪用宽图、**给 MASt3R 喂中心裁剪**；(b) 保持 HFoV ≤ 90°；(c) 换度量深度网络（UniDepth/Metric3D/Depth-Anything-V2-metric）做先验并显式对齐尺度。
2. **gt 文件必须是 c2w**（见 §5.1），写反了会得到"能跑但结果离谱"的表现。
3. **深度图占位**：`depth/`、`mono_depth/` 直接 symlink 到 `rgb/`（照抄 dl3dv 的做法），不要留空目录。
4. **`distorted` 与畸变系数**：切片后应设 `distorted: False` 且 k/p 全 0；`camera_utils` 里 `dist_coeffs` 还会被 `depth_to_3d` 与 MASt3R 调用路径用到，不要留非零残值。
5. **抽帧率与关键帧阈值必须联动**：`kf_translation × median_depth` 是相对量，换了抽帧率/步速要重调，否则关键帧要么太密（算力爆炸）要么太疏（跟踪飘）。
6. **分辨率陷阱**：3DGS-SLAM 的显存 ≈ 高斯数 + 激活（正比于像素数 × K）。K=8 + 1080p 会直接 OOM。
7. **接缝与拼接鬼影**（§2.5）：采集阶段就要注意，别等到结果不对才发现。
8. **曝光参数初始值**：跨时段数据首帧若过曝/欠曝，`exposure_a/b` 的优化可能不收敛到合理区间。

---

## 6. 改进方向：动态瞬态物与光照变化

### 6.1 先分清"动态"和"光照"对前后端的影响（你的判断是对的，这里细化）

**A. 动态瞬态物（行人、自行车、电动车）**

| 影响面 | 机制 | 后果 |
|---|---|---|
| 前端 | 动态像素违反光度一致性/静态场景假设 → 残差被拉偏 | 位姿估计漂移；极端情况直接跟丢（前端**硬失效**） |
| 后端 | 动态物体被写进高斯地图；重复观测导致"幽灵/拖影"高斯 | 地图污染；**而且污染会反噬跟踪**（渲染出的假几何引导位姿）→ 正反馈恶化 |
| 深度先验 | MASt3R/深度网络把行人当有效几何 | 错误深度进入初始化与尺度对齐，误差累积 |
| 全景的天然优势 | 视场越大，动态像素在整帧中的**占比越小** → 鲁棒估计/外点剔除更容易（360VO 的论述） | 360 输入本身就是一种动态鲁棒性来源 |

**B. 光照变化**（你的数据是"不同采集时间段"，属于**跨时段/long-term**问题，不只是帧内曝光抖动）

| 时间尺度 | 现象 | 前端 | 后端 |
|---|---|---|---|
| 帧内/秒级 | 自动曝光抖动 | 光度残差被整体偏置 | 高斯颜色被平均 |
| 分钟级 | 走出一条阴影带（窄巷 + 高墙的日照斑驳）、逆光 | 跟踪在阴影边界失稳 | 同一墙面被观测成多种亮度 |
| **跨时段（小时/季节/天气）** | 光照方向、色温、植被、临时堆放物全变 | 跨会话重定位/回环困难 | 需要**外观解耦**（albedo vs 光照）与**会话嵌入**，否则地图"发灰、发白、互相抵消" |

### 6.2 可落地的改进点清单（按"改动量 vs 收益"排序）

1. **动态概率图（前端+后端共享）** —— 参照 WildGS-SLAM：DINOv2 特征 + 轻量 MLP 预测逐像素不确定性/动态概率，用它在跟踪残差里降权、在 mapping 的损失里掩码。
   *为什么适合你*：你已有重叠切片，**跨切片重叠区可以做一致性检验**（同一物在两片中的残差是否一致）作为额外监督信号。
2. **瞬态高斯集合** —— 参照 Street Gaussians / OmniRe 的思路：把高斯分成"静态背景"与"瞬态"两组，瞬态组只在滑动窗口内有效并定期清理；或在每个窗口对瞬态高斯做 opacity 衰减/剪枝。
   *收益*：直接消除"幽灵高斯反噬跟踪"的正反馈。
3. **鲁棒核对 + 掩码传播** —— 把前端的动态/异常掩码显式传给后端，并在损失里用 robust kernel（Cauchy/Geman-McClure）替代 L2/L1。
4. **外观解耦（跨时段的核心）** —— 三步走：(a) 把现有 per-image `exposure_a/b` 扩成 **per-image 色彩仿射 + per-Gaussian 可学习 albedo**；(b) 引入**会话/时间嵌入（session embedding）**，类似 WildGaussians / NeRF-W 的做法；(c) 评估时用"跨时段留出视角"。
5. **跨会话 Sim(3) 对齐 + 子图融合** —— 每个采集时段建一个子图，段间用全景描述子（旋转不变，比 ORB 更适合全景）做回环 → Sim3 位姿图 → 高斯子图变换合并（注意：几何对齐不等于外观一致，需要外观一致性约束）。
6. **几何先验的鲁棒化** —— 用度量深度网络 + 显式尺度对齐，替代/补充 MASt3R 的 patch 尺度拟合；在低照度与动态区域降低深度先验权重。
7. **走廊退化的显式建模** —— 你的场景是"窄巷 + 墙面 + 地面"：加入地平面/竖直墙面先验（曼哈顿假设的软化版）、或直接引入 IMU（Insta360 X 系列有 IMU，且"Gyroflow"式去卷帘已有成熟工具），用惯性约束补上侧向平移/横滚方向的可观性。
8. **全景友好的一致性度量** —— 切片重叠区的光度/几何一致性作为自监督正则（**零基线 → 重叠区在无动态时应严格一致**）：这是一条**免费的强监督**，既能检测动态物体，也能做地图一致性检查。这是我认为最"你独有"的一条。
9. **效率** —— 跟踪用 K=3~4、建图用 K=8；关键帧冗余度检测可利用全景的**旋转冗余**特性（同一位置不同朝向的内容高度相似，ODGS-SLAM 正是用它做关键帧剔除）。

### 6.3 我认为最值得下手的三个选题（按推荐度）

1. **共心全景 rig 的 3DGS-SLAM 位姿参数化与约束设计**（路线 C 的形式化 + 覆盖率/方向消融 + 与 MCGS-SLAM 的零基线退化对照）。
   *价值*：把"全景→切片"从工程 trick 提升为可分析的几何贡献，且是后续所有工作的地基。
2. **动态瞬态物与跨时段光照的联合鲁棒 3DGS-SLAM**（动态概率图 + 瞬态高斯 + 会话嵌入 + 重叠区一致性监督）。
   *价值*：正是 MCGS-SLAM 自己承认的 future work（"primarily designed for static environments; future work will focus on handling dynamic scenes"），且胡同数据天然提供这两种干扰。
3. **胡同/历史街巷的 3DGS-SLAM 基准**（数据 + 协议 + 多方法对比 + pseudo-GT 方案）。
   *价值*：社区缺的就是"室外、人手持、窄街巷、动态、跨时段"的基准，数据集类工作在影响力上通常回报稳定。

---

## 7. 评测方案与 GT 获取（这是"完整测评项目"的骨架）

### 7.1 GT 从哪来（胡同里没有 GNSS）

| 方案 | 能得到什么 | 成本 | 定位 |
|---|---|---|---|
| 手持/背包 LiDAR SLAM（如 Livox 系列、Leica BLK2GO 类） | 密集点云 + 轨迹（回环后） | 中 | **pseudo-GT**：轨迹与几何基准，需明确声明其非绝对真值 |
| 全站仪/控制点 + 静态 TLS | 绝对尺度的控制点、局部几何真值 | 高 | 用于**尺度校验**与局部精度评估 |
| 同一趟数据的 COLMAP/全景摄影测量（已知 rig 外参） | 参考轨迹 + 参考几何 | 低 | 与 SLAM 结果交叉验证（注意"自己评自己"的循环性） |
| 只评建图 / 只评跟踪 | 用统一轨迹评地图，或统一地图评轨迹 | 低 | 消融实验时的务实做法 |

**建议**：至少要有 (a) 一个 LiDAR 轨迹作为 pseudo-GT，(b) 若干全站仪控制点做绝对尺度校验，(c) 明确写清 GT 的获取与局限。这一节的严谨程度直接决定审稿人是否相信你的数字。

### 7.2 指标与协议

| 类别 | 指标 | 注意事项 |
|---|---|---|
| 跟踪 | ATE RMSE（Sim3 对齐 + 分段对齐）、RPE、跟踪失败率 | 3DGS-SLAM 必须**报告多次运行的 mean ± std**（≥3 seeds） |
| 渲染 | PSNR / SSIM / LPIPS（**区分插值视角与留出视角**、区分同段与跨时段） | 只报 PSNR 是社区常见弱点，务必给几何指标 |
| 几何 | 与 GT 点云的 Chamfer / F-score、深度误差（有 LiDAR 时） | 历史街巷的木构/檐口等细结构要单独统计 |
| 鲁棒性 | 动态残留指标（沿轨迹的高斯密度异常）、跨时段渲染一致性、不同时段的重定位成功率 | 这是你的贡献最能体现的地方 |
| 效率 | 端到端耗时、FPS、峰值显存、高斯数量 | 室外 3DGS-SLAM 的必报项 |
| 数据发布 | 同时提供 ERP 原图 + 8 切片 + COLMAP 格式 + Nerfstudio `transforms.json` + GT | 便于他人复现，数据集类工作的关键 |

### 7.3 你需要先建立的"对照实验矩阵"

```
行：方法 = {S3PO-GS(单目), 路线C(S3PO-GS+共心多切片), MonoGS, Splat-SLAM, HI-SLAM2,
            MASt3R-SLAM, WildGS-SLAM, ODGS-SLAM, MCGS-SLAM(零基线/虚拟rig)}
列：配置 = {K=1, K=3(前+左右), K=5, K=8} × {同段, 跨时段} × {无动态筛选, 有动态}
格：ATE / PSNR / SSIM / LPIPS / VRAM / 时间
```

---

## 8. 相关工作（草稿，可直接改写进论文）

### 8.1 3DGS-SLAM：从室内稠密跟踪到室外大尺度

3D Gaussian Splatting 以显式、可微、可实时光栅化的场景表示，迅速成为稠密 SLAM 的主流地图载体。MonoGS 首次将 3DGS 作为单目 SLAM 的唯一表示，实现光度跟踪与地图的联合优化；随后 Splat-SLAM 与 LoopSplat 通过全局联合优化与回环闭合缓解漂移，HI-SLAM2 引入几何感知的尺度对齐以提升单目重建精度，MGS-SLAM 以稀疏化与深度平滑正则降低计算开销。面向无界室外场景，S3PO-GS 提出以 3DGS 点图为锚的自一致跟踪模块与基于 patch 的点图动态建图模块，避免累积尺度漂移并缓解单目尺度歧义；OpenGS-SLAM 则探索了开放词汇语义与高斯的耦合。然而，这些方法**绝大多数建立在窄视场单目/单相机假设之上**：视场受限导致几何约束方向性不足、尺度与平移在长直通道类场景中弱可观，且大 FoV 下标准 3DGS 光栅器的局部仿射近似会失效。多相机方向，MCGS-SLAM 提出首个纯 RGB 多相机 3DGS-SLAM 框架，通过多相机光束法平差（MCBA）与联合深度-尺度对齐（JDSA）在宽基线 rig 上取得优于单目的轨迹与渲染精度；M2-Mapping、BAMF-SLAM 等则在 NeRF/BA 侧探索了多相机融合。**但多相机方法的增益来源是相机间的非零基线与视差，其在共心（零基线）相机阵列上的行为尚未被讨论。**

### 8.2 全向视觉与三维重建/定位

全向视觉以单次曝光获取 360° 观测，天然缓解纯旋转退化与视场受限问题。传统侧，OpenVSLAM、CubemapSLAM、PAN-SLAM 与 360VO 分别以等距柱状/立方体贴图/统一全向模型实现全景特征或直接法里程计，360VO 明确指出大视场可提升静态像素占比、从而增强动态环境鲁棒性。3DGS 侧，OmniGS 与 ODGS 通过为等距柱状（或切平面-球面两步）投影推导解析梯度，实现了直接在全景图上光栅化与渲染，避免了立方体校正与切平面近似带来的误差；SC-OmniGS 进一步将相机位姿与畸变模型联合自标定；ODGS-SLAM（CVPR 2026）则首次把全向 3DGS 作为全景 SLAM 的统一表示，补全了相机外参的解析梯度，并利用全景的"同位置不同朝向内容近似"特性做基于图分析的关键帧剔除，在实拍与仿真全景序列上取得显著优于针孔 3DGS-SLAM 的 ATE。**然而，全向 3DGS-SLAM 仍主要在有控场景验证，尚未在长时间、跨时段、强动态的真实历史街巷中检验。**

### 8.3 动态环境下的 SLAM 与高斯地图

动态物体是室外 SLAM 的主要失效源。传统方法依赖语义分割或几何一致性剔除动态特征；神经/高斯方法则进一步在地图层面处理。DynaGSLAM、DGS-SLAM 等在室内动态场景中通过语义与运动先验抑制动态高斯；WildGS-SLAM 以 DINOv2 特征驱动的不确定性图在不依赖语义标签的前提下同时改进跟踪与建图，实现了"野外"动态场景的无伪影重建。城市动态场景重建方向，Street Gaussians、OmniRe、PVG 等把动态物体（车辆、行人）建模为独立可控的时序高斯集合。**这些工作或聚焦室内、或聚焦车载城市数据，缺少"人手持全景相机、边走边拍、行人与非机动车高度混杂"的窄街巷设定。**

### 8.4 光照与外观变化下的鲁棒重建

跨时段采集带来的曝光、色温与光照方向变化，会同时破坏光度跟踪与地图外观一致性。经典方案在神经辐射场中引入外观嵌入与时变辐射场（NeRF-W）；3DGS 侧，WildGaussians、GS-W 通过外观建模与不确定性处理野外图像集合。SLAM 侧，"Taming the Light" 以本征外观归一化（IAN）与动态辐射平衡损失（DRB）解耦 albedo 与瞬时照明，在挑战曝光条件下提升了跟踪与语义精度。**但在跨时段、跨季节的长期采集设定下，如何同时保证几何一致性与外观一致性，仍是开放问题。**

### 8.5 文化遗产/历史街巷的数字化

文化遗产的三维数字化长期依赖摄影测量与激光扫描；3DGS 的出现以更低的采集成本提供了照片级真实感与实时渲染能力，已在建筑遗产与城市尺度场景中得到验证（如基于无人机数据的现代建筑遗产工作、CULTURE3D 大规模文化地标数据集等）。**但对"历史街巷"这一类狭窄、高动态、跨时段、无 GNSS 的强挑战场景，目前尚缺少系统性的 3DGS-SLAM 方法与基准。**

### 8.6 小结（Gap）

综上，现有工作分别解决了**表示效率**（3DGS）、**全向投影**（OmniGS/ODGS-SLAM）、**多相机视差**（MCGS-SLAM）、**动态鲁棒**（WildGS-SLAM、DynaGSLAM）与**光照鲁棒**（WildGaussians、Taming the Light）中的部分问题，但**尚无工作系统研究"共心全景相机阵列在真实历史街巷中的 3DGS-SLAM"**：既缺少对该类输入（零基线、共享位姿）的几何分析与位姿参数化设计，也缺少在动态瞬态物与跨时段光照干扰下的鲁棒方法，更缺少相应的评测基准。本文即针对这一空白展开。

---

## 9. 参考文献与代码（按主题，含链接）

**3DGS-SLAM（单目/深度）**
- MonoGS: Gaussian Splatting SLAM, CVPR 2024 — https://github.com/muskie82/MonoGS
- Splat-SLAM: Globally Optimized RGB-only SLAM with 3D Gaussians, 3DV 2025 — https://github.com/eriksandstroem/Splat-SLAM
- LoopSplat: Loop Closure by Registering 3D Gaussian Splats, 3DV 2025
- HI-SLAM2: Geometry-Aware Gaussian SLAM for Fast Monocular Scene Reconstruction — https://github.com/WenjingTong/HI-SLAM2
- MGS-SLAM: Monocular Sparse Tracking and Gaussian Mapping with Depth Smooth Regularization, RA-L 2024
- **S3PO-GS: Outdoor Monocular SLAM with Global Scale-Consistent 3D Gaussian Pointmaps, ICCV 2025** — https://github.com/3DAgentWorld/S3PO-GS
- OpenGS-SLAM, ICRA 2025 — https://github.com/3DAgentWorld/OpenGS-SLAM
- MASt3R-SLAM: Real-Time Dense SLAM with 3D Reconstruction Priors, CVPR 2025 — https://github.com/rmurai0610/MASt3R-SLAM

**多相机 / 全向**
- **MCGS-SLAM: A Multi-Camera SLAM Framework Using Gaussian Splatting for High-Fidelity Mapping, ICRA 2026** — arXiv:2509.14191 — https://github.com/mcgs-slam/mcgs-slam
- **ODGS-SLAM: Omnidirectional Gaussian Splatting SLAM, CVPR 2026** — https://github.com/odgs-slam/odgs-slam ｜ 数据集：https://researchdata.uibk.ac.at/records/z6f6r-sjc65
- OmniGS: Fast Radiance Field Reconstruction using Omnidirectional Gaussian Splatting, IROS 2024
- SC-OmniGS: Self-Calibrating Omnidirectional Gaussian Splatting, arXiv:2502.04734
- 360VO: Visual Odometry Using A Single 360 Camera, ICRA 2022 — https://huajianup.github.io/research/360VO/
- BAMF-SLAM（多相机 BA）、M2-Mapping（多相机 NeRF SLAM）

**动态 / 光照鲁棒**
- WildGS-SLAM: Monocular Gaussian Splatting SLAM in Dynamic Environments, arXiv:2504.03886 — https://github.com/GradientSpaces/WildGS-SLAM
- DynaGSLAM / DGS-SLAM（动态场景 3DGS-SLAM）
- Street Gaussians / OmniRe / PVG（动态城市场景高斯表示）
- WildGaussians / GS-W（野外图像集合的 3DGS 外观建模）
- Taming the Light: Illumination-Invariant Semantic 3DGS-SLAM, arXiv:2511.22968

**综述 / 数据集 / 文化遗产**
- 3D Gaussian Splatting in Robotics: A Survey
- How NeRFs and 3DGS are Reshaping SLAM: A Survey
- Beyond Implicit Representations: Exploring Gaussian Splatting for Next-Generation SLAM (IOTJ 2025)
- A Survey on Collaborative SLAM with 3D Gaussian Splatting, arXiv:2510.23988
- CULTURE3D: A Large-Scale and Diverse Dataset of Cultural Landmarks and Terrains, arXiv:2501.06927
- 3D Gaussian Splatting for Modern Architectural Heritage（AGILE 2025）

**工具**
- 全景视频 → ERP → 多视角切图：Insta360 Studio / ffmpeg（`v360` 滤镜可直接做 equirect→perspective 重投影）+ 自研脚本
- 去卷帘/防抖：Gyroflow；标定：OpenCV/`kalibr`

---

## 10. 执行路线图（建议 10~12 周）

| 里程碑 | 周期 | 目标 | 交付物 |
|---|---|---|---|
| **M0** 数据与基线固化 | 1~2 周 | 数据格式落盘（§5.2）、S3PO-GS 单视角（K=1，HFoV 90°）跑通 | 第一条 ATE/PSNR/耗时基线；标定与重投影误差报告 |
| **M1** 对手复现 | 1~2 周 | MCGS-SLAM 官方 Waymo demo 跑通；补齐 MonoGS / MASt3R-SLAM / Splat-SLAM 复现 | 可比对的评测管线（ATE + 渲染 + 显存） |
| **M2** 核心实验（路线 C） | 3~4 周 | S3PO-GS 共享位姿多切片扩展；K∈{1,3,5,8} 覆盖率消融 + 前向/侧向方向消融 + **MCGS-SLAM 零基线退化对照** | 论文核心图 1~2 张；"共心多切片可观性"的分析与实验证据 |
| **M3** 鲁棒性方法 | 4~6 周 | 动态概率图/瞬态高斯 + 外观解耦与会话嵌入 + 重叠区一致性监督 | 动态与跨时段两个维度上的定量提升 |
| **M4** 基准与写作 | 2~3 周 | 补齐对比矩阵、GT 与协议文档、数据集整理发布 | 论文 + 数据集 + 开源代码 |

### 风险清单

| 风险 | 概率 | 缓解 |
|---|---|---|
| 单目 3DGS-SLAM 在胡同长序列上直接失稳，M0 拿不到可用基线 | 中 | 先用 K=1 但加严关键帧与更小分辨率；必要时先用 Waymo 序列验证流程再换数据 |
| MASt3R 在宽 FoV / 低照度下退化 | 高 | 中心裁剪喂 MASt3R；或换度量深度网络并显式对齐尺度 |
| K=8 显存/时间不可接受 | 高 | 跟踪 K=3~4、建图 K=8；分辨率下采样；子图 + 高斯剪枝 |
| 无可靠 GT，评测不被认可 | 中 | 引入 LiDAR pseudo-GT + 全站仪控制点做绝对尺度校验，并明确声明局限 |
| 跨时段数据的"非刚性变化"（植被、临时堆放）被误当作跟踪误差 | 中 | 采集时记录场景变化清单；评测中分离"几何变化"与"跟踪误差" |

---

## 附：一句话回答你最核心的那个问题

> **是"借鉴 3DGS 基线那套 COLMAP 多视角约束"，还是"把行进方向几个视角拼成一个视角"？**

都不是。你手里数据的正确抽象是：**一台共心全景相机 = 一个位姿 + K 个观测方向**。
- 因此不要去"拼接"（会破坏针孔假设），而要**重投影**（路线 B：单宽视场，先跑通）；
- 也不要把 K 个切片当成 K 个相机喂给多相机 SLAM（MCGS-SLAM 的核心增益在零基线下失效），而要**共享一个位姿、在 K 个方向上求光度损失**（路线 C：你的方法主线）；
- 如果条件允许，长期最"正统"的方向是**全向光栅器（路线 D，ODGS-SLAM/OmniGS）**——但它的工程量与生态成本更高，建议并行预研、中期决策。

**零基线带来的不是视差，而是全方位覆盖的可观性；这恰好是窄街巷最需要的东西。这就是你这套数据在 SLAM 里的真正价值。**
