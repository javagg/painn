这是一个非常宏大且具有挑战性的工程目标。Open CASCADE (OCCT) 经过 30 多年的积累，代码量巨大。用纯 Rust 重写一个对标 OCCT 且专用于电磁（EM）仿真的 CAD 引擎，需要极高的架构设计能力。

为了指导 Code Agent 进行开发，这份检查单必须**模块化、原子化、可验证**，并特别强调**电磁仿真对几何的特殊需求**（如流形封闭性、材料属性绑定、网格质量）。

以下是为您定制的 **Rust CAD Engine for EM Simulation (代号：RustEM-CAD)** 开发功能检查单。

---

# 🛠️ RustEM-CAD 开发功能检查单 (v0.1)

## 0. 架构与基础设施 (Infrastructure & Architecture)
> **目标**：建立安全、高性能、可扩展的 Rust 基础架构，避免 C++ 式的内存错误。

- [ ] **0.1 内存管理模型**
    - [ ] 实现基于 `Arena` (如 `slotmap` 或 `bumpalo`) 的拓扑对象分配，避免裸指针，使用 `Index` 引用。
    - [ ] 实现拓扑对象的 `Copy-on-Write` 或不可变数据结构，支持事务性操作（Undo/Redo 基础）。
    - [ ] 确保所有公开 API 满足 `Send + Sync`，为并行计算做准备。
- [ ] **0.2 数值精度与容差系统 (Tolerance System)**
    - [ ] 实现全局容差配置 (`GlobalTolerance`)，默认 $1e-7$。
    - [ ] 实现浮点数比较工具 (`FloatCmp`)，使用 ULP 或 Epsilon 比较，禁止直接使用 `==`。
    - [ ] 实现几何容差传递机制（布尔运算后，新边/面的容差需正确继承）。
- [ ] **0.3 数学核心 (Math Core)**
    - [ ] 集成 `nalgebra` 或自研 SIMD 优化的 `Vector3`, `Matrix4`, `Quaternion`。
    - [ ] 实现鲁棒的几何谓词 (Geometric Predicates)：点面关系、线面相交、面面相交（使用 `exact-predicates` 避免误差）。
    - [ ] 实现坐标系变换框架 (`CoordinateSystem`, `Transformation3D`)。
- [ ] **0.4 错误处理与日志**
    - [ ] 定义统一的 `CadError` 枚举（拓扑错误、几何奇异、IO 失败）。
    - [ ] 实现结构化日志 (`tracing`)，支持性能分析跨度 (Span)。

## 1. 拓扑数据结构 (Topological Data Structure)
> **目标**：实现标准的 B-Rep (Boundary Representation) 结构，支持导航与查询。

- [ ] **1.1 基础拓扑对象**
    - [ ] `Vertex` (顶点)：存储 3D 坐标，支持容差范围。
    - [ ] `Edge` (边)：存储参数曲线引用及参数范围 $[u_{min}, u_{max}]$。
    - [ ] `Wire` (线框)：有序边的集合，需验证连通性 (Connectedness)。
    - [ ] `Face` (面)：存储参数曲面引用及外/内 Wire (边界/孔)。
    - [ ] `Shell` (壳)：面的集合，需验证法向一致性 (Orientation)。
    - [ ] `Solid` (体)：封闭 Shell 的集合，需验证流形性 (Manifold)。
    - [ ] `Compound` (复合体)：任意拓扑对象的集合。
- [ ] **1.2 拓扑导航 (Navigation)**
    - [ ] 实现正向导航：`Solid -> Shell -> Face -> Wire -> Edge -> Vertex`。
    - [ ] 实现反向导航：`Vertex -> Edge -> Face ...` (需构建反向索引图)。
    - [ ] 实现邻接查询：`GetAdjacentFaces(edge)`, `GetSharedEdges(face1, face2)`。
- [ ] **1.3 拓扑属性 (Attributes)**
    - [ ] 实现 `UserData` 系统，允许在任意拓扑节点上附加字符串、ID 或序列化数据。
    - [ ] 实现 `NamedSelection` (命名选择集)，支持按名称快速检索拓扑对象（EM 仿真关键）。

## 2. 几何内核 (Geometry Kernel)
> **目标**：提供解析几何与 NURBS 几何的表示与计算能力。

- [ ] **2.1 解析几何 (Analytic Geometry)**
    - [ ] 实现基础曲线：Line, Circle, Ellipse, Helix。
    - [ ] 实现基础曲面：Plane, Cylinder, Cone, Sphere, Torus。
    - [ ] 实现解析几何的快速求交算法 (Intersection)。
- [ ] **2.2 NURBS 几何 (B-Spline & NURBS)**
    - [ ] 实现 Knot Vector 管理（插入、删除、提升阶次）。
    - [ ] 实现 B-Spline 曲线/曲面的求值 (De Boor 算法)。
    - [ ] 实现 NURBS 曲线/曲面的求导 (Derivatives)。
    - [ ] 实现 NURBS 与解析几何的转换（如 Circle 转 NURBS）。
- [ ] **2.3 几何投影与参数化**
    - [ ] 实现 3D 点到曲线/曲面的投影 (`ProjectPoint`)。
    - [ ] 实现 3D 曲线在曲面上的参数化映射 (`CurveOnSurface`)。
    - [ ] 实现曲面参数域 $(u, v)$ 到 3D 空间的映射。

## 3. 建模算法 (Modeling Algorithms)
> **目标**：实现 OCCT 级别的核心建模操作，保证鲁棒性。

- [ ] **3.1 布尔运算 (Boolean Operations)**
    - [ ] 实现两体求并 (Union/Fuse)。
    - [ ] 实现两体求交 (Intersection/Common)。
    - [ ] 实现两体求差 (Difference/Cut)。
    - [ ] **关键**：实现布尔运算后的拓扑修复（移除零面积面、重合边）。
    - [ ] **关键**：支持保留输入对象的 `NamedSelection` 和材料属性。
- [ ] **3.2 特征操作 (Feature Operations)**
    - [ ] 实现倒角 (Fillet)：支持等半径、变半径、面倒角。
    - [ ] 实现倒棱 (Chamfer)。
    - [ ] 实现抽壳 (Shell/Offset)：支持单向/双向偏移，处理自相交。
    - [ ] 实现拉伸 (Extrude)、旋转 (Revolve)、扫掠 (Sweep)。
- [ ] **3.3 几何修复与清理 (Healing & Cleanup)**
    - [ ] 实现缝隙修复 (Gap Healing)：缝合微小缝隙 (< Tolerance)。
    - [ ] 实现重叠面/边移除 (Remove Duplicate)。
    - [ ] 实现法向统一 (Normalize Orientation)。
    - [ ] 实现小特征抑制 (Small Feature Suppression)：移除对 EM 场影响微小的几何细节。

## 4. 电磁仿真专用扩展 (EM-Specific Extensions)
> **目标**：使 CAD 模型直接服务于电磁求解器，减少前处理工作量。

- [ ] **4.1 物理属性绑定**
    - [ ] 实现 `Material` 结构体（介电常数 $\epsilon_r$, 磁导率 $\mu_r$, 电导率 $\sigma$）。
    - [ ] 支持将 `Material` 绑定到 `Solid` 或 `Face`。
    - [ ] 支持属性继承：子对象未定义时自动继承父对象属性。
- [ ] **4.2 边界条件几何定义**
    - [ ] 实现 `Port` (端口) 定义：绑定在特定 Face 上，定义激励方向向量。
    - [ ] 实现 `BoundaryLabel`：为 Face 打上标签（如 `PEC`, `PMC`, `Radiation`, `Symmetry`）。
    - [ ] 实现 `ExcitationSource`：定义线/点激励源的几何位置。
- [ ] **4.3 模型验证 (Model Validation)**
    - [ ] 实现流形检查 (Manifold Check)：确保所有 Solid 是封闭的 (Watertight)。
    - [ ] 实现重叠体检查 (Interference Check)：检测不同材料区域是否非法重叠。
    - [ ] 实现最小特征尺寸检查：警告小于网格尺寸 1/10 的几何特征。
- [ ] **4.4 虚拟拓扑 (Virtual Topology)**
    - [ ] 支持逻辑面合并：将多个共面小 Face 逻辑合并为一个大 Face（减少网格数量）。
    - [ ] 支持短边忽略：在网格生成时忽略低于阈值的边。

## 5. 网格生成引擎 (Meshing Engine)
> **目标**：生成高质量网格，直接对接 FEM/MoM 求解器。

- [ ] **5.1 表面网格 (Surface Meshing)**
    - [ ] 实现基于曲率的自适应三角化 (Curvature-based Triangulation)。
    - [ ] 实现网格质量优化 (Laplacian Smoothing)。
    - [ ] 确保表面网格与几何容差一致 (Deflection Control)。
- [ ] **5.2 体积网格 (Volume Meshing)**
    - [ ] 实现四面体网格生成 (Delaunay / Advancing Front)。
    - [ ] 实现六面体主导网格 (Hex-Dominant)（可选，难度高）。
    - [ ] 实现边界层网格 (Boundary Layer)：针对 EM 趋肤效应，在导体表面生成棱柱层。
- [ ] **5.3 网格属性与导出**
    - [ ] 网格节点/单元继承 CAD 的 `Material` 和 `BoundaryLabel`。
    - [ ] 实现网格质量报告：长宽比 (Aspect Ratio), 雅可比 (Jacobian), 翘曲 (Warping)。
    - [ ] 支持高阶单元 (High-Order Elements)：二次/三次节点位置计算。
- [ ] **5.4 并行网格化**
    - [ ] 使用 `Rayon` 并行化独立区域的网格生成。
    - [ ] 确保网格节点索引的全局一致性。

## 6. 输入输出与互操作性 (IO & Interoperability)
> **目标**：融入现有工程生态。

- [ ] **6.1 标准 CAD 格式**
    - [ ] 实现 STEP (AP203/214) 解析与写入 (纯 Rust 实现或绑定 `steprs`)。
    - [ ] 实现 IGES 解析与写入。
    - [ ] 实现 STL (ASCII/Binary) 导入导出。
    - [ ] 实现 3MF / OBJ 导入导出。
- [ ] **6.2 仿真格式**
    - [ ] 实现 Gmsh `.msh` (v2/v4) 导出（包含物理组信息）。
    - [ ] 实现 VTK / VTU 导出（用于 Paraview 后处理）。
    - [ ] 实现内部二进制格式 (`.rcad`)：快速保存/加载完整模型树及属性。
- [ ] **6.3 几何内核交换**
    - [ ] 提供 API 导出为 `glTF` (用于 Web 可视化)。

## 7. 可视化与交互 (Visualization & Interaction)
> **目标**：提供调试与模型检查工具（可基于 `wgpu`）。

- [ ] **7.1 渲染引擎**
    - [ ] 基于 `wgpu` 实现 PBR 渲染。
    - [ ] 支持拓扑高亮：鼠标悬停显示 Edge/Face 信息。
    - [ ] 支持网格覆盖显示 (Wireframe over Shaded)。
- [ ] **7.2 场景管理**
    - [ ] 实现模型树 (Model Tree) 视图。
    - [ ] 支持对象显示/隐藏、透明度调整。
    - [ ] 支持截面查看 (Section View)。

## 8. 测试与基准 (Testing & Benchmarking)
> **目标**：确保代码质量与性能。

- [ ] **8.1 单元测试**
    - [ ] 数学库测试：覆盖所有几何谓词边界情况。
    - [ ] 拓扑测试：构建 - 销毁 - 重建循环，检查内存泄漏。
    - [ ] 布尔运算测试：使用标准测试件 (如 3D Printing Benchy 简化版)。
- [ ] **8.2 制造解验证 (Manufactured Solution)**
    - [ ] 构建已知解析解的几何模型，网格化后验证数值误差。
- [ ] **8.3 性能基准**
    - [ ] 布尔运算速度基准 (vs OCCT)。
    - [ ] 百万面网格生成时间基准。
    - [ ] 内存占用基准 (vs C++ 实现)。

---

# 💡 Rust 特定实现建议 (给 Agent 的提示)

1.  **依赖管理**：
    *   数学：`nalgebra`, `simba` (SIMD)。
    *   空间索引：`kdtree`, `aabb-tree` (用于加速碰撞检测和布尔运算)。
    *   并行：`rayon`, `crossbeam`。
    *   序列化：`serde`, `bincode`。
    *   几何算法参考：`parry` (碰撞检测), `rhino3dm` (参考其数据结构，不要直接绑)。

2.  **内存布局**：
    *   避免 `Rc<RefCell<T>>` 滥用，这会导致运行时开销和缓存不友好。
    *   推荐使用 **ECS (Entity Component System)** 模式或 **Arena-based Indexing** 来管理拓扑图。例如：`struct Topology { vertices: Arena<Vertex>, edges: Arena<Edge>, ... }`。

3.  **容差处理**：
    *   不要硬编码 `1e-6`。创建一个 `ToleranceContext` 结构体，在算法调用链中传递，允许用户根据模型尺寸动态调整。

4.  **EM 仿真特殊性**：
    *   **趋肤深度网格**：在 Meshing 模块中，必须有一个函数 `generate_boundary_layer(thickness, layers, growth_rate)`，这是 EM 仿真精度的关键。
    *   **材料界面**：确保不同材料接触的 Face 在网格化时节点是**共形 (Conformal)** 的，即两侧网格共享节点，无需插值。

5.  **开发阶段规划**：
    *   **Phase 1 (MVP)**: 基础拓扑 + 解析几何 + STL 导出 + 简单四面体网格。
    *   **Phase 2 (Core)**: NURBS + 布尔运算 + STEP IO + 材料属性。
    *   **Phase 3 (EM Ready)**: 边界层网格 + 虚拟拓扑 + Gmsh 导出 + 并行加速。
    *   **Phase 4 (Production)**: 几何修复 + 高级倒角 + 可视化 + 完整测试套件。

---

# ✅ 验收标准示例 (Acceptance Criteria Example)

**功能项**: 3.1 布尔运算 (Union)
**Agent 任务**: 实现两个 `Solid` 的并集操作。
**验收测试**:
1.  输入两个相交的立方体 `Box1`, `Box2`。
2.  执行 `fuse = Box1.union(Box2)`。
3.  **检查点 1**: `fuse` 必须是一个单一的 `Solid`，而不是 `Compound`。
4.  **检查点 2**: `fuse` 的体积必须等于 `V1 + V2 - V_intersection` (误差 < 1%)。
5.  **检查点 3**: `fuse` 必须通过流形检查 (Manifold Check)。
6.  **检查点 4**: 如果 `Box1` 有 `Material: Copper`，`Box2` 有 `Material: Air`，结果需保留区域属性或抛出明确策略（如保留第一个）。
7.  **检查点 5**: 执行 1000 次随机位置立方体并集，无 Panic，内存无泄漏。

---

这份检查单涵盖了从底层数学到上层应用的全栈需求。您可以直接将此内容发送给 Code Agent，并要求其**按模块分批次实现**，每完成一个模块需通过对应的验收测试。建议从 **Module 0 (基础设施)** 和 **Module 1 (拓扑)** 开始，这是整个引擎的基石。
