## 4.动力学约束下的路径规划

### （1）动力学约束

**Kinodyanmic** = **Kinematic**（运动学：eg.避障） + **Dynamic**（动力学：eg.速度，加速度和力上的 bounds）

-   Q：既然有 Trajectory Optimization 的步骤，为什么还要再 Path Finding 的过程中考虑高阶的 Kinodynamic 的约束？

​ ![image.png](https://s2.loli.net/2023/02/27/GfNqmVlednBiRCz.png)

-   A：
    -   Motion Planning 是一个 coarse to fine 的过程，并不一定要割裂的两部分，换句话说，如果在 Path Planning 中完全不考虑动力学约束，那么在动力学模型复杂的情况下可能不能得到一个很好的初始路线，那么即使经过 Trajectory Optimization，最终结果也会不那么理想
    -   Trajectory Optimization 是在局部进行优化的，并不能在全局保证动力学最优性
    -   eg. ![image.png](https://s2.loli.net/2023/02/27/eyKanrgQpZLOhjz.png)
        -   在本例中无人机具有向右的初速度，如果在 Path Planning 的过程中完全不考虑动力学约束，则会生成如紫色实线这样的路径，经过 Trajectory Optimization 后得到如紫色虚线这样的轨迹，这样的轨迹涉及到急转弯，在运动学上并不是最优的方案
        -   如果在 Path Planning 时就考虑动力学约束，得到如绿色实线这样的路径，经过 Trajectory Optimization 后很容易得到一条平滑的 feasible 的轨迹

---

### （2）常见的动力学模型

-   独轮车模型

    ![image.png](https://s2.loli.net/2023/02/27/RFs1DcM7GbaZLx5.png)

-   差速车模型

    ![image.png](https://s2.loli.net/2023/02/27/Lc91yfQ6WOSBmTt.png)

-   小车模型

![image.png](https://s2.loli.net/2023/02/27/KDEeBoHJRTjZk9u.png)

---

### （3）状态栅格搜索算法—State Lattice Planning

建树/图，使其上每条边都满足动力学约束（feasible motion connections），再运用前述的图搜索算法即可

#### A. Forward Direction：

**离散控制空间**，使机器人前进一段距离，形成路径（例如 L2 中提到的四联通，八联通，本质上是对质点模型控制量的离散化）

![image.png](https://s2.loli.net/2023/02/27/jYO64Fs3X2aPtTM.png)

-   在 differentially driven 的情况下有:

    $$
    \dot{s}=f(s,u)
    $$

-   其中 S 为状态向量，在自动驾驶任务中为

-   u 为输入量，在自动驾驶任务中为**a（油门：加速度），α（转向盘：角加速度）**

-   选定一系列输入量 u0~un，设定一个作用时间 T，即可进行前向仿真（Forward Simulation），得到一系列未来的离散状态**Sf0~Sfn**

-   ![image.png](https://s2.loli.net/2023/02/27/IJrYZXn6OgCSPps.png)

-   特点：

    -   无任务导向（类似 Dijkstra），效率低
    -   易于实施

-   实施：

    状态空间模型([ACnD 1. 状态空间模型 (State Space Model) - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/466790657))【**线性系统**】

    $$
    \dot{s}=A\cdot s + B\cdot u
    $$

    其中 s 为 n 维向量，包含系统的所有状态量，u 为系统的输入量（控制量）

    对于无人机的线性模型而言，取控制量为

    $$
    u=\begin{pmatrix}
    \ddot{x}\\
    \ddot{y}\\
    \ddot{z}\\
    \end{pmatrix}
    $$

    则状态空间为

    $$
    s=\begin{pmatrix}
    x\\y\\z\\
    \dot{x}\\\dot{y}\\\dot{z}
    \end{pmatrix}
    $$

    则有

    $$
    A=\begin{bmatrix}
    0&0&0&1&0&0\\0&0&0&0&1&0\\
    0&0&0&0&0&1\\0&0&0&0&0&0\\
    0&0&0&0&0&0\\0&0&0&0&0&0\\
    \end{bmatrix}
    &&B=\begin{bmatrix}
    0&0&0\\0&0&0\\
    0&0&0\\1&0&0\\
    0&1&0\\0&0&1\\
    \end{bmatrix}
    $$

    在如下初始条件下

    $$
    v_{0}=
    \begin{bmatrix}
    1\\0\\0
    \end{bmatrix}
    &即&s_{0}=
    \begin{bmatrix}
    x_{0}\\y_{0}\\z_{0}\\1\\0\\0
    \end{bmatrix}
    $$

    将控制量离散化，有如下结果：

    ![image.png](https://s2.loli.net/2023/02/27/vStWzPLHJfM1DIR.png)

    可以猜测离散得到的九个控制量可能为：

    $$
    u_{1}=
    \begin{pmatrix}
    1\\1\\0
    \end{pmatrix}
    u_{2}=
    \begin{pmatrix}
    2\\1\\0
    \end{pmatrix}
    u_{3}=
    \begin{pmatrix}
    3\\1\\0
    \end{pmatrix}

    u_{4}=
    \begin{pmatrix}
    1\\0\\0
    \end{pmatrix}
    u_{5}=
    \begin{pmatrix}
    2\\0\\0
    \end{pmatrix}
    u_{6}=
    \begin{pmatrix}
    3\\0\\0
    \end{pmatrix}

    u_{7}=
    \begin{pmatrix}
    1\\-1\\0
    \end{pmatrix}
    u_{8}=
    \begin{pmatrix}
    2\\-1\\0
    \end{pmatrix}
    u_{9}=
    \begin{pmatrix}
    3\\-1\\0
    \end{pmatrix}
    $$

    已知初始状态，已知控制量，可以根据状态转移方程算出输出状态

    ![image.png](https://s2.loli.net/2023/02/27/Dgn1tBGx9kiAver.png)

    在这里，如果将 A 构建为幂零矩阵（nilpotent matirx），会大大简化计算

    若再以得到的八个 s 为 s0 分别进行如上操作，则可以得到**lattice graph**，例如下图

    ![image.png](https://s2.loli.net/2023/02/27/XB3Z8KkLao4Ru6I.png)

-   eg. 对于汽车（非线性系统）

-   ![image.png](https://s2.loli.net/2023/02/27/ZQWzVIx74Ry8Sre.png)

---

#### B. Reverse Direction

**离散状态空间**，再将这些离散的状态点进行连接，形成路径（例如 L3 中提到的 PRM 本质上是对质点模型 x，y 两种维度的离散化）

![image.png](https://s2.loli.net/2023/02/27/RqBcm2gLClEsYoF.png)

-   对于 differentially driven 的情况:
-   选定一系列 Sf，计算从 S0~Sf 所需的 u 和 T（通常是固定 T，计算可能的 u）
-   ![image.png](https://s2.loli.net/2023/02/27/zIK7kViW6gHl9Y1.png)
-   ![image.png](https://s2.loli.net/2023/02/27/qcLomV8SgBj1AQE.png)
-   特点：
    -   自然的具有贪心性，很容易做得到目标导向，因此 planning 的效率较高
    -   但实施难度较大（对于给定起始和末尾状态点，有无数种状态转移路线，这就涉及到最优问题（**Optimal Boundary Value Problem**））

##### \* 两边界值最优问题（Optimal Boudary Value Problem, OBVP）

1. 获得可行解（BVP）

    - 对于一维状态转移问题
    - ![image.png](https://s2.loli.net/2023/02/28/1UDktG3SLxndRlQ.png)
    - 可以假设状态转移路线为五次曲线，五次曲线有 6 个待定参数，一对边界条件可以给出 2 个方程，因此对方程分别求一阶导和二阶导，共 2\*3=6 个方程，可以解出全部未知量
    - ![image.png](https://s2.loli.net/2023/02/28/ysH8PXr3KGAIzdE.png)

2. 获得最优解（OBVP）

    1. 问题的一般描述

        1. 用到的变量和函数有：

            s(t)：与 t 有关的 n 维变量，表示 t 时刻系统的状态量，有边界条件 s(0)

            u(t)：与 t 有关的 m 维变量，表示 t 时刻系统的输入量

            系统建模为：

            $$
            \dot{s(t)}=f(s(t),u(t))
            $$

            J(T)：终末时刻 T 时刻的惩罚函数值

            $$
            J(T)=h(s(T))+\int_0^Tg(s(t),u(t))dt
            $$

            其中:

            h(s(T))表征是否达到终点；g(s,u)表征路径中的 cost；

            H(s,u,λ)：Hamiltonian 方程

            $$
            H(s,u,\lambda)=g(s,u)+\lambda^Tf(s,u)
            $$

            λ：n 维协变量，维度与 s 一致

            ***

        2. 求解：

            s\*：最优的状态转移方程

            u\*：随时间最优的控制输入

            ***

        3. Pontryagin's Minimum Principle 庞特里亚金最小值原理

            λ(t)是如下微分方程的解：

            $$
            \dot{\lambda(t)}=-\nabla_sH(s^*(t),u^*(t),\lambda(t))
            $$

            要解此方程须有边界条件：

            $$
            \lambda(T)=-\nabla h(s^*(T))
            \\s^*(0)=s(0)\ \ \ (given)
            $$

            其中:

            $$
            \dot{s^*(t)}=f(s^*(t),u^*(t))
            $$

            则有所求的最优控制量为：

            $$
            u^*(t)=\arg\mathop{\min}\limits_{u(t)}H(s^*(t),u(t),\lambda(t))
            $$

            ***

    2. 实例：无人机

        1. 建模

            $$
            s_k=(p_k,v_k,a_k)\\(given\ \ s(0)=s_0,s(T)=s_f)\\
            u_k=j_k\\
            \dot{s_k}=f(s_k,u_k)=(v_k,a_k,j_k)\\
            其中k=x,y,z三轴\\
            J_k=\frac{1}{T}\int_0^Tj_k(t)^2dt
            $$

        2. 求解：对每个轴单独进行以下操作

            $$
            \lambda=(\lambda_1,\lambda_2,\lambda_3)\\
            H(s,u,\lambda)=\frac{1}{T}j^2+\lambda^Tf(s,u)\\=\frac{1}{T}j^2+\lambda_1 v+\lambda_2 a+\lambda_3 j\\
            \dot{\lambda}=-\nabla_sH(s^*,u^*,\lambda)=(0,\lambda_1 ,\lambda_2 )\\(对向量s求偏导就是对s中的元素分别求偏导，再组合形成向量，对上例即H分别对p,v,a求偏导)\\
            $$

            现在有如下方程组：

            $$
            \lambda=(\lambda_1,\lambda_2,\lambda_3)\\
            \dot{\lambda}=(0,-\lambda_1 ,-\lambda_2 )
            $$

            因此显然 λ 可以设为如下的形式：

            $$
            \lambda=\begin{pmatrix}
            \alpha\\-\alpha t-\beta\\\frac12\alpha t^2+\beta t+\gamma
            \end{pmatrix}^T
            $$

            因此最优控制量 u\*可以解出：

            $$
            u^*(t)=\arg\mathop{\min}\limits_{u(t)}H(s^*(t),u(t),\lambda(t))\\
            \because u(t)=j(t)\\
            \therefore u^*(t)=\arg\mathop{\min}\limits_{j(t)}H(s^*(t),j(t),\lambda(t))\\
            $$

            其中：

            $$
            H(s^*,j,\lambda)=\frac{1}{T}j^2+\lambda_1 v^*+\lambda_2 a^*+\lambda_3 j
            $$

            此时第二三项已经最小，使 H 最小的 j 的取值的求法即为：

            $$
            j^*=\arg\mathop{\min}\limits_{j}(\frac{1}{T}j^2+\lambda_3 j)
            $$

            因此，很容易求出：

            $$
            u^*=j^*=-\frac1T\lambda_3=-\frac1T(\frac12\alpha t^2+\beta t+\gamma)
            $$

            得到 j，通过积分很容易得到 a，v，p，即解出只与 α，β，γ 有关的 s\*：

            $$
            s^*=-\frac2T\begin{pmatrix}
            \frac{1}{120}\alpha t^5+\frac{1}{24}\beta t^4+\frac{1}{6}\gamma t^3+\frac{1}{2}a_0t^2+v_0t+p_0\\
            \frac{1}{24}\alpha t^4+\frac{1}{6}\beta t^3+\frac{1}{2}\gamma t^2+a_0t+v_0\\
            \frac{1}{6}\alpha t^3+\frac{1}{2}\beta t^2+\gamma t+a_0
            \end{pmatrix}^T\\
            s_0=(p_0,v_0,a_0)\ \ is\ \ given
            $$

            将末尾边界条件 sf 代入有：

            $$
            \because(T)=s_f=(p_f,v_f,a_f)\ \ is\ \ given\\
            \therefore-\frac2T
            \begin{bmatrix}
            \frac1{120}T^5&\frac1{24}T^4&\frac1{6}T^3\\
            \frac1{24}T^4&\frac1{6}T^3&\frac1{2}T^2\\
            \frac1{6}T^3&\frac1{2}T^2&T\\
            \end{bmatrix}
            \begin{bmatrix}
            \alpha \\ \beta \\ \gamma\\
            \end{bmatrix}=
            \begin{bmatrix}
            p_f-p_0-v_0T-\frac12a_0T^2\\v_f-v_0-a_0T\\a_f-a_0
            \end{bmatrix}\\
            \therefore
            \begin{bmatrix}
            \alpha \\ \beta \\ \gamma\\
            \end{bmatrix}=-\frac1{2T^4}
            \begin{bmatrix}
            720&-360&60T^2\\-360T&168T^2&-24T^3\\60T^2&-24T^3&3T^4
            \end{bmatrix}
            \begin{bmatrix}
            p_f-p_0-v_0T-\frac12a_0T^2\\v_f-v_0-a_0T\\a_f-a_0
            \end{bmatrix}\\
            $$

            此时 α，β，γ 只与 T 有关，即 s\*，u\*，J\*只与 T 有关

            - 可以给定 T，即可求出所需的 s, u
            - 或在 T 定义域内求极值，求出 J 使最小时的 T，再求出所需的 s, u

    3. 一些讨论

        1. 上例中不存在与终点有关的惩罚函数 h(s)，这是因为末端状态已经由 s(T)=sf 给出，此时的 h(s)可以认为是以下形式：

            $$
            h(s(t))=
            \begin{cases}
            0 & t=T\\\infty & t\ne T
            \end{cases}
            $$

            这个函数在 t=T 处不可导，因此不引入目标函数 J 中

        2. 可能存在 s 中某些分量的末状态给定，某些分量不给定的情况，例如：
            $$
            s(T)=(p_f,?,?)
            $$
            此时，目标函数 J 中就需要加入 h(s)，且存在边界条件
            $$
            given\ \ s_i(T)\\
            \lambda_j(T)=\frac{\partial h(s^*(T))}{\partial s_j},\ for\ j\ne i
            \\其中\ \ i+j=n
            $$
            可以想象由于 j 个分量没给出导致的 j 个自由度，全部由这 j 个新的方程约束

---

#### C. Comparison

![image.png](https://s2.loli.net/2023/02/27/mqURXfOkQFHMirT.png)

-   在道路环境下，Sample in control space 会出现一部分采样驶出路面，这就会导致可用的采样变少，即最优 path 只能从一个较小的集合中找出（离散度过大）
-   同质化问题：在 Sample in control space，当有一个障碍物使得某个 path 不可用时，其临近的多个 path 可能都不可用，这就是同质化问题，若多个 path 在拓扑环境中过于相似，这样的采样就是低效的（离散度过小）

---

#### D. Heuristic Function

![image.png](https://s2.loli.net/2023/03/01/vEpOFw4caqlz53m.png)

-   假设不考虑障碍物：即解两状态间的 OBVP 问题
-   假设不考虑动力学：绕过障碍物的直线最短路径
-   工程实际应用可能将两者求和，加权，或取最大值

![image.png](https://s2.loli.net/2023/03/01/TCHUd59blj7gpJ1.png)

---

#### E. Planning in Frenet-serret Frame

因为汽车是行驶在结构化的道路上的，辅以车道线检测的结果，可以把汽车坐标转换到 Frenet-Serret 坐标系下[(23 条消息) 弗莱纳公式（Frenet–Serret formulas）\_弗莱纳坐标系\_CA727 的博客-CSDN 博客](https://blog.csdn.net/cfan927/article/details/107233889)，并在此坐标系下进行一种特殊的 Lattice Planning，即法向 d(t)和切向 s(t)分别进行参数化，采样和解 OBVP

在论文*Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame,* Moritz Werling, Julius Ziegler, Sören Kammel, and Sebastian Thrun 和*Optimal trajectories for time-critical street scenarios using discretized terminal manifolds*, Moritz Werling, Sören Kammel, Julius Ziegler and Lutz Gröll 中将 d(t)和 s(t)都建模为**五次多项式**

![image.png](https://s2.loli.net/2023/03/01/6KrRStbCnXqFZl8.png)

例如对于变道任务，可以预想末状态法向的速度和加速度应该都为 0，那么就有

$$
D(T)=\begin{pmatrix}
d_T&0&0
\end{pmatrix}
$$

结合初状态就可以进行 OBVP 的求解

---

### （4）Hybrid A\* （基于离散控制量形成的 Lattice Graph）

#### 1. 基本思想

![image.png](https://s2.loli.net/2023/03/02/s92Ql7ixGb4tcpm.png)

离散控制量形成的 Lattice Graph 有如下两个问题：

1. 为了确保有可行解，往往将控制量离散为很多份，这样就会导致 Lattice Graph 非常稠密
2. 由于稠密，多条路径之间可能及其相似，即同质性，这样是非常低效的

故考虑对 Lattice Graph 进行剪枝（prune），剪枝的方法就是结合栅格地图的思想，即**每个栅格中只维护一个 Feasible Local Motion**

-   Q：当一个栅格中有多个 cost 时候，如何取舍？
-   A：维护从父节点到该节点的 cost 最小的一个（这里的 cost 不只是距离，而是综合定义的广义的 cost 函数的值）

#### 2. 算法流程

![image.png](https://s2.loli.net/2023/03/02/ibvJNuTREyKOZhk.png)

与传统 A\*的不同：

1. h(n)和 g(n)的设计不同
2. 找 neighbor 的方式不同：
    1. A\*是找八邻域的 nodes
    2. JPS 是找符合特殊跳点规则的 neighbor
    3. Hybrid A\*中的 neighbors 是：从父节点（motion）出发将离散的一系列控制量分别经过一次 forward simulate 得到的一系列 simulated feasible local motions
3. 需要维护每个网格中只有一个 feasible local motion：
    1. 若延申某个新的结点时发现其所在的网格中无 motion，则将该状态存在该网格中
    2. 若延申某个新的结点时发现其所在的网格中已有 motion，则分别比较他们从父节点到自身的 cost，保留更小的那个

#### 3. Heuristic Function Design

![image.png](https://s2.loli.net/2023/03/02/54oyFz9TepOURH8.png)

![image.png](https://s2.loli.net/2023/03/02/L8dzfvZ6MjnrOJ3.png)

（a）：2D-Euclidean Distance；（b）：non-holonomic-without-obstacles（即与终末状态求 OBVP）

这两者比较显然 non-holonomic-without-obstacles 更好，因为其更接近 H 的真实值

但是 non-holonomic-without-obstacles 在迷宫（多死胡同）中表现不佳，因为它没有障碍物信息，其天然的贪心性质会把机器人往目标方向引导（例如（c）图）

这是可以把 holonomic-with-obstacles（即 2D shortest path）纳入 H 函数（即每个位置解一下与目标点的最短路径（可以通过 A\*或 Dijkstra 求解）），这样采用 non-holonomic-without-obstacles + holonomic-with-obstacles 或 max（non-holonomic-without-obstacles，holonomic-with-obstacles）作为 H 函数

#### 4. 工程技巧 One-Shot

在 Exploration 的过程中，不断尝试解当前状态到目标状态的 OBVP，如果出现解无碰撞，则可以停止探索，直接采用该解（OBVP 无法考虑障碍物信息，因此解 OBVP 总是有解，但很大困难会与障碍物碰撞）

![image.png](https://s2.loli.net/2023/03/02/CwDqEG18XJyPBQc.png)

在 Exploration 初期，找到这样一个 One-Shot 解的概率较小，因此可以设置一个常数 N，即每扩展 N 次，尝试一次求 One-Shot 解，随着与目标点距离越来越近，找到 One-Shot 解的概率会变大，这时可以减小 N，这样会更频繁的尝试求 One-Shot 解

---

### （5）Kinodynamic RRT\*（基于离散状态量形成的 Lattice Graph）

![image.png](https://s2.loli.net/2023/02/20/lY6XNEtZc5OkibS.png)

Kinodynamic RRT\*与 RRT\*的不同

1. Sample：RRT\*在 x,y 二维空间中采样，K-RRT\*在 n 维空间中采样（s 向量的维度为 n）

2. 如何确定邻域：【==关于如何求 reachable set 已经有很多成熟的算法==】

    n 维空间中一个点，以及定义好的 cost 函数，cost≤r 的点的集合叫做 reachable set

    1. backward-reachable set：以该点为终点，即以高维空间中任意点为起点，以小于 r 的 cost 能够到达该点的所有点的集合
    2. forward-reachable set：以该点为起点，即高维空间中以该点为起点，以小于 r 的 cost 能到达的所有点的集合

3. ChooseParents：找到$x_{new}$后，求其 backward-reachable set，在其中找到 cost 最小的作为 Parent

4. Rewire：求出$x_{new}$的 forward-reachable set，逐一检查他们目前$g(n)$和$g(x_{new})+cost(x_{new},x_i)$的大小，维护较小的
