# 移动机器人

## 1.导论

### (1). Perception-Planning-Control action loop

![image-20230131142448377](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131142448377.png)

- Estimation: (location & status)
  - Low latency
  - High accuracy & consistency
- Perception:
  - Sensing
  - Map fusion & integration
- Planning
  - Safety
  - Dynamical feasibility
- Control



### (2). Motion Planning

- Requirements
  - Safety
  - Smoothness
  - Kinodynamic feasibility
- Pipeline
  - Front-end: **Path** finding
    - Low dimensional
    - Discrete space
  - Back-end: **Trajectory** generation
    - High dimensional
    - Continuous space



### (3). Front-end: Path Finding

#### ① Search-based Path Finding (Graph Search)

##### a. Dijkstra

![image-20230131144745325](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131144745325.png)

##### b. A*

##### c. Jump Point Search (JPS)

![image-20230131144717903](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131144717903.png)

#### ② Sampling-based Path Finding (Probabilistic Road Map)

##### a. Rapidly-exploring Random Tree (RRT)

##### b. Optimal Sampling-based Methods (RRT*)

![image-20230131144833927](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131144833927.png)

##### c. Advanced Sampling-based Methods

| <img src="C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131144933245.png" alt="image-20230131144933245" style="zoom:50%;" /> | <img src="C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131144950046.png" alt="image-20230131144950046" style="zoom:50%;" /> |
| ------------------------------------------------------------ | ------------------------------------------------------------ |



#### ③ Kinodynamic Path Finding (Search-based or Sampling-based)

##### a. Basis: State-state Boundary Value Optimal Control Problem

##### b. State Lattice Search (离散化控制量生成Graph + Graph Search in high dimensions)

![image-20230131145202560](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131145202560.png)

##### c.  Kinodynamic RRT* （随机撒高位状态点？离散化状态量）

![image-20230131145435400](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131145435400.png)

##### d. Hybrid A* (每一个栅格里只维护一个状态，每个栅格里的状态总是保证路径的总代价最小)

![image-20230131145321491](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131145321491.png)



### (4). Back-end: Trajectory Generation

#### ① Minimum Snap Trajectory Generation （从折线路径到动力学光滑轨迹？）

![image-20230131145709876](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131145709876.png)

#### ② Soft and Hard Constrained Trajectory Optimization

| Hard                                                         | Soft                                                         |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image-20230131145843188](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131145843188.png) | ![image-20230131150004826](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131150004826.png) |
| 矩形块标示可通行区域（凸的解空间），通过硬性约束生成路径，路径保持在解空间里 | 障碍物给出梯度距离场（排斥力）                               |



### (5). MDP & MPC

#### ① Markov Decision Progress-based Planning

#### ② Model Predictive Control for Robotics Planning



### (6). Map

#### ① Occupancy Grid Map https://github.com/ANYbotics/grid_map

- Most dense
- Structural
- Direct Index Query

后验概率 （建立概率栅格地图）：![image-20230131160948616](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131160948616.png)

贝叶斯滤波更新概率栅格地图：

![image-20230131171406059](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131171406059.png)

![image-20230131171434457](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131171434457.png)

![image-20230131171459005](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131171459005.png)

初始化l0=0，每一次观测都更新lt，设置一个阈值s，当lt>s时就认为该grid被占据

#### ② Octo-map (八叉树) https://octomap.github.io/

- Sparse
- Structural
- Direct Index Query (间接的，通过树的结构递归的查询)

![image-20230131150521551](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131150521551.png)

#### ③ Voxel Hashing (体素哈希表)

Map -> Bucket -> Blocks -> Voxel

- Most sparse
- Structural
- Indirect Index Query

![image-20230131154108679](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131154108679.png)

#### ④ Point Cloud Map (墙裂推荐PCL库：http://pointclouds.org/)

- Unordered
- No Index Query

#### ⑤ Truncated Signed Distance Functions (TSDF) Map (https://github.com/personalrobotics/OpenChisel) [截断距离场]

![image-20230131154649073](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131154649073.png)

#### ⑥ Euclidean Signed Distance Functions Incremental Update, Global (ESDF) Map [欧式符号距离场] （VoxBlox: https://github.com/ethz-asl/voxblox; FIESTA: https://github.com/HKUST-Aerial-Robotics/FIESTA）

![image-20230131154936540](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131154936540.png)

- 和占据栅格地图对比，ESDF存在负值（障碍表面内部为负值，值的大小为到最近的free的栅格的距离，而free区域内存放的是正值）

![image-20230131171950312](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131171950312.png)

- 建立方法
  - 从栅格地图增量
  - 从TSDF增量
  - 从栅格地图分批滚动建立（节省内存）

![image-20230131172344056](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131172344056.png)

- 一维情况（下包络线：到最近障碍物的距离）

![image-20230131200808470](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131200808470.png)

- 高维情况 [**个人理解高维情况可以做抛物面取最小包络面**]![image-20230131204600761](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131204600761.png)

![image-20230131200923337](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131200923337.png)

#### ⑦ 拓扑地图，地图骨架（用于大型地图下的规划问题）![image-20230131155420234](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131155420234.png)



## 2.基于搜索的路径规划

### (1). Graph Search Basis 

#### ① Robot Configuration Space

- Robot Configuration: 机器人的一个位姿(配置)
- Robot Degree Of Freedom (DOF): 用于描述机器人位姿的向量的维度n
- Robot Configuration Space: n维向量空间，包含机器人所有可能的位姿

![image-20230131144305379](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131144305379.png)

![image-20230131144402744](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131144402744.png)



#### ② Graphs Search

- Graphs（包含nodes和edges）

![image-20230131144658213](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131144658213.png)

- Search Tree

![image-20230131144934003](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131144934003.png)

- **Overview**

  - A container: to store all the nodes to be visited (initialized with the start state)

  - Loop:

    - 根据给定的score function从container中**remove**出一个node，意为visit该node
    - **expand**该node的所有neighbors
    - **push**这些neighbors到container中

  - 终止条件：container中无node，或visit到end node

  - if the graph is cyclic：

    - 维护一个close list，放置已经visit过的nodes，这些nodes不会再被visit

  - score function如何定义，以更快的（用更少次的expansion）到达end node

    

- Breadth First Search (BFS)

  - 其container是一个queue，遵循先进先出原则

    ![image-20230131233636919](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131233636919.png)
    
    ```mermaid
    graph TB
    a[a:onset] --> b
    a --> c
    a --> d
    b --> e
    b --> f
    b --> g
    c --> h
    c --> i
    d --> j
    e --> k
    f --> l
    g --> m
    g --> n
    n --> o[o:end]
    ```
    
    | 当前container | 当前visit的node | 当前container | expand到的node |
    | ------------- | --------------- | ------------- | -------------- |
    | a             | a               | nan           | b,c,d          |
    | b,c,d         | d               | b,c           | j              |
    | j,b,c         | c               | j,b           | h,i            |
    | h,i,j,b       | b               | h,i,j         | e,f,g          |
    | e,f,g,h,i,j   | j               | e,f,g,h,i     | nan            |
    | e,f,g,h,i     | i               | e,f,g,h       | nan            |
    | e,f,g,h       | h               | e,f,g         | nan            |
    | e,f,g         | g               | e,f           | m,n            |
    | m,n,e,f       | f               | m,n,e         | l              |
    | l,m,n,e       | e               | l,m,n         | k              |
    | k,l,m,n       | n               | k,l,m         | o              |
    
    - expand的node从左压入，visit时从右取出
    
    

- Depth First Search（DFS）

  - 其container是一个stack，遵循后进先出原则

    ![image-20230131233715105](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230131233715105.png)

  - 同一深度层级的节点压入的顺序需要自己定义 (这个定义有可能能极大的节省计算量)

    ```mermaid
    graph TB
    a[a:onset] --> b
    a --> c
    a --> d
    b --> e
    b --> f
    b --> g
    c --> h
    c --> i
    d --> j
    e --> k
    f --> l
    g --> m
    g --> n
    n --> o[o:end]
    
    ```

    | 当前container（开口向左） | 当前visit的node | 当前container（开口向左） | expand到的node |
    | ------------------------- | --------------- | ------------------------- | -------------- |
    | a                         | a               | nan                       | b,c,d          |
    | b,c,d                     | b               | c,d                       | e,f,g          |
    | e,f,g,c,d                 | e               | f,g,c,d                   | k              |
    | k,f,g,c,d                 | k               | f,g,c,d                   | nan            |
    | f,g,c,d                   | f               | g,c,d                     | l              |
    | l,g,c,d                   | l               | g,c,d                     | nan            |
    | g,c,d                     | g               | c,d                       | m,n            |
    | m,n,c,d                   | m               | n,c,d                     | nan            |
    | n,c,d                     | n               | c,d                       | o              |

    - expand的node从左压入，visit时从左取出

    

- BFS vs. DFS

![image-20230201160959202](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230201160959202.png)



- Heuristic Search (Greedy Best First Search，贪心算法)
  - expand到的node正常压入
  - visit node时，根据一个启发函数选择一个最优的node
  - 该启发函数通常为当前节点到目标节点的距离（欧式距离Euclidean Distance或曼哈顿距离Manhattan Distance）
  - ![image-20230201161452713](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230201161452713.png)
  - 贪心算法易陷入局部最优，因为启发函数在计算时是忽略了障碍物的
  - ![image-20230201161549877](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230201161549877.png)



### (2). Dijkstra Algorithm

#### ① visit策略：

- visit the node with the cheapest accumulated cost **g(n)**
  - g(n)：从起点到当前node累积cost的和
  - 在expand时，检查所有neighbors现有的cost值，例如对于当前访问node n的neighbor m，若g(n)+cost(n->m)<g(m)，则把g(m)更新为g(n)+cost(n->m)
  - 最优性保证：所有被visited过的nodes中所储存的g值总是从起点到该点的最小cost

#### ② container: 

- a priority queue（根据g值排序）
  - container = open list
  - close list：用于存放已经扩展过的nodes，这些nodes不会再被visit

#### ③ 初始化：

- container：起点Xs；
- g(Xs)=0
- 其他所有点g(n)=inf

#### ④ 伪代码：

- ![image-20230202160545230](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202160545230.png)

⑤ 示例：

- ![image-20230202160907613](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202160907613.png)

- | 当前容器(根据g值自动排序，小的先弹出) | 当前访问节点 | 当前容器               | expand到的node    | g(S) | g(a) | g(b) | g(c) | g(d) | g(e) | g(f) | g(h) | g(p) | g(q) | g(r) | g(G) |
  | ------------------------------------- | ------------ | ---------------------- | ----------------- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
  |                                       |              |                        |                   | 0    | inf  | inf  | inf  | inf  | inf  | inf  | inf  | inf  | inf  | inf  | inf  |
  | S(0)                                  | S(0)         | nan                    | e(+9),d(+3),p(+1) | 0    | inf  | inf  | inf  | 3    | 9    | inf  | inf  | 1    | inf  | inf  | inf  |
  | e(9),d(3),p(1)                        | p(1)         | e(9),d(3)              | q(+15)            | 0    | inf  | inf  | inf  | 3    | 9    | inf  | inf  | 1    | 16   | inf  | inf  |
  | q(16),e(9),d(3)                       | d(3)         | q(16),e(9)             | b(+1),c(+8),e(+2) | 0    | inf  | 4    | 11   | 3    | 5    | inf  | inf  | 1    | 16   | inf  | inf  |
  | q(16),c(11),e(5),b(4)                 | b(4)         | q(16),c(11),e(5)       | a(+2)             | 0    | 6    | 4    | 11   | 3    | 5    | inf  | inf  | 1    | 16   | inf  | inf  |
  | q(16),c(11),a(6),e(5)                 | e(5)         | q(16),c(11),a(6)       | r(+1),h(+8)       | 0    | 6    | 4    | 11   | 3    | 5    | inf  | 14   | 1    | 16   | 6    | inf  |
  | q(16),h(14),c(11),a(6),r(6)           | r(6)         | q(16),h(14),c(11),a(6) | f(+1)             | 0    | 6    | 4    | 11   | 3    | 5    | 7    | 14   | 1    | 16   | 6    | inf  |
  | q(16),h(14),c(11),f(7),a(6)           | a(6)         | q(16),h(14),c(11),f(7) | nan               | 0    | 6    | 4    | 11   | 3    | 5    | 7    | 14   | 1    | 16   | 6    | inf  |
  | q(16),h(14),c(11),f(7)                | f(7)         | q(16),h(14),c(11)      | c(+0),G(+2)       | 0    | 6    | 4    | 7    | 3    | 5    | 7    | 14   | 1    | 16   | 6    | 9    |
  | q(16),h(14),c(11),G(9),c(7)           | c(7)         | q(16),h(14),c(11),G(9) | a(+0)             | 0    | 6    | 4    | 7    | 3    | 5    | 7    | 14   | 1    | 16   | 6    | 9    |
  | q(16),h(14),c(11),G(9)                | G(9)         |                        |                   |      |      |      |      |      |      |      |      |      |      |      |      |

#### ⑥ 优缺点：

- 优点：一定能找到最优解
- 缺点：不包含终点信息，各个方向均匀扩散，效率低



### (3). A*：Dijkstra with a Heuristic

#### ① visit策略：

- visit the node with cheapest f(n) = g(n) + h(n)
  - g(n): 当前node到起点的累积最小cost
  - h(n): 当前node到终点的估计最小cost
  - 其他和Dijkstra完全一样

#### ② 伪代码：

- ![image-20230202172915788](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202172915788.png)

#### ③ 示例：

- ![image-20230202173331243](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202173331243.png)

#### ④ 最优性保证：任意一个node估计的h(n)的必须小于该点到终点实际的cost

- h(n)设计：
  - Euclidean Distance：always less -> √
  - Manhattan Distance：depends -> ? （当机器人只能横纵向移动不能斜行时是最优）
- 最优性 vs 速度：
- ![image-20230202174320194](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202174320194.png)
- ![image-20230202174524147](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202174524147.png)



#### ⑥ A*在工程中的注意事项：

##### a.如何把grid map变为graph（四联通or八联通）

- ![image-20230202174653537](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202174653537.png)

##### b.priority queue的实现

- ![image-20230202174815666](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202174815666.png)

##### c.启发函数的优化：

- ![image-20230202175129187](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202175129187.png)
- 欧式距离所估计的h(n)虽然符合h(n)≤h*(n)，但相差很多(不tight/远小于)，导致扩展过程中会2扩展到很多无用的nodes
- 解决方案：Diagonal Heuristic
- ![image-20230202175645786](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202175645786.png)

##### d.Tie Breaker

- 当任何一步的扩展中遇到f值相同的nodes时，若无操作，则会根据先后顺序排列，这些nodes可能全部得到expand，这将导致扩展很多不必要的nodes
- 通过一个操作使对这些nodes的选择不完全随机，而是有倾向性
- 当两个node的f值相同时，将其中一个放大一个很小的值
- ![image-20230202180717905](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202180717905.png)
- Tie Breaker的其他方法：
  - 当f相同时对h进行排序比较
  - 给h加上根据事先建立好的随机数表（坐标的哈希表）
  - 根据想要的加过对h进行修改（例如想要走对角线，则加一个cross cost）
  - ![image-20230202181252052](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202181252052.png)
- Tie Breaker对后续Trajectory generation的影响
  - ![image-20230202181436455](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230202181436455.png)

### (4). JPS: Jump Point Search

- 核心思想：
  - 和A*只是expand neighbors的方式不一样：
    - A* expand当前node几何上的所有邻居
    - JPS根据Look Ahead Rule和Jumping Rule来将符合条件的跳跃点加入open list
  - neighbors加入open list后依然是根据f(n)=g(n)+h(n)最小的priority queue弹出node进行visit

![image-20230204181942515](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204181942515.png)

#### ① Look Ahead Rule

 对当前visit的node X的所有可能expand的neighbors nodes而言，如果从父节点不经过X到该点的cost小于从父节点经过X到该点的cost，则为inferior neighbors，否则为natural neighbors，在expand时只考虑natural neighbors，不考虑inferior neighbors

- 直行时判断条件为**小于等于**；

![image-20230204183442710](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204183442710.png)

- eg. 当前节点为x，父节点为4：

| neighbor node | cost without x | cost with x    | result      |
| ------------- | -------------- | -------------- | ----------- |
| 1             | 4->1   1       | 4->x->1   √2   | inferior    |
| 2             | 4->2   √2      | 4->x->2   2    | inferior    |
| 3             | 4->2->3   1+√2 | 4->x->3   1+√2 | inferior    |
| 5             | 4->2->5   2√2  | 4->x->5   2    | **natural** |

- 对角线行驶时判断条件为**小于**；

![image-20230204183455174](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204183455174.png)

- eg. 当前节点为x，父节点为6：

| neighbor node | cost without x    | cost with x    | result      |
| ------------- | ----------------- | -------------- | ----------- |
| 4             | 6->4   1          | 6->x->4   √2   | inferior    |
| 1             | 6->4->1   2       | 6->x->1   2√2  | inferior    |
| 2             | 6->4->2   1+√2    | 6->x->2   1+√2 | **natural** |
| 3             | 6->4->2->3   2+√2 | 6->x->3   2√2  | **natural** |

- 特殊情况：forced neighbor（直行时或斜行时旁边紧邻有障碍物（如下图））
  - 当直行的2 or 7号位有障碍物时，则需要额外将3 or 8号位置为natural neighbor
  - 当斜行的4 or 7号位有障碍物时，则需要额外将1 or 8号位置为natural neighbor
  - ![image-20230204184243891](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204184243891.png)

#### ② Jumping Rules

- 对每一个node，优先水平方向和竖直方向expand：
  - 如果expand到具有forced neighbor的特殊点时，将该node加入open list（注意不是将这个特殊点加入了open list）
  - 如果水平竖直方向都没有找到特殊node，则沿斜向延申一个node，继续沿水平，竖直方向搜索特殊点
- 下图中蓝色点加入了open list
- 绿色点所有可能延申的方向都得到了充分的延申，则绿色点可以加入close list

| ![image-20230204225645774](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204225645774.png) | ![image-20230204225621052](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204225621052.png) | ![image-20230204225712347](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204225712347.png) | ![image-20230204225732994](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204225732994.png) | ![image-20230204225751267](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204225751267.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |

#### ③ Example1

| ![image-20230204230610459](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204230610459.png) | ![image-20230204230649208](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204230649208.png) | ![image-20230204230913511](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204230913511.png) | ![image-20230204230932481](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204230932481.png) | ![image-20230204231046282](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204231046282.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |

- 绿色点横向，纵向搜索 =》 无结果
- 绿色点斜向延申一格后横向，纵向搜索 =》 无结果
- 绿色点斜向延申直到横向，纵向搜索能找到特殊点 =》 找到黄色点（黄色点具有forced neighbor青色点）=》 黄色点加入open list
- 绿色点继续斜向延申 =》无结果 =》 绿色点加入close list
- 此时open list只有黄色点 =》弹出黄色点
- 根据Look Ahead Rule，斜行的点可以扩展横，纵，斜三个方向，其中纵和斜均无结果，横向找到特殊点青色点（青色点具有forced neighbor 蓝色点），青色点加入open list，黄色点加入close list
- 此时open list只有青色=》弹出青色点
- 根据Look Ahead Rule，横行的特殊点可能的expand方向有横行和斜下行，其中横行未找到特殊点，斜下行时找到青色点（青色点横纵向能延伸到终点），将青色点加入open list
- 弹出青色点
- 优先横纵向搜索，在纵向上找到终点

#### ④ Example2

| ![image-20230204234830922](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204234830922.png) | ![image-20230204234839341](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204234839341.png) | ![image-20230204234847606](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204234847606.png) | ![image-20230204234601366](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204234601366.png) | ![image-20230204234619456](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204234619456.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |

| ![image-20230204234715198](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204234715198.png) | ![image-20230204234725676](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204234725676.png) | ![image-20230204234736425](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204234736425.png) | ![image-20230204234745234](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204234745234.png) | ![image-20230204234755341](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204234755341.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |

- 这个例子充分的体现出了：
  1. 每个节点所有可能的延申方向得到充分的延申才会加入close list（图中变灰色的节点）
  2. 从起点expand到了两个可能的neighbor加入了open list，但是由于终点的位置不同，所走的路线不同，这是因为open list在弹出节点时和A*一样，是根据f(n)=g(n)+h(n)最优来排序的

#### ⑤ Analysis

- 在复杂的迷宫环境中JPS>A*
- 在开阔的，障碍物少的环境中JPS<A*（JPS需要延申大面积去确认没有特殊点）

![image-20230204235325509](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230204235325509.png)







