# 移动机器人

## 1.导论

### (1). Perception-Planning-Control action loop

![image.png](https://s2.loli.net/2023/02/20/2Dkn47Q3sXjodAF.png)

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

![image.png](https://s2.loli.net/2023/02/20/vguQGet16NcKbn9.png)

##### b. A*

##### c. Jump Point Search (JPS)

![image.png](https://s2.loli.net/2023/02/20/dfgcOvkeQSHaJF1.png)

#### ② Sampling-based Path Finding (Probabilistic Road Map)

##### a. Rapidly-exploring Random Tree (RRT)

##### b. Optimal Sampling-based Methods (RRT*)

![image.png](https://s2.loli.net/2023/02/20/PCbJXExs3q9Atd1.png)

##### c. Advanced Sampling-based Methods

![image.png](https://s2.loli.net/2023/02/20/GXV7gxLnJYcMTpO.png)

#### ③ Kinodynamic Path Finding (Search-based or Sampling-based)

##### a. Basis: State-state Boundary Value Optimal Control Problem

##### b. State Lattice Search (离散化控制量生成Graph + Graph Search in high dimensions)

![image.png](https://s2.loli.net/2023/02/20/a5YwXGiL9CAk6eQ.png)

##### c.  Kinodynamic RRT* （随机撒高位状态点？离散化状态量）

![image.png](https://s2.loli.net/2023/02/20/gBtMpPZO7EYqDi8.png)

##### d. Hybrid A* (每一个栅格里只维护一个状态，每个栅格里的状态总是保证路径的总代价最小)

![image.png](https://s2.loli.net/2023/02/20/fGMt36hLz7duCNX.png)



### (4). Back-end: Trajectory Generation

#### ① Minimum Snap Trajectory Generation （从折线路径到动力学光滑轨迹？）

![image.png](https://s2.loli.net/2023/02/20/RiVq32kQ7ZtpxMP.png)

#### ② Soft and Hard Constrained Trajectory Optimization

![image.png](https://s2.loli.net/2023/02/20/3J8Z9b2O6EkqrAn.png)

### (5). MDP & MPC

#### ① Markov Decision Progress-based Planning

#### ② Model Predictive Control for Robotics Planning



### (6). Map

#### ① Occupancy Grid Map https://github.com/ANYbotics/grid_map

- Most dense
- Structural
- Direct Index Query

后验概率 （建立概率栅格地图）：![image.png](https://s2.loli.net/2023/02/20/1CsZfbwXgUO6Ghr.png)

贝叶斯滤波更新概率栅格地图：

![image.png](https://s2.loli.net/2023/02/20/TOsjHU8EqZdo6FS.png)

![image.png](https://s2.loli.net/2023/02/20/6otxLRTYSJAMwZX.png)

![image.png](https://s2.loli.net/2023/02/20/2MqPTS5CHhyasp1.png)

初始化l0=0，每一次观测都更新lt，设置一个阈值s，当lt>s时就认为该grid被占据

#### ② Octo-map (八叉树) https://octomap.github.io/

- Sparse
- Structural
- Direct Index Query (间接的，通过树的结构递归的查询)

![image.png](https://s2.loli.net/2023/02/20/agCyeQ9PXY6OEz7.png)

#### ③ Voxel Hashing (体素哈希表)

Map -> Bucket -> Blocks -> Voxel

- Most sparse
- Structural
- Indirect Index Query

![image.png](https://s2.loli.net/2023/02/20/3xBPyGIqvU5j2Ss.png)

#### ④ Point Cloud Map (墙裂推荐PCL库：http://pointclouds.org/)

- Unordered
- No Index Query

#### ⑤ Truncated Signed Distance Functions (TSDF) Map (https://github.com/personalrobotics/OpenChisel) [截断距离场]

![image.png](https://s2.loli.net/2023/02/20/N7mpz3AesZ9WnL1.png)

#### ⑥ Euclidean Signed Distance Functions Incremental Update, Global (ESDF) Map [欧式符号距离场] （VoxBlox: https://github.com/ethz-asl/voxblox; FIESTA: https://github.com/HKUST-Aerial-Robotics/FIESTA）

![image.png](https://s2.loli.net/2023/02/20/EBtQxRACiXhvG31.png)

- 和占据栅格地图对比，ESDF存在负值（障碍表面内部为负值，值的大小为到最近的free的栅格的距离，而free区域内存放的是正值）

![image.png](https://s2.loli.net/2023/02/20/sWBC1uwrkxlF3id.png)

- 建立方法
  - 从栅格地图增量
  - 从TSDF增量
  - 从栅格地图分批滚动建立（节省内存）

![image.png](https://s2.loli.net/2023/02/20/g6ZInVUEMzAusqF.png)

- 一维情况（下包络线：到最近障碍物的距离）

![image.png](https://s2.loli.net/2023/02/20/OwjJuCDeVfXKn7l.png)

- 高维情况 [**个人理解高维情况可以做抛物面取最小包络面**]![image.png](https://s2.loli.net/2023/02/20/ZsDME9WpBVmJqaU.png)

![image.png](https://s2.loli.net/2023/02/20/AMaoL1fQBkECKIF.png)

#### ⑦ 拓扑地图，地图骨架（用于大型地图下的规划问题）

![image.png](https://s2.loli.net/2023/02/20/1H2LFnOzqdIpw53.png)



## 2.基于搜索的路径规划

### (1). Graph Search Basis 

#### ① Robot Configuration Space

- Robot Configuration: 机器人的一个位姿(配置)
- Robot Degree Of Freedom (DOF): 用于描述机器人位姿的向量的维度n
- Robot Configuration Space: n维向量空间，包含机器人所有可能的位姿

![image.png](https://s2.loli.net/2023/02/20/xzMpJLSkPo5RDbi.png)

![image.png](https://s2.loli.net/2023/02/20/9rVtJb3clqogvfF.png)



#### ② Graphs Search

- Graphs（包含nodes和edges）

![image.png](https://s2.loli.net/2023/02/20/s4azdrxfg6n9YVc.png)

- Search Tree

![image.png](https://s2.loli.net/2023/02/20/E4IdO1c6pPWzYQJ.png)

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

    ![image.png](https://s2.loli.net/2023/02/20/RDLQ7g5OUYqdnKF.png)
    
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

    ![image.png](https://s2.loli.net/2023/02/20/mn14KOtMi2kH7hD.png)

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

![image.png](https://s2.loli.net/2023/02/20/nyOIKgeNvsH7FrQ.png)



- Heuristic Search (Greedy Best First Search，贪心算法)
  - expand到的node正常压入
  - visit node时，根据一个启发函数选择一个最优的node
  - 该启发函数通常为当前节点到目标节点的距离（欧式距离Euclidean Distance或曼哈顿距离Manhattan Distance）
  - ![image.png](https://s2.loli.net/2023/02/20/Mr3sQcl8RLGofV7.png)
  - 贪心算法易陷入局部最优，因为启发函数在计算时是忽略了障碍物的
  - ![image.png](https://s2.loli.net/2023/02/20/pkYND1OrnUc9gja.png)



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

- ![image.png](https://s2.loli.net/2023/02/20/sHSnIqYG4Lk3ydr.png)

⑤ 示例：

- ![image.png](https://s2.loli.net/2023/02/20/fCmVk4TP1oOAR5b.png)

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

- ![image.png](https://s2.loli.net/2023/02/20/JL26jNQZaWclgeA.png)

#### ③ 示例：

- ![image.png](https://s2.loli.net/2023/02/20/gsjWVJ6M7lDkfIR.png)

#### ④ 最优性保证：任意一个node估计的h(n)的必须小于该点到终点实际的cost

- h(n)设计：
  - Euclidean Distance：always less -> √
  - Manhattan Distance：depends -> ? （当机器人只能横纵向移动不能斜行时是最优）
- 最优性 vs 速度：
- ![image.png](https://s2.loli.net/2023/02/20/BwNQY2X9vbynD8W.png)
- ![image.png](https://s2.loli.net/2023/02/20/JXwZkiWUG8fnzYE.png)



#### ⑥ A*在工程中的注意事项：

##### a.如何把grid map变为graph（四联通or八联通）

- ![image.png](https://s2.loli.net/2023/02/20/u6tkzK5rqG2NxQX.png)

##### b.priority queue的实现

- ![image.png](https://s2.loli.net/2023/02/20/qGy87fXmoYkEMbl.png)

##### c.启发函数的优化：

- ![image.png](https://s2.loli.net/2023/02/20/FLwHuaTW6kZD9qA.png)
- 欧式距离所估计的h(n)虽然符合h(n)≤h*(n)，但相差很多(不tight/远小于)，导致扩展过程中会2扩展到很多无用的nodes
- 解决方案：Diagonal Heuristic
- ![image.png](https://s2.loli.net/2023/02/20/5sSeAotQVNBwXJ2.png)

##### d.Tie Breaker

- 当任何一步的扩展中遇到f值相同的nodes时，若无操作，则会根据先后顺序排列，这些nodes可能全部得到expand，这将导致扩展很多不必要的nodes
- 通过一个操作使对这些nodes的选择不完全随机，而是有倾向性
- 当两个node的f值相同时，将其中一个放大一个很小的值
- ![image.png](https://s2.loli.net/2023/02/20/eZd4oysKAWIG1lT.png)
- Tie Breaker的其他方法：
  - 当f相同时对h进行排序比较
  - 给h加上根据事先建立好的随机数表（坐标的哈希表）
  - 根据想要的加过对h进行修改（例如想要走对角线，则加一个cross cost）
  - ![image.png](https://s2.loli.net/2023/02/20/qXt6whgST2odyUQ.png)
- Tie Breaker对后续Trajectory generation的影响
  - ![image.png](https://s2.loli.net/2023/02/20/cbjZ9XTea6v52Kh.png)

### (4). JPS: Jump Point Search

- 核心思想：
  - 和A*只是expand neighbors的方式不一样：
    - A* expand当前node几何上的所有邻居
    - JPS根据Look Ahead Rule和Jumping Rule来将符合条件的跳跃点加入open list
  - neighbors加入open list后依然是根据f(n)=g(n)+h(n)最小的priority queue弹出node进行visit

![image.png](https://s2.loli.net/2023/02/20/naRwNBT1P5K7HY9.png)

#### ① Look Ahead Rule

 对当前visit的node X的所有可能expand的neighbors nodes而言，如果从父节点不经过X到该点的cost小于从父节点经过X到该点的cost，则为inferior neighbors，否则为natural neighbors，在expand时只考虑natural neighbors，不考虑inferior neighbors

- 直行时判断条件为**小于等于**；

![image.png](https://s2.loli.net/2023/02/20/XaBKoezuPLqg1Gv.png)

- eg. 当前节点为x，父节点为4：

| neighbor node | cost without x | cost with x    | result      |
| ------------- | -------------- | -------------- | ----------- |
| 1             | 4->1   1       | 4->x->1   √2   | inferior    |
| 2             | 4->2   √2      | 4->x->2   2    | inferior    |
| 3             | 4->2->3   1+√2 | 4->x->3   1+√2 | inferior    |
| 5             | 4->2->5   2√2  | 4->x->5   2    | **natural** |

- 对角线行驶时判断条件为**小于**；

![image.png](https://s2.loli.net/2023/02/20/V2JdvMsw6yGrpET.png)

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
  - ![image.png](https://s2.loli.net/2023/02/20/ERN2flonOkSYMBz.png)

#### ② Jumping Rules

- 对每一个node，优先水平方向和竖直方向expand：
  - 如果expand到具有forced neighbor的特殊点时，将该node加入open list（注意不是将这个特殊点加入了open list）
  - 如果水平竖直方向都没有找到特殊node，则沿斜向延申一个node，继续沿水平，竖直方向搜索特殊点
- 下图中蓝色点加入了open list
- 绿色点所有可能延申的方向都得到了充分的延申，则绿色点可以加入close list

![image.png](https://s2.loli.net/2023/02/20/WniCJyYs4FV7Sl2.png)

#### ③ Example1

![image.png](https://s2.loli.net/2023/02/20/eRpWtGr2P3nKJj8.png)

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

![image.png](https://s2.loli.net/2023/02/20/qTFGe3M9XvzbAdS.png)

- 这个例子充分的体现出了：
  1. 每个节点所有可能的延申方向得到充分的延申才会加入close list（图中变灰色的节点）
  2. 从起点expand到了两个可能的neighbor加入了open list，但是由于终点的位置不同，所走的路线不同，这是因为open list在弹出节点时和A*一样，是根据f(n)=g(n)+h(n)最优来排序的

#### ⑤ Analysis

- 在复杂的迷宫环境中JPS>A*
- 在开阔的，障碍物少的环境中JPS<A*（JPS需要延申大面积去确认没有特殊点）

![image.png](https://s2.loli.net/2023/02/20/tykONQCvPLWA8zi.png)

---

## 3.基于采样的路径规划

Visualizations：https://github.com/ZJU-FAST-Lab/sampling-based-path-finding

### （1）基本思想

- 探索解空间的连续性以获得可行的或最优的解
- 方法：在连续的构型空间（Configuration Space）中采样出离散的构型空间样本，来构造树或图的结构来表征解空间的连续性
  - 采样可以批次的进行或增量式的进行
  - 算法的性能很大程度上取决于采样的策略（碰撞检测的策略）和最近邻搜索的效率
- 两个基本任务：
  - **exploration**：探索解空间拓扑连接性的信息【生长树，使之尽可能充满空间】
  - **exploitation**：增量式的提升（迭代，优化）解【修改树的结构】
- 术语：
  - Probabilistic Completeness：概率完备
  - Asymptotical Optimality：渐进最优
  - Anytime：即时性

---

### （2）Feasible Path Planning Methods

#### A. Probabilitistic Road Map（PRM）

- 算法流程

| 1.采样n个点                                                  | 2.删除碰撞的点                                               | 3.连接邻域的点                                               | 4.删除有碰撞的连接                                           | 5.搜索该图得到path                                           |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image.png](https://s2.loli.net/2023/02/20/tfv4xoRJW89Yr5h.png) | ![image.png](https://s2.loli.net/2023/02/20/xGVfQjpk3elXBNT.png) | ![image.png](https://s2.loli.net/2023/02/20/UwfYO5qFSpDyG4N.png) | ![image.png](https://s2.loli.net/2023/02/20/3FnDC6eKtVEsmZW.png) | ![image.png](https://s2.loli.net/2023/02/20/c1b8JUy4aFGRm3V.png) |

- 1~4成为**Learning Phase**；5称为**Query Phase**；
- 上述流程效率低下，主要原因有：
  - 没有引入起终点信息，因此可以做到任选起终点的Multi Query --> 希望得到特定解，而不是具有整个空间联动性的图 -->RRT的基本思想
  - 做了多余的碰撞检测，即不在路径上的碰撞的检测 -->在高维地图中碰撞检测比较耗时 --> Lazy Collision Checking
    - 采样n个点，连接所有边，search路径，检查路径是否发生碰撞，删除碰撞边，重新search【相当于把碰撞检测延迟到Query Phase做】

---

#### B. Rapidly-exploring Random Tree（RRT）

- 伪代码：
- ![image.png](https://s2.loli.net/2023/02/20/WdSgAH6T89sljbk.png)

- 算法流程（StepSize越小越准确，但计算量变大）

| 1.每一轮迭代sample一个点Xrand                                | 2.找到目前树上离Xrand最近的点Xnear                           | 3.从Xnear指向Xrand拓展一小段距离StepSize生成Xnew             | 4.检测边（Xnew，Xnear）是否发生碰撞，若碰撞则删除，不碰撞则保留 | 5.当Xnew离Xgoal的距离小于一定范围且Collision Free时，完成    |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image.png](https://s2.loli.net/2023/02/20/9uZnOTbrjdA4xWc.png) | ![image.png](https://s2.loli.net/2023/02/20/RH7sUe9raCgOIlm.png) | ![image.png](https://s2.loli.net/2023/02/20/a4kuemldnNypJ8f.png) | ![image.png](https://s2.loli.net/2023/02/20/y4DbVQHK2Mp7JlU.png) | ![image.png](https://s2.loli.net/2023/02/20/bYdR6j2pE4o5WMH.png) |

- 结果：

| 很快找到第一个可行解                                         | 但随着迭代的进行，解并没有变好，因此很难找到最优解，且在整个空间进行采样，不够高效 |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image.png](https://s2.loli.net/2023/02/20/zwy5oiPGZAIF71B.png) | ![image.png](https://s2.loli.net/2023/02/20/PTpnQjRiauBGbcr.png) |

---

### （3）Optimal Path Planning Methods

#### 1. 动态规划（Dynamic Programming）最优性准则（贝尔曼递推方程）

![image.png](https://s2.loli.net/2023/02/20/fCIYxrPVwc6W1TG.png)

- eg. ![image.png](https://s2.loli.net/2023/02/20/gVZmkvU6uyBOo3M.png)

---

#### 2. Direct DP Methods

![image.png](https://s2.loli.net/2023/02/20/3prISHne2mlOikv.png)

- 从node10到node1推，从前往后算，可以算出所有nodes的F值
- **每个node只算一次的要求**：graph必须是acyclic的，即层级结构或无环结构
- 然而motion planning中的graph往往是无层级结构的，cyclic的

---

#### 3. RRT*

*Karaman, Sertac, and Emilio Frazzoli. “Sampling-Based Algorithms for Optimal Motion Planning.” The International Journal of Robotics Research, vol. 30, no. 7, June 2011, pp. 846–894, doi:10.1177/0278364911406761*

![image.png](https://s2.loli.net/2023/02/20/8a1DNnrQGoPAMdl.png)

- 伪代码：

![image.png](https://s2.loli.net/2023/02/20/lY6XNEtZc5OkibS.png)

- 前三步和RRT完全一样
- 每次迭代中，当延申一个StepSize找到Xnew后：
  - RRT的做法是直接连接Xnew和Xnear
  - RRT*的做法是：
    - **QueryRange**：以Xnew为圆心画圆，确定邻域范围（圈住了一系列已有的nodes）
    - **ChooseParent**：在邻域内找到一个使Xnew的F值最小的node，即Xmin（F(new)=**min**{F(possible parent node)+edge(ij)}）【有点像Dijkstra中的g】，连接edge（Xnew，Xmin）
    - **Rewire**：对Xnew邻域内的节点Xi，check其目前的F(i)和F(new)+edge(i~new)的大小，如果后者更小，则修改树的结构，将i连到Xnew上（即把Xi的父节点由原来的X某改为Xnew）
    - 关于QueryRange的半径：在低维情况下，设置为比StepSize稍大的常数即可
    - ![image-20230220173448267](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230220173448267.png)

- result：可以看到随着迭代的进行RRT*优化了一开始找到的可行解，找到了更优的解

| 找到第一个可行解                                             | 优化了结果                                                   | 优化了结果                                                   |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image.png](https://s2.loli.net/2023/02/20/hNGgt8WKXC9DzLs.png) | ![image.png](https://s2.loli.net/2023/02/20/S5gLYm3nsZahEbl.png) | ![image.png](https://s2.loli.net/2023/02/20/LmdD821OGcI3VJK.png) |

---

#### 4. RRT*应用中的加速策略

- Bias Sampling（偏置采样）
- Sample Rejection（拒绝某些采样）
  - 当已经有了一个可行解后，其cost即为c*
  - 当采样一个新点并找到Xnew后，计算出Xnew的F值（即g值）
  - 设计一个heuristic函数（通常采用到目标点的欧拉距离）
  - 若g+h>c*，则拒绝该采样
- Branch-and-Bound（剪枝）
  - 当已经有了一个可行解后，其cost即为c*
  - 在现有的nodes中判断，将g+h>>c*的node及其子枝全部减除
- Graph Sparsify
  - 将图网格化
  - 通过选格子的方式进行采样（稀疏采样）
  - 只能找到在离散状态下的可能的最优解，在连续状态下一定不是最优的
- Neighbor Query
  - k-nearest
  - range query
  - KD Tree [Kd-Tree算法](https://blog.csdn.net/ysqjyjy/article/details/50060893)
  - Range Tree [区域树（Range Tree）的构建（Build）与查询（Query](https://blog.csdn.net/qq_41685265/article/details/111207494)
- Delay Collision Check
- Bi-Direction Search（双向搜索）
- Conditional Rewire（找到第一个解后再进行rewire操作）

---

### （4）Accelerate Convergence（加速收敛）

#### 1. RRT*存在的问题

- Over-exploitation：rewire了很多non-promising的节点

  - 例如当存在一个可行解，且其cost=c0，对于f(Xnew)+h(Xnew)>c0的节点的邻域节点再进行rewire是不必要的

  - ![image.png](https://s2.loli.net/2023/02/20/SLztkogYi8lfqRE.png)

- Under-exploitation：有一些可能优化解的rewire没有得到操作

  - | 图中可以观察到Xnew右侧的点由于得到了rewire其f值得到了优化，  | 但是到goal节点的路径并没有得到优化，这是因为goal的父节点并不在Xnew的邻域范围内，没有被rewire到 |
    | ------------------------------------------------------------ | ------------------------------------------------------------ |
    | ![image.png](https://s2.loli.net/2023/02/20/TqcaXGr7kO9RSEf.png) | ![image.png](https://s2.loli.net/2023/02/20/Z1LrPO3u2JoKDyi.png) |

#### 2. RRT#（从exploitation角度提升收敛速度）

*O. Arslan and P. Tsiotras, "Use of relaxation methods in sampling-based algorithms for optimal motion planning," 2013 IEEE International Conference on Robotics and Automation, 2013, pp. 2421-2428, doi: 10.1109/ICRA.2013.6630906*.

![image.png](https://s2.loli.net/2023/02/20/5nLHhDpAO392cjG.png)

- 从图中可以看到RRT*向各个方向都进行了充分了延申，这对于single-query的问题来说是浪费的
- RRT#更专注于特定方向的求解，因此效率更高

![image.png](https://s2.loli.net/2023/02/20/lPEtygDoNsG8TcW.png)

#### 3. Informed RRT*（从exploration的角度提升收敛速度）

*J. D. Gammell, T. D. Barfoot and S. S. Srinivasa, "Informed Sampling for Asymptotically Optimal Path Planning," in IEEE Transactions on Robotics, vol. 34, no. 4, pp. 966-984, Aug. 2018, doi: 10.1109/TRO.2018.2830331*.

![image.png](https://s2.loli.net/2023/02/20/oK9ugEwHbaMWzIq.png)

##### a.基本思想

![image.png](https://s2.loli.net/2023/02/20/sNFrOvKCphbzmZi.png)

- 对全知集的估计
  - 全知集Omniscient Set：在该集合内采样可以提升解的最优性的的集合（即上图中的不规则图形）
- 估计形式：
  1. Bounding Box：在每个维度设置坐标上下限限制区域，二维即为矩形，三维长方体
  2. Path Heuristics：在现有路径周围横向扩展一定宽度形成的区域
  3. L2 Heuristics：以起终点为焦点，现有路径的长度为长轴，画椭圆

![image.png](https://s2.loli.net/2023/02/20/I45k7E3tYN8LKW6.png)

- 对比结果：L2 Heuristics的Precision率和Recall率均为最高
- **只有以欧氏距离为Heuristics时，才可以采用该椭圆为Informed Set，采用其他Heuristics时，Informed Set需要另外设计，在高维中，很可能不能显式的设计出来**

![image.png](https://s2.loli.net/2023/02/20/wgeORAUhrC3mLo4.png)

##### b.采样操作

![image.png](https://s2.loli.net/2023/02/20/sg59p6c87LSGxZR.png)

1. 在单位圆中均匀采样
2. 放缩长短轴，使其变为椭圆形
3. 旋转一定角度
4. 平移到所需要的位置

#### 4.GuILD (Guided Incremental Local Densification)（也是从exploration的角度提升收敛速度，相当于Informed的提升版）

*Aditya Mandalika and Rosario Scalise and Brian Hou and Sanjiban Choudhury and Siddhartha S. Srinivasa, Guided Incremental Local Densification for Accelerated Sampling-based Motion Planning," in Arxiv, 2021, https://arxiv.org/abs/2104.05037*

##### a.椭圆Informed Set策略的局限性

1. 对于下图所示的两种情况，可以预见，初始找到的可行解极大概率是曲折的，这意味着c*（当前cost）将较大，这会导致椭圆面积较大，此时采样到小径的概率是极低的，因此很难找到最优解

   ![image.png](https://s2.loli.net/2023/02/20/x3AJcLoTiaWE9et.png)

2. 只有在c*变小的时候，椭圆的面积才可能减小

##### b.基本思想

- 从当前的已经扩展的点中选择一个中间信标点b，建立两个椭圆，在这两个椭圆的并集范围内采样

![image.png](https://s2.loli.net/2023/02/20/MP16HBgKopcAhjE.png)

- 第一个椭圆以起点和信标点为焦点，以从起点到该点的当前最优cost（g(b)）为长轴
- 第二个椭圆以信标点和终点为焦点，以（当前最优cost-g(b)）为长轴
- 易证，这两个椭圆的并集（LSs：Local Subsets）是Informed Set（IS）的子集

![image.png](https://s2.loli.net/2023/02/20/AnBRLyO6WDPkKhu.png)

- 效果：

![image.png](https://s2.loli.net/2023/02/20/FeRgcqLaOzplTkA.png)

### （5）发展趋势

![image.png](https://s2.loli.net/2023/02/20/3t42hx5MzDLmjrk.png)

[1]OMPL：https://ompl.kavrakilab.org/ （强推，很全面）

[2] https://moveit.ros.org/ 

[3] https://github.com/ZJU-FAST-Lab/sampling-based-path-finding

---

## 4.动力学约束下的路径规划

### （1）动力学约束

**Kinodyanmic** = **Kinematic**（运动学：eg.避障） + **Dynamic**（动力学：eg.速度，加速度和力上的bounds）

- Q：既然有Trajectory Optimization的步骤，为什么还要再Path Finding的过程中考虑高阶的Kinodynamic的约束？

​	![image.png](https://s2.loli.net/2023/02/27/GfNqmVlednBiRCz.png)

- A：
  - Motion Planning是一个coarse to fine的过程，并不一定要割裂的两部分，换句话说，如果在Path Planning中完全不考虑动力学约束，那么在动力学模型复杂的情况下可能不能得到一个很好的初始路线，那么即使经过Trajectory Optimization，最终结果也会不那么理想
  - Trajectory Optimization是在局部进行优化的，并不能在全局保证动力学最优性
  - eg.    ![image.png](https://s2.loli.net/2023/02/27/eyKanrgQpZLOhjz.png)
    - 在本例中无人机具有向右的初速度，如果在Path Planning的过程中完全不考虑动力学约束，则会生成如紫色实线这样的路径，经过Trajectory Optimization后得到如紫色虚线这样的轨迹，这样的轨迹涉及到急转弯，在运动学上并不是最优的方案
    - 如果在Path Planning时就考虑动力学约束，得到如绿色实线这样的路径，经过Trajectory Optimization后很容易得到一条平滑的feasible的轨迹

---

### （2）常见的动力学模型

- 独轮车模型

  ![image.png](https://s2.loli.net/2023/02/27/RFs1DcM7GbaZLx5.png)

- 差速车模型

  ![image.png](https://s2.loli.net/2023/02/27/Lc91yfQ6WOSBmTt.png)

- 小车模型

![image.png](https://s2.loli.net/2023/02/27/KDEeBoHJRTjZk9u.png)

---

### （3）状态栅格搜索算法—State Lattice Planning

建树/图，使其上每条边都满足动力学约束（feasible motion connections），再运用前述的图搜索算法即可

#### A. Forward Direction：

**离散控制空间**，使机器人前进一段距离，形成路径（例如L2中提到的四联通，八联通，本质上是对质点模型控制量的离散化）

![image.png](https://s2.loli.net/2023/02/27/jYO64Fs3X2aPtTM.png)

- 在differentially driven的情况下有:
  $$
  \dot{s}=f(s,u)
  $$

- 其中S为状态向量，在自动驾驶任务中为

- u为输入量，在自动驾驶任务中为**a（油门：加速度），α（转向盘：角加速度）**

- 选定一系列输入量u0~un，设定一个作用时间T，即可进行前向仿真（Forward Simulation），得到一系列未来的离散状态**Sf0~Sfn**

- ![image.png](https://s2.loli.net/2023/02/27/IJrYZXn6OgCSPps.png)

- 特点：

  - 无任务导向（类似Dijkstra），效率低
  - 易于实施

- 实施：

  状态空间模型([ACnD 1. 状态空间模型 (State Space Model) - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/466790657))【**线性系统**】
  $$
  \dot{s}=A\cdot s + B\cdot u
  $$
  其中s为n维向量，包含系统的所有状态量，u为系统的输入量（控制量）

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

  在这里，如果将A构建为幂零矩阵（nilpotent matirx），会大大简化计算

  

  

  若再以得到的八个s为s0分别进行如上操作，则可以得到**lattice graph**，例如下图

  ![image.png](https://s2.loli.net/2023/02/27/XB3Z8KkLao4Ru6I.png)

- eg. 对于汽车（非线性系统）

- ![image.png](https://s2.loli.net/2023/02/27/ZQWzVIx74Ry8Sre.png)

---

#### B. Reverse Direction

**离散状态空间**，再将这些离散的状态点进行连接，形成路径（例如L3中提到的PRM本质上是对质点模型x，y两种维度的离散化）

![image.png](https://s2.loli.net/2023/02/27/RqBcm2gLClEsYoF.png)

- 对于differentially driven的情况:
- 选定一系列Sf，计算从S0~Sf所需的u和T（通常是固定T，计算可能的u）
- ![image.png](https://s2.loli.net/2023/02/27/zIK7kViW6gHl9Y1.png)
- ![image.png](https://s2.loli.net/2023/02/27/qcLomV8SgBj1AQE.png)
- 特点：
  - 自然的具有贪心性，很容易做得到目标导向，因此planning的效率较高
  - 但实施难度较大（对于给定起始和末尾状态点，有无数种状态转移路线，这就涉及到最优问题（**Optimal Boundary Value Problem**））

##### * 两边界值最优问题（Optimal Boudary Value Problem, OBVP）

1. 获得可行解（BVP）

   - 对于一维状态转移问题
   - ![image.png](https://s2.loli.net/2023/02/28/1UDktG3SLxndRlQ.png)
   - 可以假设状态转移路线为五次曲线，五次曲线有6个待定参数，一对边界条件可以给出2个方程，因此对方程分别求一阶导和二阶导，共2*3=6个方程，可以解出全部未知量
   - ![image.png](https://s2.loli.net/2023/02/28/ysH8PXr3KGAIzdE.png)

2. 获得最优解（OBVP）

   1. 问题的一般描述

      1. 用到的变量和函数有：

         s(t)：与t有关的n维变量，表示t时刻系统的状态量，有边界条件s(0)

         u(t)：与t有关的m维变量，表示t时刻系统的输入量

         系统建模为：
         $$
         \dot{s(t)}=f(s(t),u(t))
         $$
         J(T)：终末时刻T时刻的惩罚函数值
         $$
         J(T)=h(s(T))+\int_0^Tg(s(t),u(t))dt
         $$
         其中:

         h(s(T))表征是否达到终点；g(s,u)表征路径中的cost；

         H(s,u,λ)：Hamiltonian方程
         $$
         H(s,u,\lambda)=g(s,u)+\lambda^Tf(s,u)
         $$
         λ：n维协变量，维度与s一致

         ---

      2. 求解：

         s*：最优的状态转移方程

         u*：随时间最优的控制输入

         ---

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

         ---

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
         因此显然λ可以设为如下的形式：
         $$
         \lambda=\begin{pmatrix}
         \alpha\\-\alpha t-\beta\\\frac12\alpha t^2+\beta t+\gamma
         \end{pmatrix}^T
         $$
         因此最优控制量u*可以解出：
         $$
         u^*(t)=\arg\mathop{\min}\limits_{u(t)}H(s^*(t),u(t),\lambda(t))\\
         \because u(t)=j(t)\\
         \therefore u^*(t)=\arg\mathop{\min}\limits_{j(t)}H(s^*(t),j(t),\lambda(t))\\
         $$
         其中：
         $$
         H(s^*,j,\lambda)=\frac{1}{T}j^2+\lambda_1 v^*+\lambda_2 a^*+\lambda_3 j
         $$
         此时第二三项已经最小，使H最小的j的取值的求法即为：
         $$
         j^*=\arg\mathop{\min}\limits_{j}(\frac{1}{T}j^2+\lambda_3 j)
         $$
         因此，很容易求出：
         $$
         u^*=j^*=-\frac1T\lambda_3=-\frac1T(\frac12\alpha t^2+\beta t+\gamma)
         $$
         得到j，通过积分很容易得到a，v，p，即解出只与α，β，γ有关的s*：
         $$
         s^*=-\frac2T\begin{pmatrix}
         \frac{1}{120}\alpha t^5+\frac{1}{24}\beta t^4+\frac{1}{6}\gamma t^3+\frac{1}{2}a_0t^2+v_0t+p_0\\
         \frac{1}{24}\alpha t^4+\frac{1}{6}\beta t^3+\frac{1}{2}\gamma t^2+a_0t+v_0\\
         \frac{1}{6}\alpha t^3+\frac{1}{2}\beta t^2+\gamma t+a_0
         \end{pmatrix}^T\\
         s_0=(p_0,v_0,a_0)\ \ is\ \ given
         $$
         将末尾边界条件sf代入有：
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
         此时α，β，γ只与T有关，即s\*，u\*，J*只与T有关
         
         - 可以给定T，即可求出所需的s, u
         - 或在T定义域内求极值，求出J使最小时的T，再求出所需的s, u
      
   3. 一些讨论
   
      1. 上例中不存在与终点有关的惩罚函数h(s)，这是因为末端状态已经由s(T)=sf给出，此时的h(s)可以认为是以下形式：
         $$
         h(s(t))=
         \begin{cases}
         0 & t=T\\\infty & t\ne T
         \end{cases}
         $$
         这个函数在t=T处不可导，因此不引入目标函数J中
   
      2. 可能存在s中某些分量的末状态给定，某些分量不给定的情况，例如：
         $$
         s(T)=(p_f,?,?)
         $$
         此时，目标函数J中就需要加入h(s)，且存在边界条件
         $$
         given\ \ s_i(T)\\
         \lambda_j(T)=\frac{\partial h(s^*(T))}{\partial s_j},\ for\ j\ne i
         \\其中\ \ i+j=n
         $$
         可以想象由于j个分量没给出导致的j个自由度，全部由这j个新的方程约束

---

#### C. Comparison

![image.png](https://s2.loli.net/2023/02/27/mqURXfOkQFHMirT.png)

- 在道路环境下，Sample in control space会出现一部分采样驶出路面，这就会导致可用的采样变少，即最优path只能从一个较小的集合中找出（离散度过大）
- 同质化问题：在Sample in control space，当有一个障碍物使得某个path不可用时，其临近的多个path可能都不可用，这就是同质化问题，若多个path在拓扑环境中过于相似，这样的采样就是低效的（离散度过小）

---

#### D. Heuristic Function

![image.png](https://s2.loli.net/2023/03/01/vEpOFw4caqlz53m.png)

- 假设不考虑障碍物：即解两状态间的OBVP问题
- 假设不考虑动力学：绕过障碍物的直线最短路径
- 工程实际应用可能将两者求和，加权，或取最大值

![image.png](https://s2.loli.net/2023/03/01/TCHUd59blj7gpJ1.png)

---

#### E. Planning in Frenet-serret Frame

因为汽车是行驶在结构化的道路上的，辅以车道线检测的结果，可以把汽车坐标转换到Frenet-Serret坐标系下[(23条消息) 弗莱纳公式（Frenet–Serret formulas）_弗莱纳坐标系_CA727的博客-CSDN博客](https://blog.csdn.net/cfan927/article/details/107233889)，并在此坐标系下进行一种特殊的Lattice Planning，即法向d(t)和切向s(t)分别进行参数化，采样和解OBVP

在论文*Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame,* Moritz Werling, Julius Ziegler, Sören Kammel, and Sebastian Thrun和*Optimal trajectories for time-critical street scenarios using discretized terminal manifolds*, Moritz Werling, Sören Kammel, Julius Ziegler and Lutz Gröll中将d(t)和s(t)都建模为**五次多项式**

![image.png](https://s2.loli.net/2023/03/01/6KrRStbCnXqFZl8.png)

例如对于变道任务，可以预想末状态法向的速度和加速度应该都为0，那么就有
$$
D(T)=\begin{pmatrix}
d_T&0&0
\end{pmatrix}
$$
结合初状态就可以进行OBVP的求解

---

### （4）Hybrid A* （基于离散控制量形成的Lattice Graph）

#### 1. 基本思想

![image.png](https://s2.loli.net/2023/03/02/s92Ql7ixGb4tcpm.png)

离散控制量形成的Lattice Graph有如下两个问题：

1. 为了确保有可行解，往往将控制量离散为很多份，这样就会导致Lattice Graph非常稠密
2. 由于稠密，多条路径之间可能及其相似，即同质性，这样是非常低效的

故考虑对Lattice Graph进行剪枝（prune），剪枝的方法就是结合栅格地图的思想，即**每个栅格中只维护一个Feasible Local Motion**

- Q：当一个栅格中有多个cost时候，如何取舍？
- A：维护从父节点到该节点的cost最小的一个（这里的cost不只是距离，而是综合定义的广义的cost函数的值）

#### 2. 算法流程

![image.png](https://s2.loli.net/2023/03/02/ibvJNuTREyKOZhk.png)

与传统A*的不同：

1. h(n)和g(n)的设计不同
2. 找neighbor的方式不同：
   1. A*是找八邻域的nodes
   2. JPS是找符合特殊跳点规则的neighbor
   3. Hybrid A*中的neighbors是：从父节点（motion）出发将离散的一系列控制量分别经过一次forward  simulate得到的一系列simulated feasible local motions
3. 需要维护每个网格中只有一个feasible local motion：
   1. 若延申某个新的结点时发现其所在的网格中无motion，则将该状态存在该网格中
   2. 若延申某个新的结点时发现其所在的网格中已有motion，则分别比较他们从父节点到自身的cost，保留更小的那个

#### 3. Heuristic Function Design

![image.png](https://s2.loli.net/2023/03/02/54oyFz9TepOURH8.png)

![image.png](https://s2.loli.net/2023/03/02/L8dzfvZ6MjnrOJ3.png)

（a）：2D-Euclidean Distance；（b）：non-holonomic-without-obstacles（即与终末状态求OBVP）

这两者比较显然non-holonomic-without-obstacles更好，因为其更接近H的真实值

但是non-holonomic-without-obstacles在迷宫（多死胡同）中表现不佳，因为它没有障碍物信息，其天然的贪心性质会把机器人往目标方向引导（例如（c）图）

这是可以把holonomic-with-obstacles（即2D shortest path）纳入H函数（即每个位置解一下与目标点的最短路径（可以通过A*或Dijkstra求解）），这样采用non-holonomic-without-obstacles + holonomic-with-obstacles或max（non-holonomic-without-obstacles，holonomic-with-obstacles）作为H函数

#### 4. 工程技巧 One-Shot

在Exploration的过程中，不断尝试解当前状态到目标状态的OBVP，如果出现可解状态，则可以停止探索，直接采用该解

![image.png](https://s2.loli.net/2023/03/02/CwDqEG18XJyPBQc.png)

在Exploration初期，找到这样一个One-Shot解的概率较小，因此可以设置一个常数N，即每扩展N次，尝试一次求One-Shot解，随着与目标点距离越来越近，找到One-Shot解的概率会变大，这时可以减小N，这样会更频繁的尝试求One-Shot解

---

### （5）Kinodynamic RRT*（基于离散状态量形成的Lattice Graph）

![image.png](https://s2.loli.net/2023/02/20/lY6XNEtZc5OkibS.png)

Kinodynamic RRT\*与RRT\*的不同

1. Sample：RRT\*在x,y二维空间中采样，K-RRT\*在n维空间中采样（s向量的维度为n）

2. 如何确定邻域：

   n维空间中一个点，以及定义好的cost函数，cost≤r的点的集合叫做reachable set

   1.  backward-reachable set：以该点为终点，即以高维空间中任意点为起点，以小于r的cost能够到达该点的所有点的集合
   2.  forward-reachable set：以该点为起点，即高维空间中以该点为起点，以小于r的cost能到达的所有点的集合

3. ChooseParents：找到$x_new$ 
