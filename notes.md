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










