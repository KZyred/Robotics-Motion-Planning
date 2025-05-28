## 2.基于搜索的路径规划

### (1). Graph Search Basis

#### ① Robot Configuration Space

-   Robot Configuration: 机器人的一个位姿(配置)
-   Robot Degree Of Freedom (DOF): 用于描述机器人位姿的向量的维度 n
-   Robot Configuration Space: n 维向量空间，包含机器人所有可能的位姿

![image.png](https://s2.loli.net/2023/02/20/xzMpJLSkPo5RDbi.png)

![image.png](https://s2.loli.net/2023/02/20/9rVtJb3clqogvfF.png)

#### ② Graphs Search

-   Graphs（包含 nodes 和 edges）

![image.png](https://s2.loli.net/2023/02/20/s4azdrxfg6n9YVc.png)

-   Search Tree

![image.png](https://s2.loli.net/2023/02/20/E4IdO1c6pPWzYQJ.png)

-   **Overview**

    -   A container: to store all the nodes to be visited (initialized with the start state)

    -   Loop:

        -   根据给定的 score function 从 container 中**remove**出一个 node，意为 visit 该 node
        -   **expand**该 node 的所有 neighbors
        -   **push**这些 neighbors 到 container 中

    -   终止条件：container 中无 node，或 visit 到 end node

    -   if the graph is cyclic：

        -   维护一个 close list，放置已经 visit 过的 nodes，这些 nodes 不会再被 visit

    -   score function 如何定义，以更快的（用更少次的 expansion）到达 end node

-   Breadth First Search (BFS)

    -   其 container 是一个 queue，遵循先进先出原则

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

        | 当前 container | 当前 visit 的 node | 当前 container | expand 到的 node |
        | -------------- | ------------------ | -------------- | ---------------- |
        | a              | a                  | nan            | b,c,d            |
        | b,c,d          | d                  | b,c            | j                |
        | j,b,c          | c                  | j,b            | h,i              |
        | h,i,j,b        | b                  | h,i,j          | e,f,g            |
        | e,f,g,h,i,j    | j                  | e,f,g,h,i      | nan              |
        | e,f,g,h,i      | i                  | e,f,g,h        | nan              |
        | e,f,g,h        | h                  | e,f,g          | nan              |
        | e,f,g          | g                  | e,f            | m,n              |
        | m,n,e,f        | f                  | m,n,e          | l                |
        | l,m,n,e        | e                  | l,m,n          | k                |
        | k,l,m,n        | n                  | k,l,m          | o                |

        -   expand 的 node 从左压入，visit 时从右取出

-   Depth First Search（DFS）

    -   其 container 是一个 stack，遵循后进先出原则

        ![image.png](https://s2.loli.net/2023/02/20/mn14KOtMi2kH7hD.png)

    -   同一深度层级的节点压入的顺序需要自己定义 (这个定义有可能能极大的节省计算量)

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

        | 当前 container（开口向左） | 当前 visit 的 node | 当前 container（开口向左） | expand 到的 node |
        | -------------------------- | ------------------ | -------------------------- | ---------------- |
        | a                          | a                  | nan                        | b,c,d            |
        | b,c,d                      | b                  | c,d                        | e,f,g            |
        | e,f,g,c,d                  | e                  | f,g,c,d                    | k                |
        | k,f,g,c,d                  | k                  | f,g,c,d                    | nan              |
        | f,g,c,d                    | f                  | g,c,d                      | l                |
        | l,g,c,d                    | l                  | g,c,d                      | nan              |
        | g,c,d                      | g                  | c,d                        | m,n              |
        | m,n,c,d                    | m                  | n,c,d                      | nan              |
        | n,c,d                      | n                  | c,d                        | o                |

        -   expand 的 node 从左压入，visit 时从左取出

-   BFS vs. DFS

![image.png](https://s2.loli.net/2023/02/20/nyOIKgeNvsH7FrQ.png)

-   Heuristic Search (Greedy Best First Search，贪心算法)
    -   expand 到的 node 正常压入
    -   visit node 时，根据一个启发函数选择一个最优的 node
    -   该启发函数通常为当前节点到目标节点的距离（欧式距离 Euclidean Distance 或曼哈顿距离 Manhattan Distance）
    -   ![image.png](https://s2.loli.net/2023/02/20/Mr3sQcl8RLGofV7.png)
    -   贪心算法易陷入局部最优，因为启发函数在计算时是忽略了障碍物的
    -   ![image.png](https://s2.loli.net/2023/02/20/pkYND1OrnUc9gja.png)

### (2). Dijkstra Algorithm

#### ① visit 策略：

-   visit the node with the cheapest accumulated cost **g(n)**
    -   g(n)：从起点到当前 node 累积 cost 的和
    -   在 expand 时，检查所有 neighbors 现有的 cost 值，例如对于当前访问 node n 的 neighbor m，若 g(n)+cost(n->m)<g(m)，则把 g(m)更新为 g(n)+cost(n->m)
    -   最优性保证：所有被 visited 过的 nodes 中所储存的 g 值总是从起点到该点的最小 cost

#### ② container:

-   a priority queue（根据 g 值排序）
    -   container = open list
    -   close list：用于存放已经扩展过的 nodes，这些 nodes 不会再被 visit

#### ③ 初始化：

-   container：起点 Xs；
-   g(Xs)=0
-   其他所有点 g(n)=inf

#### ④ 伪代码：

-   ![image.png](https://s2.loli.net/2023/02/20/sHSnIqYG4Lk3ydr.png)

⑤ 示例：

-   ![image.png](https://s2.loli.net/2023/02/20/fCmVk4TP1oOAR5b.png)

-   | 当前容器(根据 g 值自动排序，小的先弹出) | 当前访问节点 | 当前容器               | expand 到的 node  | g(S) | g(a) | g(b) | g(c) | g(d) | g(e) | g(f) | g(h) | g(p) | g(q) | g(r) | g(G) |
    | --------------------------------------- | ------------ | ---------------------- | ----------------- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
    |                                         |              |                        |                   | 0    | inf  | inf  | inf  | inf  | inf  | inf  | inf  | inf  | inf  | inf  | inf  |
    | S(0)                                    | S(0)         | nan                    | e(+9),d(+3),p(+1) | 0    | inf  | inf  | inf  | 3    | 9    | inf  | inf  | 1    | inf  | inf  | inf  |
    | e(9),d(3),p(1)                          | p(1)         | e(9),d(3)              | q(+15)            | 0    | inf  | inf  | inf  | 3    | 9    | inf  | inf  | 1    | 16   | inf  | inf  |
    | q(16),e(9),d(3)                         | d(3)         | q(16),e(9)             | b(+1),c(+8),e(+2) | 0    | inf  | 4    | 11   | 3    | 5    | inf  | inf  | 1    | 16   | inf  | inf  |
    | q(16),c(11),e(5),b(4)                   | b(4)         | q(16),c(11),e(5)       | a(+2)             | 0    | 6    | 4    | 11   | 3    | 5    | inf  | inf  | 1    | 16   | inf  | inf  |
    | q(16),c(11),a(6),e(5)                   | e(5)         | q(16),c(11),a(6)       | r(+1),h(+8)       | 0    | 6    | 4    | 11   | 3    | 5    | inf  | 14   | 1    | 16   | 6    | inf  |
    | q(16),h(14),c(11),a(6),r(6)             | r(6)         | q(16),h(14),c(11),a(6) | f(+1)             | 0    | 6    | 4    | 11   | 3    | 5    | 7    | 14   | 1    | 16   | 6    | inf  |
    | q(16),h(14),c(11),f(7),a(6)             | a(6)         | q(16),h(14),c(11),f(7) | nan               | 0    | 6    | 4    | 11   | 3    | 5    | 7    | 14   | 1    | 16   | 6    | inf  |
    | q(16),h(14),c(11),f(7)                  | f(7)         | q(16),h(14),c(11)      | c(+0),G(+2)       | 0    | 6    | 4    | 7    | 3    | 5    | 7    | 14   | 1    | 16   | 6    | 9    |
    | q(16),h(14),c(11),G(9),c(7)             | c(7)         | q(16),h(14),c(11),G(9) | a(+0)             | 0    | 6    | 4    | 7    | 3    | 5    | 7    | 14   | 1    | 16   | 6    | 9    |
    | q(16),h(14),c(11),G(9)                  | G(9)         |                        |                   |      |      |      |      |      |      |      |      |      |      |      |      |

#### ⑥ 优缺点：

-   优点：一定能找到最优解
-   缺点：不包含终点信息，各个方向均匀扩散，效率低

### (3). A\*：Dijkstra with a Heuristic

#### ① visit 策略：

-   visit the node with cheapest f(n) = g(n) + h(n)
    -   g(n): 当前 node 到起点的累积最小 cost
    -   h(n): 当前 node 到终点的估计最小 cost
    -   其他和 Dijkstra 完全一样

#### ② 伪代码：

-   ![image.png](https://s2.loli.net/2023/02/20/JL26jNQZaWclgeA.png)

#### ③ 示例：

-   ![image.png](https://s2.loli.net/2023/02/20/gsjWVJ6M7lDkfIR.png)

#### ④ 最优性保证：任意一个 node 估计的 h(n)的必须小于该点到终点实际的 cost

-   h(n)设计：
    -   Euclidean Distance：always less -> √
    -   Manhattan Distance：depends -> ? （当机器人只能横纵向移动不能斜行时是最优）
-   最优性 vs 速度：
-   ![image.png](https://s2.loli.net/2023/02/20/BwNQY2X9vbynD8W.png)
-   ![image.png](https://s2.loli.net/2023/02/20/JXwZkiWUG8fnzYE.png)

#### ⑥ A\*在工程中的注意事项：

##### a.如何把 grid map 变为 graph（四联通 or 八联通）

-   ![image.png](https://s2.loli.net/2023/02/20/u6tkzK5rqG2NxQX.png)

##### b.priority queue 的实现

-   ![image.png](https://s2.loli.net/2023/02/20/qGy87fXmoYkEMbl.png)

##### c.启发函数的优化：

-   ![image.png](https://s2.loli.net/2023/02/20/FLwHuaTW6kZD9qA.png)
-   欧式距离所估计的 h(n)虽然符合 h(n)≤h\*(n)，但相差很多(不 tight/远小于)，导致扩展过程中会 2 扩展到很多无用的 nodes
-   解决方案：Diagonal Heuristic
-   ![image.png](https://s2.loli.net/2023/02/20/5sSeAotQVNBwXJ2.png)

##### d.Tie Breaker

-   当任何一步的扩展中遇到 f 值相同的 nodes 时，若无操作，则会根据先后顺序排列，这些 nodes 可能全部得到 expand，这将导致扩展很多不必要的 nodes
-   通过一个操作使对这些 nodes 的选择不完全随机，而是有倾向性
-   当两个 node 的 f 值相同时，将其中一个放大一个很小的值
-   ![image.png](https://s2.loli.net/2023/02/20/eZd4oysKAWIG1lT.png)
-   Tie Breaker 的其他方法：
    -   当 f 相同时对 h 进行排序比较
    -   给 h 加上根据事先建立好的随机数表（坐标的哈希表）
    -   根据想要的加过对 h 进行修改（例如想要走对角线，则加一个 cross cost）
    -   ![image.png](https://s2.loli.net/2023/02/20/qXt6whgST2odyUQ.png)
-   Tie Breaker 对后续 Trajectory generation 的影响
    -   ![image.png](https://s2.loli.net/2023/02/20/cbjZ9XTea6v52Kh.png)

### (4). JPS: Jump Point Search

-   核心思想：
    -   和 A\*只是 expand neighbors 的方式不一样：
        -   A\* expand 当前 node 几何上的所有邻居
        -   JPS 根据 Look Ahead Rule 和 Jumping Rule 来将符合条件的跳跃点加入 open list
    -   neighbors 加入 open list 后依然是根据 f(n)=g(n)+h(n)最小的 priority queue 弹出 node 进行 visit

![image.png](https://s2.loli.net/2023/02/20/naRwNBT1P5K7HY9.png)

#### ① Look Ahead Rule

对当前 visit 的 node X 的所有可能 expand 的 neighbors nodes 而言，如果从父节点不经过 X 到该点的 cost 小于从父节点经过 X 到该点的 cost，则为 inferior neighbors，否则为 natural neighbors，在 expand 时只考虑 natural neighbors，不考虑 inferior neighbors

-   直行时判断条件为**小于等于**；

![image.png](https://s2.loli.net/2023/02/20/XaBKoezuPLqg1Gv.png)

-   eg. 当前节点为 x，父节点为 4：

| neighbor node | cost without x | cost with x  | result      |
| ------------- | -------------- | ------------ | ----------- |
| 1             | 4->1 1         | 4->x->1 √2   | inferior    |
| 2             | 4->2 √2        | 4->x->2 2    | inferior    |
| 3             | 4->2->3 1+√2   | 4->x->3 1+√2 | inferior    |
| 5             | 4->2->5 2√2    | 4->x->5 2    | **natural** |

-   对角线行驶时判断条件为**小于**；

![image.png](https://s2.loli.net/2023/02/20/V2JdvMsw6yGrpET.png)

-   eg. 当前节点为 x，父节点为 6：

| neighbor node | cost without x  | cost with x  | result      |
| ------------- | --------------- | ------------ | ----------- |
| 4             | 6->4 1          | 6->x->4 √2   | inferior    |
| 1             | 6->4->1 2       | 6->x->1 2√2  | inferior    |
| 2             | 6->4->2 1+√2    | 6->x->2 1+√2 | **natural** |
| 3             | 6->4->2->3 2+√2 | 6->x->3 2√2  | **natural** |

-   特殊情况：forced neighbor（直行时或斜行时旁边紧邻有障碍物（如下图））
    -   当直行的 2 or 7 号位有障碍物时，则需要额外将 3 or 8 号位置为 natural neighbor
    -   当斜行的 4 or 7 号位有障碍物时，则需要额外将 1 or 8 号位置为 natural neighbor
    -   ![image.png](https://s2.loli.net/2023/02/20/ERN2flonOkSYMBz.png)

#### ② Jumping Rules

-   对每一个 node，优先水平方向和竖直方向 expand：
    -   如果 expand 到具有 forced neighbor 的特殊点时，将该 node 加入 open list（注意不是将这个特殊点加入了 open list）
    -   如果水平竖直方向都没有找到特殊 node，则沿斜向延申一个 node，继续沿水平，竖直方向搜索特殊点
-   下图中蓝色点加入了 open list
-   绿色点所有可能延申的方向都得到了充分的延申，则绿色点可以加入 close list

![image.png](https://s2.loli.net/2023/02/20/WniCJyYs4FV7Sl2.png)

#### ③ Example1

![image.png](https://s2.loli.net/2023/02/20/eRpWtGr2P3nKJj8.png)

-   绿色点横向，纵向搜索 =》 无结果
-   绿色点斜向延申一格后横向，纵向搜索 =》 无结果
-   绿色点斜向延申直到横向，纵向搜索能找到特殊点 =》 找到黄色点（黄色点具有 forced neighbor 青色点）=》 黄色点加入 open list
-   绿色点继续斜向延申 =》无结果 =》 绿色点加入 close list
-   此时 open list 只有黄色点 =》弹出黄色点
-   根据 Look Ahead Rule，斜行的点可以扩展横，纵，斜三个方向，其中纵和斜均无结果，横向找到特殊点青色点（青色点具有 forced neighbor 蓝色点），青色点加入 open list，黄色点加入 close list
-   此时 open list 只有青色=》弹出青色点
-   根据 Look Ahead Rule，横行的特殊点可能的 expand 方向有横行和斜下行，其中横行未找到特殊点，斜下行时找到青色点（青色点横纵向能延伸到终点），将青色点加入 open list
-   弹出青色点
-   优先横纵向搜索，在纵向上找到终点

#### ④ Example2

![image.png](https://s2.loli.net/2023/02/20/qTFGe3M9XvzbAdS.png)

-   这个例子充分的体现出了：
    1. 每个节点所有可能的延申方向得到充分的延申才会加入 close list（图中变灰色的节点）
    2. 从起点 expand 到了两个可能的 neighbor 加入了 open list，但是由于终点的位置不同，所走的路线不同，这是因为 open list 在弹出节点时和 A\*一样，是根据 f(n)=g(n)+h(n)最优来排序的

#### ⑤ Analysis

-   在复杂的迷宫环境中 JPS>A\*
-   在开阔的，障碍物少的环境中 JPS<A\*（JPS 需要延申大面积去确认没有特殊点）

![image.png](https://s2.loli.net/2023/02/20/tykONQCvPLWA8zi.png)
