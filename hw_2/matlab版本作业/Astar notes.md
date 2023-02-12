### main.m

- 变量：
  - ![image-20230212161425070](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230212161425070.png)
  - map ((n+2)*2 double): 用于储存起点，n个障碍物点和终点的坐标
- 算法流程：
  - 调用obstacle_map生成map
  - 调用A_star_search生成path**（todo）**
  - 可视化：visualize_map(map, path, [])

---

### A_star_search.m框架

- 变量：

  - ![image-20230212162307349](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230212162307349.png)
  - MAP：栅格热图，起点，终点，障碍物点和free点上分别具有不同的值
  - CLOSED：close list：用于存放障碍物和已经visit过的节点
  - CLOSED_COUNT = len（CLOSED）
  - OPEN：open list：用于存放expand到的点，其数据结构如下：
    - ![image-20230212163425148](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230212163425148.png)
    - 其中第一位为flag位，当点被弹出open list后不用将其从open list中删除，只要将该flag位置为0即可，这样做的好处是可以记录所有expand过的节点
    - **从逻辑上讲，算法中的container等效于该OPEN中所有flag位=1的点的集合**
  - OPEN_COUNT = len（OPEN）
  - xStart，yStart存放起始点坐标
  - xTarget，yTarget存放目标点坐标
  - xNode，yNode存放当前visit点的坐标（流动）
  - goal_distance：当前node到目标点的距离，即为Heuristic，可以通过修改distance.m采用不同的启发函数（默认为欧氏距离）
  - path_cost：从父节点到该点走过的距离

- 算法流程：

  - 对2D grid map进行初始化：

    ![image-20230212163036395](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230212163036395.png)

  - 定义OPEN LIST：

    ![image-20230212163425148](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230212163425148.png)

  - 定义CLOSE LIST：

    ![image-20230212163459361](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230212163459361.png)

  - 将所有障碍物置入CLOSE LIST：

    ![image-20230212163635878](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230212163635878.png)

  - **注意：CLOSE LIST和OPEN LIST每次update后要及时update他们的COUNT**

  - 初始化：visit起始点

  - ![image-20230212165552929](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230212165552929.png)

---

### 可用封装工具

- dist = distance(x1,y1,x2,y2)：计算两点的欧氏距离

- n_index = node_index(OPEN,xval,yval)：根据坐标xval，yval在OPEN list中找到该点的索引

- new_row = insert_open(xval,yval,parent_xval,parent_yval,hn,gn)：扩展OPEN list

  - ![image-20230212171234806](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230212171234806.png)
  - flag位默认置为1
  - f由h和g自动计算得出

- i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget)：找到container中f值最小的node

- exp_array=expand_array(node_x,node_y,gn,xTarget,yTarget,CLOSED,MAX_X,MAX_Y)：根据当前节点找到所有可扩展结点

  - 输出的数据格式：

  ![image-20230212172803066](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230212172803066.png)

  - ps：这里的gn是从当前节点扩展到该结点的gn和

  ![image-20230212173410864](C:/Users/Administrator/AppData/Roaming/Typora/typora-user-images/image-20230212173410864.png)

---

### A_star_search.m实现**（todo）**

- 逻辑根据伪代码进行即可
- 开始循环前初始化一个含有起点的container
- 开始循环：
  1. 退出条件：若container为空，退出循环
  2. 从container中弹出具有最小f的节点
     - 弹出的同时visit该节点
     - 已访问则将OPEN的flag位置为0
     - 将该点加入closelist
  3. 退出条件：若visit到目标点，退出循环
  4. expand节点，对所有expand到的neighbors检查是否已经在OPEN中
     - 若已经在，则比较g值，维护最小的g值
     - 若还不在，则把该neighbor加入OPEN中，g值存为现有的g值
  5. 下一循环
- 生成path：找到target后当前访问的node即为终点，沿终点逐一沿父节点回溯直到找到起始点



![result](F:/shenlanxueyuan/robotics/L2/hw_2/hw_2/matlab%E7%89%88%E6%9C%AC%E4%BD%9C%E4%B8%9A/code/result.jpg)





























