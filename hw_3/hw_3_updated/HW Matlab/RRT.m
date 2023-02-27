%***************************************
%Author: Linxuan Jia
%Date: 2023-02-27
%***************************************
%% 流程初始化
clc
clear all; close all;
x_I=50; y_I=50;           % 设置初始点
x_G=750; y_G=750;       % 设置目标点（可尝试修改终点）
Thr=50;                 % 设置目标点阈值
Delta= 30;              % 设置扩展步长（即StepSize）
%% 建树初始化
T.v(1).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % 起始节点的父节点仍然是其本身
T.v(1).yPrev = y_I;
T.v(1).distPrev=0;      % 从父节点到该节点的距离，这里可取欧氏距离
T.v(1).indPrev = 0;     % 父节点的index
%% 开始构建树，作业部分
% figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,2);%地图x轴长度
yL=size(Imp,1);%地图y轴长度
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',5,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
hold on
plot(x_G, y_G, 'go', 'MarkerSize',5,'MarkerEdgeColor','g', 'MarkerFaceColor','g');% 绘制起点和目标点
hold on
count=1;
bFind = false;

for iter = 1:3000
    %Step 1: 在地图中随机采样一个点x_rand
    x_rand=[];
    x_rand(1)=round(xL*rand);
    x_rand(2)=round(yL*rand);
    
    %Step 2: 遍历树，从树中找到最近邻近点x_near 
    x_near=[];
    dlist=[];
    if count==1
        x_near(1)=T.v(1).x;
        x_near(2)=T.v(1).y;
        I=1;
    elseif count>1
        for i=1:count
            dlist(i)=distance(x_rand(1),x_rand(2),T.v(i).x,T.v(i).y);
        end
        [M,I]=min(dlist);
        x_near(1)=T.v(I).x;
        x_near(2)=T.v(I).y;
    else
        disp('count error!')
    end

    %Step 3: 扩展得到x_new节点
    x_new=[];
    angle=atan((x_rand(2)-x_near(2))/(x_rand(1)-x_near(1)));
    if (x_rand(1)-x_near(1))>0
        x_new(1)=x_near(1)+Delta*cos(angle);
        x_new(2)=x_near(2)+Delta*sin(angle);
    else
        x_new(1)=x_near(1)-Delta*cos(angle);
        x_new(2)=x_near(2)-Delta*sin(angle);
    end
    
    % 检查节点是否是collision-free,如果是的话，可视化并将x_new加入Tree
    if ~collisionChecking(x_near,x_new,Imp) 
       continue;
    end
    plot(x_new(1), x_new(2), 'yo', 'MarkerSize',7, 'MarkerEdgeColor','y','MarkerFaceColor','y');
    hold on
    plot([x_near(1),x_new(1)],[x_near(2),x_new(2)],'-bs','LineWidth',1)
    hold on
    count=count+1;
    
    %Step 4: 将x_new插入树T，父节点是x_near
    T.v(count).x = x_new(1);    
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = x_near(1);     % 起始节点的父节点仍然是其本身
    T.v(count).yPrev = x_near(2);
    T.v(count).distPrev= distance(x_new(1),x_new(2),x_near(1),x_near(2));      % 从父节点到该节点的距离，这里可取欧氏距离
    T.v(count).indPrev = I;     % 父节点的index
    
    %Step 5:检查是否到达目标点附近 
    %提示：注意使用目标点阈值Thr，若当前节点和终点的欧式距离小于Thr，则跳出当前for循环
    if distance(x_new(1),x_new(2),x_G,y_G)<Thr
        count=count+1;
        T.v(count).x = x_G;    
        T.v(count).y = y_G; 
        T.v(count).xPrev = x_new(1);     % 起始节点的父节点仍然是其本身
        T.v(count).yPrev = x_new(2);
        T.v(count).distPrev= distance(x_new(1),x_new(2),x_G,y_G);      % 从父节点到该节点的距离，这里可取欧氏距离
        T.v(count).indPrev = count-1;     % 父节点的index
        plot([x_G,x_new(1)],[y_G,x_new(2)],'-bs','LineWidth',1)
        hold on
        bFind=true;
        break
    end
   
%     pause(0.05); %暂停一会，使得RRT扩展过程容易观察
end
%% 路径已经找到，反向查询
if bFind
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % 终点加入路径
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % 沿终点回溯到起点
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 起点加入路径
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'r', 'Linewidth', 2);
    end
    saveas(gcf, 'result', 'png')
else
    disp('Error, no path found!');
end
