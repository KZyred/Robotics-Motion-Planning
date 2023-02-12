function i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget)
% Function to return the Node with minimum fn
% This function takes the list OPEN as its input and returns the index of the
% node that has the least cost （找到open list中cost最小的node的index）
%
%   Copyright 2009-2010 The MathWorks, Inc.

 temp_array=[];
 k=1;
 flag=0;
 goal_index=0;
 for j=1:OPEN_COUNT   % 遍历open list
     if (OPEN(j,1)==1)   % 只访问flag位为1的nodes，即container中的nodes
         temp_array(k,:)=[OPEN(j,:) j]; % temp_array除了把flag=1的nodes筛选了出来，
                                        % 同时记录了他们在原list中的索引
                                        % 即temp_array(:,9)记录的是索引信息
         if (OPEN(j,2)==xTarget && OPEN(j,3)==yTarget)
             flag=1;
             goal_index=j;  % 存放目标点的索引值
         end
         k=k+1;
     end
 end %Get all nodes that are on the list open
 %if flag == 1 % one of the successors is the goal node so send this node
 %    i_min=goal_index;
 %Send the index of the smallest node
 %end;
 if size(temp_array ~= 0)
  [~,temp_min]=min(temp_array(:,8)); %Index of the smallest node in temp array
  i_min=temp_array(temp_min,9); %Index of the smallest node in the OPEN array
 else
     i_min=-1;%The temp_array is empty i.e No more paths are available.
 end