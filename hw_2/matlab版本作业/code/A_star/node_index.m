function n_index = node_index(OPEN,xval,yval)
    %This function returns the index of the location of a node in the list
    %OPEN (根据坐标找索引)
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    i=1;
    while(OPEN(i,2) ~= xval || OPEN(i,3) ~= yval )
        i=i+1;
    end
    n_index=i;
end