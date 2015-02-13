clear('all');
close all
format compact

x=1:10;

y=1:2:30;
iter=1;
dist_can_reach=[];
                position_can_reach=[];
while y(iter)-x(end) <= 0
                    dist_can_reach(iter)=y(iter);
                  
                iter=iter+1;
            end