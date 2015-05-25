function [a,modifiedQDD,modifiedQDDD ] = calc_derivatives3(totalT, totalQD)
% data = load('totalQD.mat');
% times = load('totalT.mat');
modifiedQDD = [];
f1 = [];
modifiedQDDD = [];
f2 = [];
d =  totalQD;
t = totalT;
n2 = size(t);
step = t(end)/ n2(1,1);
x = t(1):step:t(end);
x = x(1:end -2);
n1 = length(x);
newstep = 3 / n1;
% t_interest = [0:0.1:3];
a = [0:newstep:3];
n2  = size(d,2);
for i = 1:1:n2
    
    f = d(:,i)';
    f1(:,i) = f;
    y = diff(f)/step;
    p = polyfit(x,y,7);
    y_interest = polyval(p,a);
    modifiedQDD = [modifiedQDD, y_interest'];
    
% % % %%  plot accleration and velocity for comparision
% % %     figure(100);
% % %     plot(t_interest1,y_interest','black.');hold on;
% % %     plot(t(1:end)',f,'g.');grid on;
end
modifiedQDD;
data1 = modifiedQDD;

for j = 1:1:n2
    
    fu = data1(:,j)';
    f2(:,j) = fu;
    y2 = diff(fu)/step;
    p2 = polyfit(x,y2,7);
    y_interest2 = polyval(p2,a);
    modifiedQDDD = [modifiedQDDD, y_interest2'];
% % %     plot jerk and accleration for comparision
% % %     figure(101);
% % %     plot(t_interest1,y_interest2','black.');hold on;
% % %     plot(t(1:end)',fu,'g.');grid on;
    
    
end

modifiedQDDD;
end







