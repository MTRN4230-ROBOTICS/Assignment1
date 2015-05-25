data = load('totalQD.mat');
times = load('totalT.mat');
% format long g
d = data.totalQD;
t = times.totalT;
n = size(d,2);
ans = [];
% setA = d(1:10,1);
% setB = d(:,2);
times = diff([0; t]);
    for j=1:1:n
 
        ans(:,j) = diff( [0; d(:,j)] )./ times;
    
    
    end   
    
 ans;
 figure(101);
 qplot(t,ans);
    
%  figure(101);plot(t, ans,'g');
    disp('------------');
%     disp(ddQtf);

    
    

    
% figure(100)
% plot(t,d,'r.');hold on;




%     for j=1:N
%         ddQtf(:,j) = diff( [0; dQt(:,j)] )./diff([0; tsim]);
%     end  
%     
%   
% 
% plot(t(1:end-1)', dq1,'g.');hold on;
% plot(t(1:end-1)', dq2,'b.');
% for i  = 2:1:n-1
% % 
% % dqdt(i) = ( setA(i+1) - setA(i-1) ) / (t(i+1) - t(i-1));
% dq(i) = (setA(i+1) -  setA(i-1)) / (t(i+1) - t(i-1));
% dt(i) =(t(i+1) - t(i-1));
% % dq(i) = mod(setA(i+1) - setA(i),t(i+1) - t(i));
% % dt(i) = t(i+1) - t(i);
% % for i = t
% % d(:,1)
% % 
% % 
% % 
% % 
% end 
% figure(100);
% plot(t(1:end-1),dq,'r.');
% o = size(dt);
% l = size(dqdt);
% axis([0 3 0 3]);


% plot(dt,dqdt,'r');



a = 1;





