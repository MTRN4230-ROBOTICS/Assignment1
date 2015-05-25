%%%%%%%%%%%%%
%author: Ziwei Guo, Zicong He,Jialiang Su
% 150511
% Adapted from Corke ch 9.
clear variables; close all; clc; % Take care to clear variables or else elements of L may not be overwritten correctly - maybe a Matlab optimiser problem.

% Initialise all link kinematics
% Note axis limits can only can be queried, do not effect the dynamics!


%% parameter
% joint angles
q0 = [0, 0, 0, 0, 0, 0];   % Home Position 
q1 = pi/180.*[0, 6.5, 68.8, 0, 14.7, 0];
q2 = pi/180.*[-71.4, 70.3, -39.0, 0, 58.7,-71.4];
q3 = pi/180.*[71.4, 70.3, -39.0, 0, 58.7, 71.4];
q4 = pi/180.*[0, 70.3, -39.0, 0, 58.7, 0];
q5 = (pi/180).*[90 -41.61262 0 90 -90 0]; % singularity 
q6 = (pi/180).*[-44.9621, -58.9096, 30.1910, 0, 119.5486, 44.9633]; % singularity 

% DH table 
% theta  d     a        alfa
dh = [
    
    0   0.290     0       pi/2;   %1
    0   0       0.270     0;      %2
    0   0       0.070      -pi/2;  %3
    0   0.302     0       pi/2;   %4
    0   0       0       pi/2;   %5
    0   0.137     0       0;      %6

];


irb_120 = SerialLink(dh);

% offsets
irb_120.offset(1) = pi;
irb_120.offset(2) = pi/2;
irb_120.offset(5) = pi;
    
%limits (product specification P30)
% irb_120.links(1).qlim = (pi/180).*[-165 165];
% irb_120.links(2).qlim = (pi/180).*[-110 110];
% irb_120.links(3).qlim = (pi/180).*[-110 70];
% irb_120.links(4).qlim = (pi/180).*[-160 160];
% irb_120.links(5).qlim = (pi/180).*[-120 120];
% irb_120.links(6).qlim = (pi/180).*[-400 400];


irb_120.links(1).qlim = (pi/180).*[0 0];
irb_120.links(2).qlim = (pi/180).*[0 0];
irb_120.links(3).qlim = (pi/180).*[0 0];
irb_120.links(4).qlim = (pi/180).*[0 0];
irb_120.links(5).qlim = (pi/180).*[0 0];
irb_120.links(6).qlim = (pi/180).*[0 0];

% Define various dynamic parameters of each link
% Mass (kg).
irb_120.links(1).m = 3.067;
irb_120.links(2).m = 3.909;
irb_120.links(3).m = 2.944;
irb_120.links(4).m = 1.328;
irb_120.links(5).m = 0.547;
irb_120.links(6).m = 0.013;


% Position of centre of mass of each link [m]
irb_120.links(1).r = [0 0.250 0];
irb_120.links(2).r = [0.100 0 0];
irb_120.links(3).r = [0.070 0 0.05];
irb_120.links(4).r = [0 0.250 0];
irb_120.links(5).r = [0 0 0.030];
irb_120.links(6).r = [0 0 0];

% Moment of inertia matrix
irb_120.links(1).I = [0.014 0 0; 0 0.014 0; 0 0 0.014];
irb_120.links(2).I = [0.025 0 0; 0 0.04 0; 0 0 0.06];
irb_120.links(3).I = [0.01 0 0; 0 0.01 0; 0 0 0.008];
irb_120.links(4).I = [0.004 0 0; 0 0.003 0; 0 0 0.005];
irb_120.links(5).I = [0.0008 0 0; 0 0.0004 0; 0 0 0.0009];
irb_120.links(6).I = [1e-6 0 0; 0 1e-6 0; 0 0 2e-6];

% Note this final moment of inertia matrix doesn't include the end
% effector. It can be safely ignored for asst 3.

% Motor inertia
irb_120.links(1).Jm = 0.0005;
irb_120.links(2).Jm = 0.0002;
irb_120.links(3).Jm = 0.0001;
irb_120.links(4).Jm = 0.0001;
irb_120.links(5).Jm = 0.00005;
irb_120.links(6).Jm = 0.00001;

% Motor friction
irb_120.links(1).B = 5e-4;
irb_120.links(2).B = 2e-4;
irb_120.links(3).B = 1e-4;
irb_120.links(4).B = 5e-5;
irb_120.links(5).B = 2.5e-5;
irb_120.links(6).B = 1e-5;
    
% Coulomb friction. Set this to 0 as the fdyn function is unable to handle
% non-zero values effectively.
irb_120.links(1).nofriction(); 
irb_120.links(2).nofriction();
irb_120.links(3).nofriction();
irb_120.links(4).nofriction();
irb_120.links(5).nofriction();
irb_120.links(6).nofriction();

% Gear ratio
gear_ratio = 80;
irb_120.links(1).G = gear_ratio; 
irb_120.links(2).G = gear_ratio; 
irb_120.links(3).G = gear_ratio; 
irb_120.links(4).G = gear_ratio; 
irb_120.links(5).G = gear_ratio; 
irb_120.links(6).G = gear_ratio; 

    
% Define gravity 
irb_120.gravity = [0 0 9.8]';


%% 1.1 
% irb_120.links(:).dyn
% See Corke section 9.1.2.



%% 1.2

% simulate the gravity loading(toque against the gravity when there is no load in the pose)
global Torque0; 
Torque0 = irb_120.rne(q2,zeros(1,6),zeros(1,6));   %Define target torque where robot remain still at targer position q2

% Simulate motion of a robot with torques applied at its joints from
% starting pose qi and no initial velocity.
% qi = initial configuration for fdyn
% qid = initial velocity for fdyn
% dqi = initial configuration for controller (same as qi but needs to be
% passed as an additional input argument)
% dqid = initial velocity for controller (same as qid but needs to be
% passed as an additional input argument)
% dqidd = initial acceleration for controller

qi = q1 ;
qid = zeros(size(qi));
dqi = qi;
dqid = zeros(size(qi));
dqidd = zeros(size(qi));
sim_time = 3;
% , dqi, dqid, dqidd

[T,Q,QD] = irb_120.fdyn(sim_time, @irb120_p_controller, qi, qid, dqi, dqid, dqidd);
[QD,QDD,QDDD ] = calc_derivatives(T, QD);

% Show corresponding joint positions as a function of time.
figure(1);
qplot(T,Q);
title('1.2 Joint angle');

figure(2);
qplot(T, QD);
title('1.2 joint Velocity');

figure(3);
qplot(T, QDD);
title('1.2 joint Acceleration');

figure(4);
qplot(T, QDDD);
title('1.2 joint Jerks state');

% plot the a locus(the trajectory of the end effector)
TendEff = irb_120.fkine(Q);
spacePoint = transl(TendEff); % obtain the translation point of the end effector from the matrix
figure(5);
plot3(spacePoint(:,1),spacePoint(:,2),spacePoint(:,3));
xyzlabel;
title('1.2 controller Locus of end effector q1 to q2');
grid on;


% used by other cases
Q1_2 = Q; 
T1_2 = T; 

%% 2.1 
% % % % % % global DesireQ;
% % % % % % global DesireQD;
% % % % % % 
% % % % % % 
% % % % % % totalStep = 605;  % to make number of time is the same the time of controller
% % % % % % [Q,QD,QDD] = jtraj(q1, q2, totalStep); 
% % % % % % % time = linspace(0, 3, steps); 
% % % % % % % [a,b,QDDD ] = calc_derivatives(time', QD);
% % % % % % step = 3/totalStep;
% % % % % % 
% % % % % % totalT = 0; 
% % % % % % totalQ = zeros(1,6); 
% % % % % % totalQD = zeros(1,6);
% % % % % % 
% % % % % % % modified variables for plotting 
% % % % % % Qmodified = Q(1,:);
% % % % % % QDmodified = zeros(1,6); 
% % % % % % 
% % % % % % qi = q1;  % initial q0
% % % % % % qid = zeros(1,6); 
% % % % % % dqi = qi;
% % % % % % dqid = zeros(size(qi));
% % % % % % dqidd = zeros(size(qi));
% % % % % % 
% % % % % % for i = 1 : totalStep-1
% % % % % %     
% % % % % %         DesireQ = Q(i+1,:);
% % % % % %         DesireQD = QD(i+1,:);
% % % % % %         Torque0 = irb_120.rne(DesireQ,DesireQD,zeros(1,6)); 
% % % % % %         
% % % % % %         [controlT,controlQ,controlQD] = irb_120.fdyn(step, @irb120_p_controller2, qi, qid, dqi, dqid, dqidd);
% % % % % %         
% % % % % %         if i == 1 
% % % % % %             
% % % % % %             totalT = controlT; 
% % % % % %             totalQ = controlQ;
% % % % % %             totalQD = controlQD;
% % % % % %             
% % % % % %         else
% % % % % %             
% % % % % %              totalT = [totalT ; totalT(end,1)+ controlT];
% % % % % %              totalQ = [totalQ ; controlQ];
% % % % % %              totalQD = [totalQD ; controlQD]; 
% % % % % %             
% % % % % %         end
% % % % % %             
% % % % % %         % modify Q QD Qdd for root mean square
% % % % % %          Qmodified = [Qmodified ; controlQ(end,:)];
% % % % % %          
% % % % % %          meanqdot = [mean(controlQD(:,1)), mean(controlQD(:,2)), mean(controlQD(:,3)), mean(controlQD(:,4)), mean(controlQD(:,5)), mean(controlQD(:,6))];
% % % % % %          QDmodified = [QDmodified ; meanqdot];
% % % % % %         
% % % % % %          
% % % % % %         qi = controlQ(end,:);  % initial q0
% % % % % %         qid = controlQD(end,:);
% % % % % %         dqi = qi;
% % % % % %         dqid = qid;
% % % % % %         dqidd = zeros(size(qi));        
% % % % % % 
% % % % % % end 
% % % % % % 
% % % % % % 
% % % % % % figure(6);
% % % % % % qplot(totalT, totalQ);
% % % % % % xlabel('time (s)')
% % % % % % ylabel('angle (rad)')
% % % % % % title('2.1 jtraj joint angle');
% % % % % % 
% % % % % % figure(7);
% % % % % % qplot(totalT, totalQD);
% % % % % % xlabel('time (s)')
% % % % % % ylabel('velocity (rad/s)')
% % % % % % title('2.1 jtraj joint velocity');
% % % % % % 
% % % % % % 
% % % % % % %__________________________________________________________________
% % % % % % % original, too much fluctuation, use the modified ones 
% % % % % % 
% % % % % % % [a,totalQDD,totalQDDD] = calc_derivatives(totalT, totalQD);
% % % % % % 
% % % % % % % figure(8);
% % % % % % % qplot(totalT, totalQDD);
% % % % % % % xlabel('time (s)')
% % % % % % % ylabel('accerleration (rad/(s^2))')
% % % % % % % title('2.1 jtraj joint acceleration');
% % % % % % 
% % % % % % % figure(9);
% % % % % % % qplot(totalT, totalQDDD);
% % % % % % % xlabel('time (s)')
% % % % % % % ylabel('jerks (rad/(s^3))')
% % % % % % % title('2.1 jtraj jerks');
% % % % % % 
% % % % % % %__________________________________________________________________
% % % % % % 
% % % % % % 
% % % % % % [a,modifiedQDD,modifiedQDDD] = calc_derivatives3(totalT, totalQD);
% % % % % % 
% % % % % % figure(8);
% % % % % % qplot(a, modifiedQDD);
% % % % % % xlabel('time (s)')
% % % % % % ylabel('accerleration (rad/(s^2))')
% % % % % % title('2.1 jtraj joint acceleration');
% % % % % % axis([0 3 -5 5]);
% % % % % % 
% % % % % % figure(9);
% % % % % % qplot(a, modifiedQDDD);
% % % % % % xlabel('time (s)')
% % % % % % ylabel('jerks (rad/(s^3))')
% % % % % % title('2.1 jtraj jerks');
% % % % % % axis([0 3 -5 5]);
% % % % % % 
% % % % % % % plot the a locus(the trajectory of the end effector)
% % % % % % TendEff = irb_120.fkine(totalQ);
% % % % % % spacePoint = transl(TendEff); % obtain the translation point of the end effector from the matrix
% % % % % % figure(10);
% % % % % % plot3(spacePoint(:,1),spacePoint(:,2),spacePoint(:,3));
% % % % % % xyzlabel;
% % % % % % title('2.1 Locus of end effector');
% % % % % % grid on;
% % % % % % 
% % % % % % % root mean square 
% % % % % % % Q 
% % % % % % rootMeanSqure = sqrt((Q1_2 - Qmodified).^2/605); 
% % % % % % figure(11); 
% % % % % % qplot(T1_2,rootMeanSqure)
% % % % % % xlabel('time');
% % % % % % ylabel('error (rad)');
% % % % % % title('2.1 jtraj root mean square of pose');


%% 2.2

% % % % % % % totalStep = 605;  % to make number of time is the same the time of controller
% % % % % % % [Q,QD,QDD] = mtraj(@lspb, q1, q2, totalStep);
% % % % % % % % time = linspace(0, 3, steps); 
% % % % % % % % [a,b,QDDD ] = calc_derivatives(time', QD);
% % % % % % % step = 3/totalStep;
% % % % % % % 
% % % % % % % totalT = 0; 
% % % % % % % totalQ = zeros(1,6); 
% % % % % % % totalQD = zeros(1,6);
% % % % % % % 
% % % % % % % % modified variables for plotting 
% % % % % % % Qmodified = Q(1,:);
% % % % % % % QDmodified = zeros(1,6); 
% % % % % % % 
% % % % % % % qi = q1;  % initial q0
% % % % % % % qid = zeros(1,6); 
% % % % % % % dqi = qi;
% % % % % % % dqid = zeros(size(qi));
% % % % % % % dqidd = zeros(size(qi));
% % % % % % % 
% % % % % % % for i = 1 : totalStep-1
% % % % % % %     
% % % % % % %         DesireQ = Q(i+1,:);
% % % % % % %         DesireQD = QD(i+1,:);
% % % % % % %         Torque0 = irb_120.rne(DesireQ,DesireQD,zeros(1,6)); 
% % % % % % %         
% % % % % % %         [controlT,controlQ,controlQD] = irb_120.fdyn(step, @irb120_p_controller2, qi, qid, dqi, dqid, dqidd);
% % % % % % %         
% % % % % % %         if i == 1 
% % % % % % %             
% % % % % % %             totalT = controlT; 
% % % % % % %             totalQ = controlQ;
% % % % % % %             totalQD = controlQD;
% % % % % % %             
% % % % % % %         else
% % % % % % %             
% % % % % % %              totalT = [totalT ; totalT(end,1)+ controlT];
% % % % % % %              totalQ = [totalQ ; controlQ];
% % % % % % %              totalQD = [totalQD ; controlQD]; 
% % % % % % %             
% % % % % % %         end
% % % % % % %             
% % % % % % %         % modify Q QD Qdd for root mean square
% % % % % % %          Qmodified = [Qmodified ; controlQ(end,:)];
% % % % % % %          
% % % % % % %          meanqdot = [mean(controlQD(:,1)), mean(controlQD(:,2)), mean(controlQD(:,3)), mean(controlQD(:,4)), mean(controlQD(:,5)), mean(controlQD(:,6))];
% % % % % % %          QDmodified = [QDmodified ; meanqdot];
% % % % % % %         
% % % % % % %          
% % % % % % %         qi = controlQ(end,:);  % initial q0
% % % % % % %         qid = controlQD(end,:);
% % % % % % %         dqi = qi;
% % % % % % %         dqid = qid;
% % % % % % %         dqidd = zeros(size(qi));        
% % % % % % % 
% % % % % % % end 
% % % % % % % 
% % % % % % % 
% % % % % % % figure(12);
% % % % % % % qplot(totalT, totalQ);
% % % % % % % xlabel('time (s)')
% % % % % % % ylabel('angle (rad)')
% % % % % % % title('2.2 mtraj joint angle');
% % % % % % % 
% % % % % % % figure(13);
% % % % % % % qplot(totalT, totalQD);
% % % % % % % xlabel('time (s)')
% % % % % % % % ylabel('velocity (rad/s)')
% % % % % % % title('2.2 mtraj joint velocity');
% % % % % % % 
% % % % % % % 
% % % % % % % [a,modifiedQDD,modifiedQDDD] = calc_derivatives3(totalT, totalQD);
% % % % % % % 
% % % % % % % figure(14);
% % % % % % % qplot(a, modifiedQDD);
% % % % % % % xlabel('time (s)')
% % % % % % % ylabel('accerleration (rad/(s^2))')
% % % % % % % title('2.2 mtraj joint acceleration');
% % % % % % % axis([0 3 -5 5]);
% % % % % % % 
% % % % % % % figure(15);
% % % % % % % qplot(a, modifiedQDDD);
% % % % % % % xlabel('time (s)')
% % % % % % % ylabel('jerks (rad/(s^3))')
% % % % % % % title('2.2 mtraj jerks');
% % % % % % % axis([0 3 -5 5]);
% % % % % % % 
% % % % % % % % plot the a locus(the trajectory of the end effector)
% % % % % % % TendEff = irb_120.fkine(totalQ);
% % % % % % % spacePoint = transl(TendEff); % obtain the translation point of the end effector from the matrix
% % % % % % % figure(16);
% % % % % % % plot3(spacePoint(:,1),spacePoint(:,2),spacePoint(:,3));
% % % % % % % xyzlabel;
% % % % % % % title('2.2 mtraj Locus of end effector q1 to q2');
% % % % % % % grid on;
% % % % % % % 
% % % % % % % 
% % % % % % % % root mean square 
% % % % % % % % Q 
% % % % % % % rootMeanSqure = sqrt((Q1_2 - Qmodified).^2/605); 
% % % % % % % figure(17); 
% % % % % % % qplot(T1_2,rootMeanSqure)
% % % % % % % xlabel('time');
% % % % % % % ylabel('error (rad)');
% % % % % % % title('2.2 mtraj root mean square of pose q1 to q2');



%% 2.3

% ctraj is straight line motion
totalStep = 605; % to make number of time is the same the time of controller
T1 = irb_120.fkine(q1);
T2 = irb_120.fkine(q2);
Tgroup = ctraj(T1, T2, totalStep);
QGroup = irb_120.ikunc(Tgroup);    % ikunc is the another method for inverse transfermation matrix without the joint limit

initialPoint = transl(T1) 
endPoint = transl(T2)

Q = QGroup;
time = linspace(0, 3, totalStep); 
[a,QD,QDD ] = calc_derivatives(time', Q);



step = 3/totalStep;

totalT = 0; 
totalQ = zeros(1,6); 
totalQD = zeros(1,6);

% modified variables for plotting 
Qmodified = Q(1,:);
QDmodified = zeros(1,6); 

qi = Q(1,:);  % initial 
qid = zeros(1,6); 
dqi = qi;
dqid = zeros(size(qi));
dqidd = zeros(size(qi));

for i = 1 : totalStep-1
    
        DesireQ = Q(i+1,:);
        DesireQD = QD(i+1,:);
        Torque0 = irb_120.rne(DesireQ,DesireQD,zeros(1,6)); 
        
        [controlT,controlQ,controlQD] = irb_120.fdyn(step, @irb120_p_controller2, qi, qid, dqi, dqid, dqidd);
        
        if i == 1 
            
            totalT = controlT; 
            totalQ = controlQ;
            totalQD = controlQD;
            
        else
            
             totalT = [totalT ; totalT(end,1)+ controlT];
             totalQ = [totalQ ; controlQ];
             totalQD = [totalQD ; controlQD]; 
            
        end
            
        % modify Q QD Qdd for root mean square
         Qmodified = [Qmodified ; controlQ(end,:)];
         
         meanqdot = [mean(controlQD(:,1)), mean(controlQD(:,2)), mean(controlQD(:,3)), mean(controlQD(:,4)), mean(controlQD(:,5)), mean(controlQD(:,6))];
         QDmodified = [QDmodified ; meanqdot];
        
         
        qi = controlQ(end,:);  % initial 
        qid = controlQD(end,:);
        dqi = qi;
        dqid = qid;
        dqidd = zeros(size(qi));        

end 


figure(18);
qplot(totalT, totalQ);
xlabel('time (s)')
ylabel('angle (rad)')
title('2.3 ctraj joint angle q1 to q2');

figure(19);
qplot(totalT, totalQD);
xlabel('time (s)')
ylabel('velocity (rad/s)')
title('2.3 ctraj joint velocity q1 to q2');

[a,modifiedQDD,modifiedQDDD] = calc_derivatives3(totalT, totalQD);

figure(20);
qplot(a, modifiedQDD);
xlabel('time (s)')
ylabel('accerleration (rad/(s^2))')
title('2.3 ctraj joint acceleration');
axis([0 3 -5 5]);

figure(21);
qplot(a, modifiedQDDD);
xlabel('time (s)')
ylabel('jerks (rad/(s^3))')
title('2.3 ctraj jerks');
axis([0 3 -5 5]);

% plot the a locus(the trajectory of the end effector)

TendEff = irb_120.fkine(totalQ); 
spacePoint = transl(TendEff); % obtain the translation point of the end effector from the matrix
figure(22);
plot3(spacePoint(:,1),spacePoint(:,2),spacePoint(:,3));
xyzlabel;
title('2.3 ctraj Locus of end effector q1 to q2');
grid on;
axis([0.1 0.2 -0.6 0.1 0.1 0.2]);

% root mean square 
% Q 
rootMeanSqure = sqrt((Q1_2 - Qmodified).^2/605); 
figure(23); 
qplot(T1_2,rootMeanSqure)
xlabel('time');
ylabel('error (rad)');
title('2.3 ctraj root mean square of pose q1 to q2');



%% 2.5
% 
% % multi segment trajectory 
% via = [q2; q4; q3; q1];
% Q = mstraj(via, [], [3 3 3 3], q1, 0.1, 0);
% 
% % plot the a locus(the trajectory of the end effector)
% TendEff = irb_120.fkine(Q); 
% spacePoint = transl(TendEff); % obtain the translation point of the end effector from the matrix
% figure(24);
% plot3(spacePoint(:,1),spacePoint(:,2),spacePoint(:,3));
% xyzlabel;
% title('2.5 mstraj Locus of end effector q1-q2-q4-q3-q1');
% grid on;
% % axis([0.1 0.2 -0.6 0.1 0.1 0.2]);
% 
% 
% 
% % use controller 
% 
% % q1 to q2
% DesireQ = q2;
% DesireQD = zeros(1,6);
% Torque0 = irb_120.rne(DesireQ,DesireQD,zeros(1,6)); 
% simTime = 3;
% [controlT1,controlQ1,controlQD1] = irb_120.fdyn(simTime, @irb120_p_controller2, q1, zeros(1,6));
% 
% % q2 to q4
% DesireQ = q4;
% DesireQD = zeros(1,6);
% Torque0 = irb_120.rne(DesireQ,DesireQD,zeros(1,6)); 
% simTime = 3;
% [controlT2,controlQ2,controlQD2] = irb_120.fdyn(simTime, @irb120_p_controller2, q2, zeros(1,6));
% 
% % q4 to q3
% DesireQ = q3;
% DesireQD = zeros(1,6);
% Torque0 = irb_120.rne(DesireQ,DesireQD,zeros(1,6)); 
% simTime = 3;
% [controlT3,controlQ3,controlQD3] = irb_120.fdyn(simTime, @irb120_p_controller2, q4, zeros(1,6));
% 
% % q3 to q1
% DesireQ = q1;
% DesireQD = zeros(1,6);
% Torque0 = irb_120.rne(DesireQ,DesireQD,zeros(1,6)); 
% simTime = 3;
% [controlT4,controlQ4,controlQD4] = irb_120.fdyn(simTime, @irb120_p_controller2, q3, zeros(1,6));
% 
% 
% controlT = [controlT1; controlT2 + 3; controlT3 + 6; controlT4 + 9];
% controlQ = [controlQ1; controlQ2; controlQ3; controlQ4];
% 
% 
% a1 = irb_120.fkine(q1);
% a1p = transl(a1);
% 
% a2 = irb_120.fkine(q2);
% a2p = transl(a2);
% 
% a3 = irb_120.fkine(q3);
% a3p = transl(a3);
% 
% a4 = irb_120.fkine(q4);
% a4p = transl(a4);
% 
% 
% % plot the a locus(the trajectory of the end effector)
% TendEff = irb_120.fkine(controlQ); 
% spacePoint = transl(TendEff); % obtain the translation point of the end effector from the matrix
% figure(25); hold on; 
% plot3(a1p(1,1),a1p(2,1),a1p(3,1),'r*');
% plot3(a2p(1,1),a2p(2,1),a2p(3,1),'r*');
% plot3(a3p(1,1),a3p(2,1),a3p(3,1),'r*');
% plot3(a4p(1,1),a4p(2,1),a4p(3,1),'r*');
% 
% plot3(spacePoint(:,1),spacePoint(:,2),spacePoint(:,3));
% xyzlabel;
% title('2.5 controller Locus of end effector q1-q2-q4-q3-q1');
% grid on;
% 
% 
% % totalStep = 605;  % to make number of time is the same the time of controller
% % [Q,QD,QDD] = jtraj(q1, q2, totalStep); 
% % % time = linspace(0, 3, steps); 
% % % [a,b,QDDD ] = calc_derivatives(time', QD);
% % step = 3/totalStep;
% 
% totalT = 0; 
% totalQ = zeros(1,6); 
% totalQD = zeros(1,6);
% 
% % modified variables for plotting 
% Qmodified = Q(1,:);
% QDmodified = zeros(1,6); 
% 
% qi = q1;  % initial q0
% qid = zeros(1,6); 
% dqi = qi;
% dqid = zeros(size(qi));
% dqidd = zeros(size(qi));
% 
% for i = 1 : totalStep-1
%     
%         DesireQ = Q(i+1,:);
%         DesireQD = QD(i+1,:);
%         Torque0 = irb_120.rne(DesireQ,DesireQD,zeros(1,6)); 
%         
%         [controlT,controlQ,controlQD] = irb_120.fdyn(step, @irb120_p_controller2, qi, qid, dqi, dqid, dqidd);
%         
%         if i == 1 
%             
%             totalT = controlT; 
%             totalQ = controlQ;
%             totalQD = controlQD;
%             
%         else
%             
%              totalT = [totalT ; totalT(end,1)+ controlT];
%              totalQ = [totalQ ; controlQ];
%              totalQD = [totalQD ; controlQD]; 
%             
%         end
%             
%         % modify Q QD Qdd for root mean square
%          Qmodified = [Qmodified ; controlQ(end,:)];
%          
%          meanqdot = [mean(controlQD(:,1)), mean(controlQD(:,2)), mean(controlQD(:,3)), mean(controlQD(:,4)), mean(controlQD(:,5)), mean(controlQD(:,6))];
%          QDmodified = [QDmodified ; meanqdot];
%         
%          
%         qi = controlQ(end,:);  % initial q0
%         qid = controlQD(end,:);
%         dqi = qi;
%         dqid = qid;
%         dqidd = zeros(size(qi));        
% 
% end 
% 
% 
% 
% figure(16);
% qplot(time, Q);
% xlabel('time (s)')
% ylabel('angle (rad)')
% title('2.4 jtraj joint angle');
% 
% figure(17);
% qplot(time, QD);
% xlabel('time (s)')
% ylabel('velocity (rad/s)')
% title('2.4 jtraj joint velocity');
% 
% figure(18);
% qplot(time, QDD);
% xlabel('time (s)')
% ylabel('accerleration (rad/(s^2))')
% title('2.4 jtraj joint acceleration');
% 
% figure(19);
% qplot(time, QDDD);
% xlabel('time (s)')
% ylabel('jerks (rad/(s^3))')
% title('2.4 jtraj jerks');
% 
% % plot the a locus(the trajectory of the end effector)
% TendEff = irb_120.fkine(Q);
% spacePoint = transl(TendEff); % obtain the translation point of the end effector from the matrix
% figure(15);
% plot3(spacePoint(:,1),spacePoint(:,2),spacePoint(:,3));
% xyzlabel;
% title('2.4 Locus of end effector q1 to q2');
% grid on;










