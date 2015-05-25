function [ dQt, ddQtf, dddQtf ] = calc_derivatives(tsim, dQt)
% CALC_DERIVATIVES calculates acceleration and jerk using the velocity
% vector returned by fdyn. Low pass filtering is used to remove noise.
%
% Inputs
% tsim      time vector returned by fdyn
% dQt       joint velocity vector returned by fdyn
%
% Outputs
% dQtf      filtered joint velocity vector
% ddQtf     filtered joint acceleration vector
% dddQtf    filtered joint jerk vector
%

    % Number of robot links
    N       = size(dQt,2);
    
    % Initialise acceleration and jerk vectors
    ddQtf   = zeros(size(dQt));
    dddQtf  = zeros(size(dQt));
    
    % Filter velocity vector:
%     dQtf = lp_filter(tsim, dQt, 5e2);
    
    
    % Calculate acceleration:
    for j=1:N
        ddQtf(:,j) = diff( [0; dQt(:,j)] )./diff([0; tsim]);
    end
    % Filter acceleration vector:
%     ddQtf = lp_filter(tsim, ddQtf, 5e2);
    
    % Calculate jerk:
    for j=1:N
        dddQtf(:,j) = diff( [0; ddQtf(:,j)] )./diff([0; tsim]);
    end    
    % Filter jerk vector:
%     dddQtf = lp_filter(tsim, dddQtf, 5e2);
    
end % CALC_DERIVATIVES


function yfilt = lp_filter(t, y, wc)
% LP_FILTER implements a 1st order low pass filter, discretized by a Tustin
% approximation. Works with variable time steps.
%
% Inputs
% t         time vector that can contain unevenly spaced time steps [sec]
% y         signal to be filtered
% wc        cutt off frequency [rad/sec]
%
% Outputs
% yfilt     filtered signal
%

    % Initialise output
    yfilt = zeros(size(y));
    
    % Filtering
    for i=3:length(t)
       % Get time steps
       Ts0 = t(i)   - t(i-1);
       Ts1 = t(i-1) - t(i-2);
       % Calculate transfer function coefficients
       b0 = (Ts0*wc)/(Ts0*wc + 2);
       b1 = (Ts1*wc)/(Ts1*wc + 2);
       a1 = (Ts1*wc - 2)/(Ts1*wc + 2);
       % Calculate filtered output
       yfilt(i,:) = b0.*y(i,:) + b1.*y(i-1,:) - a1.*yfilt(i-1,:);
    end

end % LP_FILTER

