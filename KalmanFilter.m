"""
The follwoing four sections outline how an airplane, flying in a two-dimensional plane, can have variations in trajecotry based on noise and the introduction of a Kalman filter
"""



"""
The next two functions implement airplane dynamics using a linear system, suitable for use in a Kalman filter - and implenment a simple policy
"""
function KalmanFilterOne
    %Build Mat.
    F = [1, 0, 1, 0; 0, 1, 0, 1; 0, 0, 1, 0;0, 0, 0, 1];
    B = [0,0; 0,0; 1,0; 0,1];
    initState = [0,0,0,0];
    currTime = 1;
    currState = initState;
    maxTime = 2000;
    traj = zeros(4, maxTime);
    traj(:, 1) = currState;
    
    while currTime <= maxTime
        [uX, uY] = policy(currTime, currState);
        u = [uX; uY];
        currState = F*currState + B*u;
        currTime = currTime + 1;
        traj(:, currTime) = currState;
    end
    
         figure();
         plot(traj(1,:), traj(2,:));
    
end

%Policy of a sine function implemented
function [uX, uY] = policy(time, state)
    if state(3) < 60
        uX = 6;
    else
        uX = 0;
    end
    
    uY = sin(time/60);
end


"""
The following two functions add Gaussian noise to the control and plot trajectory
"""
function KalmanFilterTwo
    F = [1, 0, 1, 0; 0, 1, 0, 1; 0, 0, 1, 0;0, 0, 0, 1];
    B = [0,0; 0,0; 1,0; 0,1];
    initState = [0,0,0,0];
    traj = zeros(4, maxTime);
    traj(:, 1) = currState;
    maxTime = 2000;
    currTime = 1;
    currState = initState;
    
    
    while currTime <= maxTime
        [uX, uY] = policy(currTime, currState);
        u = [uX; uY];        
        currState = F*currState + B*u + mvnrnd([0;0;0;0], eye(4));
        currTime = currTime + 1;        
        traj(:, currTime) = currState;
    end
    
% %     figure();
% %     
% %     plot(traj(1,:), traj(2,:));    
end

%Policy of a sine function implemented
function [uX, uY] = policy(time, state)
    if state(3) < 60
        uX = 6;
    else
        uX = 0;
    end
    uY = sin(time/60);
end

"""
Creates a noisy observation function by adding Gaussian noise to the airplane position and velocity
"""
function KalmanFilterThree
    F = [1, 0, 1, 0; 0, 1, 0, 1; 0, 0, 1, 0;0, 0, 0, 1];
    B = [0,0; 0,0; 1,0; 0,1];
    H = eye(4);
    R = eye(4)*500000;
    
    initState = [0,0,0,0];
    currTime = 1;
    currState = initState;
    currObs = noisyObservation(currState, R, H);
    maxTime = 1000;
    
    traj = zeros(4, maxTime);
    obsTraj = zeros(4, maxTime);
    traj(:, 1) = currState;
    obsTraj(:, currTime) = currObs;
    while currTime <= maxTime
        [uX, uY] = policy(currTime, currObs);
        u = [uX; uY];
        
        %Take action-update; gaussian noise added
        currState = F*currState + B*u + mvnrnd([0;0;0;0], eye(4));
        currObs = noisyObservation(currState, R, H);
        currTime = currTime + 1;
        
        traj(:, currTime) = currState;
        obsTraj(:, currTime) = currObs;
        
    end
    
% %     figure();
% %     plot(traj(1,:), traj(2,:));
% %     hold on
% %     plot(obsTraj(1,:), obsTraj(2,:));
% %     legend('True', 'Observed');    
end

function [uX, uY] = policy(time, state)
    if state(3) < 60
        uX = 6;
    else
        uX = 0;
    end
    
    uY = sin(time/60);
    
end

function noisyObs = noisyObservation(state, R, H)
    noisyObs = H*state+mvnrnd([0,0,0,0],R);
end

"""
The next three functions implement a Kalman filter and show the estimated trajectory of the plane overlaid on its true trajectory (in addition to the noisy observation)
"""
function KalmanFilterFour
    maxTime = 2000; %Sim Time
    t = 1;
    
    F = [1, 0, 1, 0; 0, 1, 0, 1; 0, 0, 1, 0;0, 0, 0, 1];
    B = [0,0; 0,0; 1,0; 0,1];
    H = eye(4);
    Q = eye(4);
    R = eye(4)*500000;

    %Initial states and observations
    states(:, t) = [0,0,0,0];
    observations(:, t) = H * states(:,t) + mvnrnd([0,0,0,0],R);

    %Track Variables
    states = zeros(4, maxTime);
    observations = zeros(4, maxTime);
    
    predictedStates(:,t) = states(:,t);
    predictedCovariance(:,t).R = R;
    
    %Simulation loop
    while t <= maxTime

        %next action
        [uX, uY] = policy(t, observations(:,t));
        ut1 = [uX; uY];
        
        %Kalman Variables
        states(:,t+1) = F * states(:,t) + B * ut1 + mvnrnd([0;0;0;0], Q);
        observations(:,t+1) = H * states(:,t+1) + mvnrnd([0,0,0,0], R);
        
        xp = F * predictedStates(:,t) + B * ut1;
        pp = F * predictedCovariance(:, t).R * F + Q;
        
        [x, p] = kalmanUpdate(R, H, xp, pp, observations(:,t+1));
        predictedStates(:,t+1) = x;
        predictedCovariance(:, t+1).R = p;
        
        
        t = t + 1; 
    end
    
    % %     figure();
    % %     subplot(1,3,1)
    % %     plot(states(1,:), states(2,:));
    % %     subplot(1,3,2)
    % %     plot(observations(1,:), observations(2,:));
    % %     subplot(1,3,3)
    % %     plot(predictedStates(1,:), predictedStates(2,:));
    % %
    % %     figure();
    % %     hold on
    % %     plot(predictedStates(1,:), predictedStates(2,:));
    % %     plot(states(1,:), states(2,:));
    % %     plot(observations(1,:), observations(2,:));
end

function [estState,estCov] = kalmanUpdate(R,H,xp,pp,observation)
    y = observation - H * xp;
    S = H * pp * H + R;
    K = pp * H * inv(S);
    
    estState = xp + K * y;
    estCov = (eye(4) - K * H) * pp;
end

function [uX, uY] = policy(time,state)
    if state(3) < 60
        uX = 6;
    else
        uX = 0;
    end
    
    uY = sin(time/60);
end

