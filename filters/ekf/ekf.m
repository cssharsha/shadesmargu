function [ mu,sigma ] = ekf( mu,sigma,u,z,deltaT,puflag )
% Applies the kallman filter at each step when called.
%   An addition flag puflag is used 

R = [0.01 0 0;0 0.01 0;0 0 0.001];
Q = [0.001 0 0;0 0.001 0;0 0 0.001];
G = [1 0 (u(1)*deltaT*(-sin(mu(3))));0 1 (u(1)*deltaT*cos(mu(3)));0 0 1];
mubar = [(mu(1) + u(1)*deltaT*cos(mu(3))) (mu(2) + u(1)*deltaT*sin(mu(3))) (mu(3) + u(2)*deltaT)];
mubar = mubar';
sigmabar = G*sigma*G' + R;
% Should do only predict step.
if puflag == 0
    mu = mubar;
    sigma = sigmabar;
    fprintf("Returning with only prediction\n");
% Should perform update step as well.
else
    H(1,1) = (2*(mubar(1))-10)/(2*sqrt((mubar(1)-5)^2+(mubar(2)-5)^2));
    H(1,2) = (2*(mubar(2))-10)/(2*sqrt((mubar(1)-5)^2+(mubar(2)-5)^2));
    H(1,3) = 0;
    H(2,1) = (2*(mubar(1))-8)/(2*sqrt((mubar(1)-4)^2+(mubar(2)-7)^2));
    H(2,2) = (2*(mubar(2))-14)/(2*sqrt((mubar(1)-4)^2+(mubar(2)-7)^2));
    H(2,3) = 0;
    H(3,1) = (2*(mubar(1))+6)/(2*sqrt((mubar(1)+3)^2+(mubar(2)-2)^2));
    H(3,2) = (2*(mubar(2))-4)/(2*sqrt((mubar(1)+3)^2+(mubar(2)-2)^2));
    H(3,3) = 0;
    K = (sigmabar*H')/((H*sigmabar*H')+Q);
    h_mu = [sqrt((mubar(1)-5)^2+(mubar(2)-5)^2) sqrt((mubar(1)-4)^2+(mubar(2)-7)^2) sqrt((mubar(1)+3)^2+(mubar(2)-2)^2)];
    h_mu = h_mu';
    mu = mubar+(K*(z-h_mu));
    sigma = sigmabar - K*H*sigmabar;
    fprintf("Returning with update performed!\n");
end

end

