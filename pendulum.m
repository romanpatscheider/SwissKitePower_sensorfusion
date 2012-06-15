function dy=pendulum(t,y)
g=9.81;

%dy = zeros(8,1);    % a column vector

% Model from Roman
% dy(1) = sin(y(3))*cos(y(3))*y(2)^2-g/l*sin(y(3)); % theta ..
% dy(2) = -2*cos(y(3))*y(1)*y(2); % phi ..
% dy(3) = y(1); % theta .
% dy(4) = y(2); % phi .

% My Model with rigid string 
% y= [phi theta psi r dphi dtheta dpsi dr]

dy(6) = g/y(4)*cos(y(2))-sin(y(2))*cos(y(2))*y(7)^2;
dy(7) = 2*tan(y(2))*y(6)*y(7);
dy(5) = -(-cos(y(2))*y(6)*y(7)-sin(y(2))*dy(7));%0;
dy(8) = 0;
dy(1) = y(5);
dy(2) = y(6);
dy(3) = y(7);
dy(4) = 0;
dy=transp(dy);


