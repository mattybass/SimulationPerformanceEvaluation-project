function [position] = leastSquaresMethod(beacon_pos,d)
    d=reshape(d,[5,1]);
    pos_x = beacon_pos(:,1);
    pos_y = beacon_pos(:,2);
    n = size(pos_x,1);
    A=[2.*(pos_x(1:n-1,:)-pos_x(n)),2.*(pos_y(1:n-1,:)-pos_y(n))];
    B= pos_x(1:n-1,:).^2 - pos_x(n).^2 + pos_y(1:n-1,:).^2 - pos_y(n).^2 + d(n).^2-d(1:n-1,:).^2; 
    position = ((A/(A'*A))' * B)'; %equal to (inv(A' * A)* A' * B)';
end

