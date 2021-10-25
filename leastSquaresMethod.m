function [position] = leastSquaresMethod(beacon_pos,d)
pos_x = beacon_pos(:,1);
pos_y = beacon_pos(:,2);
n = size(pos_x,1);
W=zeros(n,n);
for i=1:(n-1)
       A(i,1) = 2*(pos_x(i) - (pos_x(n))); 
       A(i,2) = 2*(pos_y(i) - (pos_y(n)));  
       B(i,1) = pos_x(i).^2 - pos_x(n).^2 + pos_y(i).^2 - pos_y(n).^2 + d(n).^2-d(i).^2; 
end

    %W=1/sqrt(alfa.^2+beta.^2);
    %alfa=sum
    W=eye(n-1);
    position = (inv(A' * A)* A' * B)';


end

