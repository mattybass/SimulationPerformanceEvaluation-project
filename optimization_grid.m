function [position_OPT] = optimization_grid(beacon_pos,d,position_pre)
%% Uniform sampling
s = size(beacon_pos,1);
vect_fit=cell(1,s);
beta=0.5;

for i=1:s
    vect_fit{i}{1}=beacon_pos(i,:);
    vect_fit{i}{2}=d(i);
end
         
fitness_2d=@(x,y)...
                abs(vect_fit{1}{2}-sqrt((x-vect_fit{1}{1}(1)).^2+(y-vect_fit{1}{1}(2)).^2))^2+...
                abs(vect_fit{2}{2}-sqrt((x-vect_fit{2}{1}(1)).^2+(y-vect_fit{2}{1}(2)).^2))^2+...
                abs(vect_fit{3}{2}-sqrt((x-vect_fit{3}{1}(1)).^2+(y-vect_fit{3}{1}(2)).^2))^2+...
                abs(vect_fit{4}{2}-sqrt((x-vect_fit{4}{1}(1)).^2+(y-vect_fit{4}{1}(2)).^2))^2+...
                abs(vect_fit{5}{2}-sqrt((x-vect_fit{5}{1}(1)).^2+(y-vect_fit{5}{1}(2)).^2))^2;


thick=1000;
%PSO_x=linspace(position_pre(1)-beta,position_pre(1)+beta,thick);
%PSO_y=linspace(position_pre(2)-beta,position_pre(2)+beta,thick);
%f_mat=zeros(size(PSO_x,2),size(PSO_y,2));
%{
for k=1:s
    for i=1:size(PSO_x,2)
        for j=1:size(PSO_y,2)
            %f_mat(i,j)=f_mat(i,j)+(d(k)-norm([PSO_x(i),PSO_y(j)]-beacon_pos(k,:)))^2;
            f_mat=fitness([PSO_x,PSO_y]);
        end
    end
end
%}
[x,y] = meshgrid(position_pre(1)-beta:0.01:position_pre(1)+beta,position_pre(2)-beta:0.01:position_pre(2)+beta);
f_mat=fitness_2d(x,y);

[x_index,y_index]=find(f_mat==min(f_mat,[],'all'));
position_OPT=[position_pre(1)-beta+2*beta/thick*x_index;position_pre(2)-beta+2*beta/thick*y_index];

end

