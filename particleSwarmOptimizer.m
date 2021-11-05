function [position] = particleSwarmOptimizer(beacon_pos,d,position_pre,print)
s = size(beacon_pos,1);
%rng default  % For reproducibility
beta=2;
vect_fit=cell(1,s);
for i=1:s
    vect_fit{i}{1}=beacon_pos(i,:);
    vect_fit{i}{2}=d(i);
end

%fitness_1=@(pos) sum( cellfun( @(arg) ((arg{2}-norm(pos-arg{1}))^2) ,vect_fit));
%{
fitness1=@(x,y) vect_fit{1}{2}-vectnorm([x,y]-vect_fit{1}{1},2,2).^2+...
                vect_fit{2}{2}-vectnorm([x,y]-vect_fit{2}{1},2,2).^2+...
                vect_fit{3}{2}-vectnorm([x,y]-vect_fit{3}{1},2,2).^2+...
                vect_fit{4}{2}-vectnorm([x,y]-vect_fit{4}{1},2,2).^2+...
                vect_fit{5}{2}-vectnorm([x,y]-vect_fit{5}{1},2,2).^2;
            %}
        
fitness_2d=@(x,y)...
                abs(vect_fit{1}{2}-sqrt((x-vect_fit{1}{1}(1)).^2+(y-vect_fit{1}{1}(2)).^2))+...
                abs(vect_fit{2}{2}-sqrt((x-vect_fit{2}{1}(1)).^2+(y-vect_fit{2}{1}(2)).^2))+...
                abs(vect_fit{3}{2}-sqrt((x-vect_fit{3}{1}(1)).^2+(y-vect_fit{3}{1}(2)).^2))+...
                abs(vect_fit{4}{2}-sqrt((x-vect_fit{4}{1}(1)).^2+(y-vect_fit{4}{1}(2)).^2))+...
                abs(vect_fit{5}{2}-sqrt((x-vect_fit{5}{1}(1)).^2+(y-vect_fit{5}{1}(2)).^2));
            %{
fitness=@(pos) ((vect_fit{1}{2}-sqrt((pos(1)-vect_fit{1}{1}(1)).^2+(pos(2)-vect_fit{1}{1}(2)).^2)).^2+...
                (vect_fit{2}{2}-sqrt((pos(1)-vect_fit{2}{1}(1)).^2+(pos(2)-vect_fit{2}{1}(2)).^2)).^2+...
                (vect_fit{3}{2}-sqrt((pos(1)-vect_fit{3}{1}(1)).^2+(pos(2)-vect_fit{3}{1}(2)).^2)).^2+...
                (vect_fit{4}{2}-sqrt((pos(1)-vect_fit{4}{1}(1)).^2+(pos(2)-vect_fit{4}{1}(2)).^2)).^2+...
                (vect_fit{5}{2}-sqrt((pos(1)-vect_fit{5}{1}(1)).^2+(pos(2)-vect_fit{5}{1}(2)).^2)).^2);
%}
fitness=@(pos) sum( cellfun( @(arg)...
                   (arg{2}- sqrt( (pos(1)-arg{1}(1)).^2  +  (pos(2)-arg{1}(2)).^2 )).^2 ...
                   ,vect_fit));

nvars = 2;
options = optimoptions('particleswarm');
options.Display='off';
%options.InitialSwarmMatrix=position_pre;
options.InertiaRange=[0.4,1];
options.SelfAdjustmentWeight=2;
options.SocialAdjustmentWeight=0.5;
%[position_pre(1)-beta,position_pre(2)-beta],[position_pre(1)+beta,position_pre(2)+beta]
lb=[0,0];
up=[20,10];
position = particleswarm(fitness,nvars,lb,up,options);

    if (print==1)
        figure()
        [x,y] = meshgrid(position_pre(1)-beta:0.05:position_pre(1)+beta,position_pre(2)-beta:0.05:position_pre(2)+beta);
        surf(x,y,fitness_2d(x,y));
        hold on;
        scatter3(position(1),position(2),fitness([position(1),position(2)]));
        hold on; scatter3(position_pre(1),position_pre(2),fitness([position_pre(1),position_pre(2)]));
    end
end

