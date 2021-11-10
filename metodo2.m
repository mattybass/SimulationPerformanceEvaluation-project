clear 
tic
%% MODEL
space.x=0:0.01:20;
space.y=0:0.01:10;
[space.x_grid,space.y_grid]=meshgrid(space.x,space.y);


% formulas
model.RSSI=@(A,n,d) A-10*n*log10(d); 
model.computeDist=@(A,RSSI,n) 10.^((A-RSSI)./(10.*n));
model.nvars = 2;
model.chess_lenght=100;

% RSSI
model.n=2;
model.A=-50;

% shadowing
model.shadowing.mu=0;
model.shadowing.sigma=4;

% fading
model.fading.mu=1;

% noise
model.noise.mu=0;
model.noise.sigma=1;

% PSO
model.options = optimoptions('particleswarm');
model.options.Display='off';
model.options.InertiaRange=[0.4,1];
model.options.SelfAdjustmentWeight=2;
model.options.SocialAdjustmentWeight=0.5;
%options.InitialSwarmMatrix=position_pre;

%% BEACON
beacon.position=[3.05,4.05; 8.05,2.05; 9.05,3.05; 7.05,2.05; 1.05, 3.55];
beacon.used=[3 5];
space.beacon_size=size(beacon.position,1);

%%
    measure.index_x=find(mod(space.x,1)==0); %point to take RSSI
    measure.index_y=find(mod(space.y,1)==0);
    [measure.grid_y,measure.grid_x]=meshgrid(space.y(measure.index_y),space.x(measure.index_x));
    
for i=1:space.beacon_size
    dist_X=(beacon.position(i,1)-space.x)'*ones(size(space.y));
    dist_Y=ones(size(space.x))'*(beacon.position(i,2)-space.y);
    beacon.dist(:,:,i)=sqrt(dist_X.^2+dist_Y.^2);
    measure.sampled_dist(:,:,i)=beacon.dist(measure.index_x,measure.index_y,i);% z with array of distances
    
    clear dist_X dist_Y
end
[~,measure.order]=sort(measure.sampled_dist,3);


ntimes=20;
distRSSI_temp=zeros(numel(measure.index_x),numel(measure.index_y),space.beacon_size,ntimes);
shadowing=compute_shadowing(space,model);
for i=1:ntimes
    fading=compute_fading(space,model); 
    distRSSI_temp(:,:,:,i)=compute_distRSSI(beacon,measure,shadowing,fading,model,space);    
end

measure.distRSSI{1}=compute_distRSSI(beacon,measure,shadowing,zeros(size(fading)),model,space);    
measure.distRSSI{2}=mean(distRSSI_temp,4);
%{
for i=1:numel(beacon.used)
    measure.distRSSI_beacon{i}=measure.distRSSI(measure.order(:,:,1:beacon.used(i)));
end
%}
%% LOCALIZATION

position_LSM=cell(1,numel(measure.distRSSI));
position_PSO=cell(1,numel(measure.distRSSI));

for k=1:numel(measure.distRSSI)
    position_LSM{k}=zeros(size(measure.distRSSI{1},1),size(measure.distRSSI{1},2),model.nvars);
    position_PSO{k}=zeros(size(measure.distRSSI{1},1),size(measure.distRSSI{1},2),model.nvars);
    for i=1:size(measure.distRSSI{k},1)
        for j=1:size(measure.distRSSI{k},2)
            position_LSM{k}(i,j,:)=leastSquaresMethod(beacon.position,measure.distRSSI{k}(i,j,:));
            position_PSO{k}(i,j,:)=particleSwarmOptimizer(beacon.position,measure.distRSSI{k}(i,j,:),model);
        end
    end
end

T=toc;

%% EVALUATION
errors.LSM{1}=round(sqrt((position_LSM{1}(:,:,1)-measure.grid_x).^2+(position_LSM{1}(:,:,2)-measure.grid_y).^2),3);
errors.PSO{1}=round(sqrt((position_LSM{1}(:,:,1)-measure.grid_x).^2+(position_LSM{1}(:,:,2)-measure.grid_y).^2),3);

errors.LSM{2}=round(sqrt((position_LSM{2}(:,:,1)-measure.grid_x).^2+(position_LSM{2}(:,:,2)-measure.grid_y).^2),3);
errors.PSO{2}=round(sqrt((position_LSM{2}(:,:,1)-measure.grid_x).^2+(position_LSM{2}(:,:,2)-measure.grid_y).^2),3);

figure();

subplot(2,2,1);
imagesc(errors.LSM{1});
subtitle('LSM with fading');

subplot(2,2,2);
imagesc(errors.PSO{1});
subtitle('PSO with fading');

subplot(2,2,3);
imagesc(errors.LSM{2});
subtitle('LSM without fading');

subplot(2,2,4);
imagesc(errors.PSO{2});
subtitle('PSO without fading');

title('Localization error');

figure();
subplot(2,2,1);
histogram(errors.LSM{1});
subtitle('LSM with fading');

subplot(2,2,2);
histogram(errors.PSO{1});
subtitle('PSO with fading');

histogram(errors.LSM{2});
subplot(2,2,3);
subtitle('LSM without fading');

subplot(2,2,4);
histogram(errors.PSO{2});
subtitle('PSO without fading');
    %errorx=position_LSM(:,:,1)-measure.grid_x;
    %errory=position_LSM(:,:,2)-measure.grid_y;
    
%{
    position_LSM(i,:) = leastSquaresMethod(beacon.position,dist_RSSI);
    position_PSO(i,:)=particleSwarmOptimizer(beacon.position,dist_RSSI,position_LSM(i,:),0);
    position_OPT(i,:)=optimization_grid(beacon.position,dist_RSSI,position_LSM(i,:));
    clear fading
    if(sum(stop_times==i)~=0)
        position_LSM_mean(find(stop_times==i),:)=[mean(position_LSM(:,1)),mean(position_LSM(:,2))];
        position_PSO_mean(find(stop_times==i),:)=[mean(position_PSO(:,1)),mean(position_PSO(:,2))];
        errors(find(stop_times==i),1)=(sum(sqrt( (position_LSM(:,1)-position_real(1)).^2+(position_LSM(:,2)-position_real(2)).^2 )))/size(position_LSM,1);
        errors(find(stop_times==i),2)=(sum(sqrt( (position_PSO(:,1)-position_real(1)).^2+(position_PSO(:,2)-position_real(2)).^2 )))/size(position_PSO,1);
        %position_LSM_errors=sqrt( (position_LSM(:,1)-position_real(1)).^2+(position_LSM(:,2)-position_real(2)).^2 );
        %position_PSO_errors=sqrt( (position_PSO(:,1)-position_real(1)).^2+(position_PSO(:,2)-position_real(2)).^2 );
    end
    
    %{
    %temp=[position_LSM(:,1);position_PSO(:,1)];
    %CI_x=[mean(temp)-1.96*std(temp)/mean(temp),mean(temp)+1.96*std(temp)/sqrt(numel(temp))];
    %temp=[position_LSM(:,2);position_PSO(:,2)];
    %CI_y=[mean(temp)-1.96*std(temp)/mean(temp),mean(temp)+1.96*std(temp)/sqrt(numel(temp))];  
    eta=2.58;
    CI_LSM_x=[mean(position_LSM(:,1))-1.96*std(position_LSM(:,1))/sqrt(numel(position_LSM(:,1))),mean(position_LSM(:,1))+eta*std(position_LSM(:,1))/sqrt(numel(position_LSM(:,1)))];
    CI_LSM_y=[mean(position_LSM(:,2))-1.96*std(position_LSM(:,2))/sqrt(numel(position_LSM(:,2))),mean(position_LSM(:,2))+eta*std(position_LSM(:,2))/sqrt(numel(position_LSM(:,2)))];
    CI_PSO_x=[mean(position_PSO(:,1))-1.96*std(position_PSO(:,1))/sqrt(numel(position_PSO(:,1))),mean(position_PSO(:,1))+eta*std(position_PSO(:,1))/sqrt(numel(position_PSO(:,1)))];
    CI_PSO_y=[mean(position_PSO(:,2))-1.96*std(position_PSO(:,2))/sqrt(numel(position_PSO(:,2))),mean(position_PSO(:,2))+eta*std(position_PSO(:,2))/sqrt(numel(position_PSO(:,2)))];
    %prova_LSM=position_LSM(position_LSM(:,1)>CI_x(1)&position_LSM(:,1)<CI_x(2)&position_LSM(:,2)>CI_y(1)&position_LSM(:,2)<CI_y(2),:);
    %prova_PSO=position_PSO(position_PSO(:,1)>CI_x(1)&position_PSO(:,1)<CI_x(2)&position_PSO(:,2)>CI_y(1)&position_PSO(:,2)<CI_y(2),:);
    
    save_LSM=position_LSM(position_LSM(:,1)>CI_LSM_x(1)&position_LSM(:,1)<CI_LSM_x(2)&position_LSM(:,2)>CI_LSM_y(1)&position_LSM(:,2)<CI_LSM_y(2),:);
    save_PSO=position_PSO(position_PSO(:,1)>CI_PSO_x(1)&position_PSO(:,1)<CI_PSO_x(2)&position_PSO(:,2)>CI_PSO_y(1)&position_PSO(:,2)<CI_PSO_y(2),:);   
    [dist_RSSI]=compute_dist(x,y,beacon_pos,dist,shadowing,fading_mean,position_real,RSSI,computeDist);

    position_LSM_asympt = leastSquaresMethod(beacon_pos,dist_RSSI);
    position_PSO_asympt= particleSwarmOptimizer(beacon_pos,dist_RSSI,position_LSM(i,:),0);
    prova= [position_OPT,position_PSO];
    %}
%% PSO

position_LSM_mean=[mean(position_LSM(:,1)),mean(position_LSM(:,2))];
position_PSO_mean=[mean(position_PSO(:,1)),mean(position_PSO(:,2))];
%{
%position_OPT_mean=[mean(position_OPT(:,1)),mean(position_OPT(:,2))];
(sqrt( (position_LSM(:,1)-position_real(1)).^2+(position_LSM(:,2)-position_real(2)).^2 ))/size(position_LSM,1);
position_LSM_errors=sqrt( (position_LSM(:,1)-position_real(1)).^2+(position_LSM(:,2)-position_real(2)).^2 );
position_PSO_errors=sqrt( (position_PSO(:,1)-position_real(1)).^2+(position_PSO(:,2)-position_real(2)).^2 );
%}
%figure();
%histogram(position_PSO_errors,200);
%figure();
%histogram(position_LSM_errors,200);

%prova_LSM=position_LSM(position_LSM(:,1)>CI_x(1)&position_LSM(:,1)<CI_x(2)&position_LSM(:,2)>CI_y(1)&position_LSM(:,2)<CI_y(2),:);
%prova_PSO=position_PSO(position_PSO(:,1)>CI_x(1)&position_PSO(:,1)<CI_x(2)&position_PSO(:,2)>CI_y(1)&position_PSO(:,2)<CI_y(2),:);
%%
k=3;
choice=kmeans(position_LSM,k);
colors={'g','b','r','c','y'};
figure()
for i=1:k
    scatter(position_LSM(choice==i,1),position_LSM(choice==i,2),colors{i});
    hold on;
end
    %}