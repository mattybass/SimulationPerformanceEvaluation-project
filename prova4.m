%data = randi([1,10], 3,3,4);    % your matrix with n = 4
%data_det = arrayfun(@(i)det(data(:,:,i)), 1:size(data,3));    
%data_trace = arrayfun(@(i)trace(data(:,:,i)), 1:size(data,3));
%data_inv = arrayfun(@(i)inv(data(:,:,i)), 1:size(data,3), 'UniformOutput', false);
%[matrix.i,matrix.j]=meshgrid(1:size(dist_RSSI_final,1),1:size(dist_RSSI_final,2));
%matrix.i=1:size(dist_RSSI_final,1);
%matrix.j=1:size(dist_RSSI_final,2);
%provaaaaaa = arrayfun(@(index)leastSquaresMethod(beacon.position,dist_RSSI_final(index.i,index.j,:)), matrix, 'UniformOutput', false); 
a=zeros(size(dist_RSSI_final,1),size(dist_RSSI_final,2),model.nvars);
b=zeros(size(dist_RSSI_final,1),size(dist_RSSI_final,2),model.nvars);
for i=1:size(dist_RSSI_final,1)
    for j=1:size(dist_RSSI_final,2)
        a(i,j,:)=leastSquaresMethod(beacon.position,dist_RSSI_final(i,j,:));
        b(i,j,:)=particleSwarmOptimizer(beacon.position,dist_RSSI_final(i,j,:),model);
    end
end
