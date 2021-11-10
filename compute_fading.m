function [fading] = compute_fading(space,model)
    x_index=[1:model.chess_lenght:numel(space.x)];
    x_index(end+1)=numel(space.x);
    x_index=unique(x_index);
    
    y_index=[1:model.chess_lenght:numel(space.y)];
    y_index(end+1)=numel(space.y);
    y_index=unique(y_index);
    
    fading=zeros(numel(space.x),numel(space.y),size(space.beacon_size,1));
    %fading=cell(1,size(beacon_pos,1));
    %fading_mean=cell(1,size(beacon_pos,1));
    for k=1:size(fading,3)
        %fading{k}=zeros(numel(x),numel(y));
        %fading_mean{k}=ones(numel(x),numel(y)).*0.5;
        for i=1:size(x_index,2)-1
            for j=1:size(y_index,2)-1
                fading(x_index(i):x_index(i+1),y_index(j):y_index(j+1),k)=binornd(1,0.50)*exprnd(1);
            end
        end
    end
end

