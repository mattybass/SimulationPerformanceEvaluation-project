function [shadowing] = compute_shadowing(space,model)
    x_index=[1:model.chess_lenght:numel(space.x)];
    x_index(end+1)=numel(space.x);
    x_index=unique(x_index);
    
    y_index=[1:model.chess_lenght:numel(space.y)];
    y_index(end+1)=numel(space.y);
    y_index=unique(y_index);

    shadowing=zeros(numel(space.x),numel(space.y));
    for i=1:size(x_index,2)-1
        for j=1:size(y_index,2)-1
            shadowing(x_index(i):x_index(i+1),y_index(j):y_index(j+1))=normrnd(model.shadowing.mu,model.shadowing.sigma);
        end
    end

end

