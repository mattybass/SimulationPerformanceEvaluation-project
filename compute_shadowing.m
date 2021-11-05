function [shadowing] = compute_shadowing(x,y,chess_lenght)
    mu=0;
    sigma=6;

    x_index=[1:chess_lenght:numel(x)];
    x_index(end+1)=numel(x);
    x_index=unique(x_index);
    
    y_index=[1:chess_lenght:numel(y)];
    y_index(end+1)=numel(y);
    y_index=unique(y_index);

    shadowing=zeros(numel(x),numel(y));
    for i=1:size(x_index,2)-1
        for j=1:size(y_index,2)-1
            shadowing(x_index(i):x_index(i+1),y_index(j):y_index(j+1))=normrnd(mu,sigma);
        end
    end

end

