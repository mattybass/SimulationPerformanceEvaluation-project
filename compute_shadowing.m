function [shadowing] = compute_shadowing(x,y,chess_lenght)
    mu=;
    sigma=;

    x_index=[1:chess_lenght:numel(x)];
    x_index(end+1)=numel(x);
    x_index=unique(x_index);
    
    y_index=[1:chess_lenght:numel(y)];
    y_index(end+1)=numel(y);
    y_index=unique(y_index);

    shadowing=zeros(numel(y),numel(x));
    for i=1:size(x_index,2)-1
        for j=1:size(y_index,2)-1
            shadowing(y_index(j):y_index(j+1),x_index(i):x_index(i+1))=lognrnd(mu,sigma);
        end
    end

end

