function Y=poly2str(X)
    if isvector(X) == 0,
        disp('X is not vector!');
        return ;
    end;
    Y='';
    n = length(X);
    for i=1:n,
        if(i~=1 && X(i)>0)
            Y = [Y '+'];
        end;
        if(X(i)==0),
            continue;
        elseif(X(i)==1 && i~=n),
            Y=Y;
        else
            Y=[Y num2str(X(i))];
        end;
        if(i==n-1),
            Y = [Y 'x']
        elseif(i==n),
            Y=Y;
        else
            Y=[Y 'x^' num2str(n-i)];
        end;
    end;
    if(Y(1)=='+')
        Y(1)=[];
    end;
    % É¾³ý+1»òÕß-1
    index = regexp(Y,'1x|-1x');
    if(isempty(index)==0),
        for i=1:length(index),
            Y(index(i))=[];
        end;
    end;