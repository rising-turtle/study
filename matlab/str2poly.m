function Y=str2poly(X)
    if ischar(X)==0,
        disp('X is not str!');
        return ;
    end;
    % segment the str by '+' and '-'
    index = regexp(X,'\+|\-'); 
    L = length(index);
    term = cell(1,L+1);
    term(1) = cellstr(X(1:index(1)-1));
    for i=2:L,
        term(i)=cellstr(X(index(i-1):index(i)-1));
    end;
    term(L+1)=cellstr(X(index(L):end));
    
    % calculate coefficient and power
    coefficient = [];
    power = [];
    if(isempty(char(term(1)))),
        term(1) =[];
        L = L-1;
    end;
    for i=1:L+1,
        substr = char(term(i));
        index2 = regexp(substr,'x\^');
        % match x.^
        if(isempty(index2)==0),
            power = [power str2num(substr((index2(1)+2):end))];
            if(index2(1)==1),
                coefficient = [coefficient 1];
            elseif(substr(1)=='+'),
                if(index2(1)==2),
                    coefficient = [coefficient 1];
                else
                    coefficient = [coefficient str2num(substr(2:index2(1)-1))];
                end;
            elseif(substr(1)=='-'),
                if(index2(1)==2),
                    coefficient = [coefficient -1];
                else
                    coefficient = [coefficient str2num(substr(1:index2(1)-1))];
                end;
            end;
        else % match x
            index3  = regexp(substr, 'x');
            if(isempty(index3)==0),
                power = [power 1];
                if(index3(1)==1),
                    coefficient = [coefficient 1];
                elseif(substr(1)=='+'),
                    if(index3(1)==2),
                        coefficient = [coefficient 1];
                    else
                        coefficient = [coefficient str2num(substr(2:index3(1)-1))];
                    end;
                elseif(substr(1)=='-'),
                    if(index3(1)==2),
                        coefficient = [coefficient -1];
                    else
                        coefficient = [coefficient str2num(substr(1:index3(1)-1))];
                    end;
                else
                    coefficient = [coefficient str2num(substr(1:index3(1)-1))];
                end;
            else % constant
                power = [power 0];
                coefficient = [coefficient str2num(substr)];
            end;
        end;
    end;
    % 合并同类项
    N = max(power)+1;
    Y = zeros(1,N);
    for i=1:N,
        index4 = find(power==(N-i));
        Y(i) = sum(coefficient(index4));
    end;
        