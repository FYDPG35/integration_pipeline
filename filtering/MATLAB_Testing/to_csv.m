function csv = to_csv( A )
    index = 1;
    for i=1:size(A,1)
        for j=1:size(A,2)
            csv(index,1) = i;
            csv(index,2) = j;
            csv(index,3) = A(i,j);
            index = index + 1;
        end
    end
end

