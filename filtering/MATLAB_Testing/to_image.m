function image = to_image( B, row, col )
    image = zeros(row,col);
    for i=1:length(B)
        index_x = B(i,1);
        index_y = B(i,2);
        image(index_x, index_y) = uint8(B(i,3));
    end
end

