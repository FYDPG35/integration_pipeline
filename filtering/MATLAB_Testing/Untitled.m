close all;
clear all;

A = imread('cameraman.tif');
csv_out = csvread('cameraman_out.csv');


data = to_csv(A);
for i=1:length(csv_out);
    csv_out(i,1) = 5*csv_out(i,1);
    csv_out(i,2) = 5*csv_out(i,2);
end
B = to_image(csv_out, 1274, 1274);
B = uint8(B);

figure
surf(B);
colormap(jet);

% csvwrite('cameraman.csv', data);