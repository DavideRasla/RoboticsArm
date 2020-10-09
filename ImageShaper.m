clear all; clc; 

function binaryImage = CreatingBinary()


format long g;
format compact;
fontSize = 36;
rgbImage = imread('Hands.png');

% Get the dimensions of the image.  numberOfColorBands should be = 3.
[rows, columns, numberOfColorBands] = size(rgbImage);

%imshow(rgbImage);
greenChannel = rgbImage(:, :, 2);

% Get the binaryImagepathAll = [path; zeros(1,numcols(path))];
binaryImage = greenChannel < 200;


%imshow(binaryImage);


end