function binaryImage = CreatingBinary()


format long g;
format compact;
fontSize = 36;
rgbImage = imread('Images/Image2.png');

% Get the dimensions of the image.  numberOfColorBands should be = 3.
[rows, columns, numberOfColorBands] = size(rgbImage);

%imshow(rgbImage);

rgbImage = imread('Images/Image2.png');

greenChannel = rgbImage(:, :, 2);

binaryImage = greenChannel < 200;


%imshow(binaryImage);


end