function binaryImage = CreatingBinary()


format long g;
format compact;
fontSize = 36;
rgbImage = imread('Images/Image4.png');

% Get the dimensions of the image.  numberOfColorBands should be = 3.
[rows, columns, numberOfColorBands] = size(rgbImage);

%imshow(rgbImage);


greenChannel = rgbImage(:, :, 2);

binaryImage = greenChannel < 200;


%imshow(binaryImage);


end