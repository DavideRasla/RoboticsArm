clear all; clc;

format long g;
format compact;
fontSize = 36;
rgbImage = imread('Image.png');
% Get the dimensions of the image.  numberOfColorBands should be = 3.
[rows, columns, numberOfColorBands] = size(rgbImage);
% Display the original color image.
subplot(2, 2, 1);
imshow(rgbImage);
axis on;
title('Original Color Image', 'FontSize', fontSize);
% Enlarge figure to full screen.
set(gcf, 'Units', 'Normalized', 'Outerposition', [0, 0, 1, 1]);
% Extract the individual red, green, and blue color channels.
% redChannel = rgbImage(:, :, 1);
greenChannel = rgbImage(:, :, 2);
% blueChannel = rgbImage(:, :, 3);
% Get the binaryImage
binaryImage = greenChannel < 200;
% Display the original color image.
subplot(2, 2, 2);
imshow(binaryImage);
axis on;
title('Binary Image', 'FontSize', fontSize);
% Find the baseline
verticalProfile  = sum(binaryImage, 2);
lastLine = find(verticalProfile, 1, 'last')
% Scan across columns finding where the top of the hump is
for col = 1 : columns
  yy = lastLine - find(binaryImage(:, col), 1, 'first');
  if isempty(yy)
    y(col) = 0;
  else
    y(col) = yy;
  end
end
subplot(2, 2, 3);
plot(1 : columns, y, 'b-', 'LineWidth', 3);
grid on;
title('Y vs. X', 'FontSize', fontSize);
ylabel('Y', 'FontSize', fontSize);
xlabel('X', 'FontSize', fontSize);

[rows, columns] = find(binaryImage==0); % 2 1-D vectors.
xy = [columns, rows]; % Stitch together to form N by 2 (x,y) array





