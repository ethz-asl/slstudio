clc;
clear('all');
close('all');

pause('on')

p = Projector(1); % Type in screen number
c = Camera(0,0); 
% First argument - interface mode indice (E.g. Spinnaker, flycapture, etc) 
% Second argument - camera number 
% Note: Fixed to use software trigger
c.startCapture()

p.GetScreenInfo();

response = zeros(256, 1);

i=255
tex = repmat(uint8(i), [1 1 3]);
p.displayTexture(tex);
disp("Texture displayed!");
pause(10);
disp("Wait is over!");
tic
I = c.getFrame();
toc
disp("Acquired frame!");
response(i+1) = mean(I(:));
figure;
imshow(I);

i=0
tex = repmat(uint8(i), [1 1 3]);
p.displayTexture(tex);
disp("Texture displayed!");
pause(10);
disp("Wait is over!");
tic
I = c.getFrame();
toc
disp("Acquired frame!");
response(i+1) = mean(I(:));
figure;
imshow(I);

i=255
tex = repmat(uint8(i), [1 1 3]);
p.displayTexture(tex);
disp("Texture displayed!");
pause(10);
disp("Wait is over!");
tic
I = c.getFrame();
toc
disp("Acquired frame!");
response(i+1) = mean(I(:));
figure;
imshow(I);





% for i=0:10:255
%     tex = repmat(uint8(i), [1 1 3]);
%     p.displayTexture(tex);
%     pause(0.1);
%     I = c.getFrame();
%     response(i+1) = mean(I(:));
% end
% 
% figure;
% plot(response);
% xlabel('Input intensity');
% ylabel('Output intensity');

%% Clean up
c.stopCapture();
c.delete();
p.delete();