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

figure;

num_samples = 1;

i=200;
tex = repmat(uint8(i), [1 1 3]);
p.displayTexture(tex);
pause(0.25);
I = c.getFrame();

imshow(I);

% for i=0:255
%     i
%     samples = zeros(1,10);
%     
%     for j=1:1:num_samples
%     
%         tex = repmat(uint8(i), [1 1 3]);
%         p.displayTexture(tex);
%         pause(0.25);
%         I = c.getFrame();
% 
%         imshow(I);
% 
%         samples(j) = mean(I(:));
%     end
%     
%     mean(samples)
%     response(i+1) = mean(samples);
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