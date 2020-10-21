clc;
clear('all');
%close('all');

p = Projector(1); % Type in screen number
c = Camera(0,0); % Need to change indice based on camera number (see camera class)
c.startCapture();

response = zeros(256, 1);

for i=0:255
    tex = repmat(uint8(i), [1 1 3]);
    p.displayTexture(tex);
    pause(0.1);
    I = c.getFrame();
    response(i+1) = mean(I(:));
end

figure;
plot(response);
xlabel('Input intensity');
ylabel('Output intensity');