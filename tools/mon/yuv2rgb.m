function [img]=yuv2rgb(img)

y=img(:,:,1);
u=img(:,:,2)*2-255;
v=img(:,:,3)*2-255;
img(:,:,1)=y+u;
img(:,:,2)=(y-0.51*u-0.19*v);
img(:,:,3)=(y+v);

