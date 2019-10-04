size_y=16;
size_u=64;
size_v=64;
tmap=uint8(zeros(size_y*size_u*size_v,1));


for y=1:16
  for u=37:57
    for v=30:47
      i=((y-1)*size_u+(u-1))*size_v+(v-1);
      tmap(i)=7;
    end
  end
end

fp=fopen('out.tmap','w');
fprintf(fp,'TMAP\nYUV8\n%d %d %d\n',size_y,size_u,size_v);
fwrite(fp,tmap,'uint8');
fclose(fp);
