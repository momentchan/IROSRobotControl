m=textread('record.txt');
d=[];

for i=1:size(m,1)
    d=[d; m(i,5:8)-m(i,1:4)/255.0*100];   
end
% hold on;
% plot(d(:,2),'magenta');
% plot(d(:,3),'yellow');
% plot(d(:,4),'black');

 plot(mean(d,2));

