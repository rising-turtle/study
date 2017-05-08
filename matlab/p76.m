A = [0,-6,-1;6,2,-16;-5,20,-10];
x0 = [1;1;1];
t = 0:0.01:1;
xt = [];
for i=1:length(t),
    xt(i,:)=expm(t(i)*A)*x0;
end;
plot3(xt(:,1),xt(:,2),xt(:,3),'-o')
grid on;
xlabel('xt(:,1)');
ylabel('xt(:.2)');
zlabel('xt(:,3)');
title('Yade');