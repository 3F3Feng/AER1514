close all
clear all
clc

load('real_IMU&Odom_data');

%GPS data processf for real data
GPS = navodom(:,4:10);
GPS(:,1:3) = GPS(:,1:3)-GPS(1,1:3);
GPSR = zeros(3,3);
GPStheta = zeros(length(GPS),1);
xline = [1;0;0];
for i = 1:length(GPS)
    GPSR = qGetR(GPS(i,4:7));
    xline = GPSR*(xline);
    rag = xline/(abs(xline(1))+abs(xline(2)));
    GPStheta(i) = atan2(rag(2),rag(1));
end
for i = 1:(length(GPStheta)/2-1)
    GPStheta(2*i-1) = GPStheta(2*i-1)+pi;
end
GPSpos = GPS(:,1:3);
GPS2D = [GPS(:,1:2) GPStheta(:)];

% Vehicle model constants
dt_o = 0.02;
dt_G = 0.01;
r_l = 0.32385;
r_r = 0.32385;
b = 1.501/2;
theta0 = pi/2;
stepsize = 1;
testdata = (-1)*[wheelodom(:,12),wheelodom(:,15)];

%Generate Path from odometer
pos = odom2path(dt_o,r_l,r_r,b,theta0,stepsize,testdata,'ye');
for i = 1:length(pos)
    if pos(i,3)>2*pi
        pos(i,3) = pos(i,3) - 2*pi;
    end
end

pathplot;
sliceplot;

%initialization
T = [1 0 0;0 1 0;0 0 1];
eW = [1 0 0;0 1 0;0 0 1];
alpha = -0.001;
epochs = 5000;
datasize = 1500;
Eplot = zeros(epochs,1);

figure;
hold on
posimu_rot = (T * [pos(:,1:2) ones(length(pos),1)]')';
posimu_rot = posimu_rot/posimu_rot(1,3);
plot(posimu_rot(1:datasize,1),posimu_rot(1:datasize,2),'o');
plot(GPS2D(1:datasize*2,1),GPS2D(1:datasize*2,2),'+');
axis equal

for j = 1:epochs
    E = 0;
    dEdx = 0;
    dEdy = 0;
    dEdtheta = 0;
    for i = 1:datasize
        GPSpred = T * [pos(i,1);pos(i,2);1];
        GPSpred(3) = pos(i,3)+atan2(T(2,1),T(1,1));
        e = (GPS2D(2*i,:)' - GPSpred)'*(GPS2D(2*i,:)' - GPSpred);
        dEdx = dEdx + 2*(GPS2D(2*i,1) - GPSpred(1))/datasize;
        dEdy = dEdy + 2*(GPS2D(2*i,2) - GPSpred(2))/datasize;
        dEdtheta = dEdtheta + 0.01*(2*(GPS2D(2*i,1) - GPSpred(1))*(-T(1,2)*GPS2D(2*i,1)+T(1,1)*GPS2D(2*i,2))...
                            + 2*(GPS2D(2*i,2) - GPSpred(2))*(-T(1,1)*GPS2D(2*i,1)-T(1,2)*GPS2D(2*i,2))...
                            + 2*(GPS2D(2*i,3)-GPSpred(3)))/datasize;
%          dEdx = dEdx/sqrt((abs(dEdx)^2+abs(dEdy)^2+abs(dEdtheta)^2));
%          dEdy = dEdy/sqrt((abs(dEdx)^2+abs(dEdy)^2+abs(dEdtheta)^2));
%          dEdtheta = dEdtheta /sqrt((abs(dEdx)^2+abs(dEdy)^2+abs(dEdtheta)^2));
        E =E + sum(sum(e))/datasize;
    end
    if mod(j,100)==0
        posimu_rot = (T * [pos(1:datasize,1:2) ones(datasize,1)]')';
        posimu_rot = posimu_rot/posimu_rot(1,3);
        plot(posimu_rot(1:datasize,1),posimu_rot(1:datasize,2));
    end
    Eplot(j) = E;
    if E<0.5
        break
    end
    theta = atan2(T(2,1),T(1,1)) + alpha * dEdtheta;
    T(1:2,1:2) = rad2C(theta);
    T(1,3) = T(1,3) - alpha * dEdx ;
    T(2,3) = T(2,3) - alpha * dEdy ;
end
figure;
plot(Eplot)
E = 0;
for i = 1:datasize
        GPSpred = T * [pos(i,1);pos(i,2);1];
        GPSpred(3) = pos(i,3)+atan2(T(2,1),T(1,1));
        e = (GPS2D(2*i,:)' - GPSpred)'*(GPS2D(2*i,:)' - GPSpred);
        E = E+sum(sum(e))/datasize;
end
posimu_rot = (T * [pos(:,1:2) ones(length(pos),1)]')';
posimu_rot = posimu_rot/posimu_rot(1,3);
figure;
hold on
plot(posimu_rot(1:datasize,1),posimu_rot(1:datasize,2));
plot(GPS2D(1:datasize*2,1),GPS2D(1:datasize*2,2));
axis equal
figure;
hold on
plot(posimu_rot(:,1),posimu_rot(:,2));
plot(GPS2D(:,1),GPS2D(:,2));
axis equal
E
min(Eplot)
T