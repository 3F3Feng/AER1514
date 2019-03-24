function [pos] = odom2path(dt,r_l,r_r,b,theta0,stepsize,testdata,whetherplot)
data_length = fix(length(testdata(:,1))/stepsize);
% initialize position matrix
pos = zeros(data_length,3);
vel = zeros(2,data_length);
pos(1,3) = theta0;
j = 0;
for i=1:(length(testdata(:,1))-stepsize)
    if rem(i,stepsize) == 0
        j = j+1 ;
        if abs(testdata(i+stepsize,1)-testdata(i,1)) < 1000
            odom_dot_l = 2*pi*(testdata(i+stepsize,1)-testdata(i,1))/(stepsize*dt*48);
        else if testdata(i+stepsize,1)-testdata(i,1) > 0
                odom_dot_l = 2*pi*(testdata(i+stepsize,1)-testdata(i,1)-1024)/(stepsize*dt*48);
            else 
                odom_dot_l = 2*pi*(testdata(i+stepsize,1)-testdata(i,1)+1024)/(stepsize*dt*48);
            end
        end
        if abs(testdata(i+stepsize,2)-testdata(i,2)) < 1000
            odom_dot_r = 2*pi*(testdata(i+stepsize,2)-testdata(i,2))/(stepsize*dt*48);
        else if testdata(i+stepsize,2)-testdata(i,2) > 0
                odom_dot_r = 2*pi*(testdata(i+stepsize,2)-testdata(i,2)-1024)/(stepsize*dt*48);
            else
                odom_dot_r = 2*pi*(testdata(i+stepsize,2)-testdata(i,2)+1024)/(stepsize*dt*48);
            end
        end
        vel(:,j) = [r_l/2 r_r/2; r_l/(2*b) -r_r/(2*b)]*[odom_dot_l odom_dot_r]';
    
        trans = [cos(pos(j,3)) 0; sin(pos(j,3)) 0; 0 1];
        [gvel] = trans*vel(:,j);
        pos(j+1,:) = (pos(j,:)' + [gvel]*stepsize*dt)';
    end
end
if strcmp(whetherplot,'yes')
    figure(1)
    comet(pos(:,1),pos(:,2))
end
end

