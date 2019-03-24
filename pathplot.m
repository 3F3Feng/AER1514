T = 10;
GPSline = GPSpos(T/dt_G,:)-GPSpos(1,:);
Odomline = pos(T/dt_o,1:2)-pos(1,1:2);
rad = atan2(GPSline(2),GPSline(1))-atan2(Odomline(2),Odomline(1));
rotpos = (rad2C(rad)*pos(:,1:2)')';

hold on
plot(-GPSpos(:,1),-GPSpos(:,2));
plot(-rotpos(:,1),-rotpos(:,2));
