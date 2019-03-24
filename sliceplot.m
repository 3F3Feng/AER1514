slice_size = 100;
figure;
hold on
for i = 1:length(rotpos)
    if mod(i,slice_size) == 0
        rotpos(i:length(rotpos),1:2) = rotpos(i:length(rotpos),1:2) -...
            (rotpos(i,1:2)-GPSpos(2*i,1:2));
        T = 0.1;
        GPSline = GPSpos(2*i+T/dt_G,:)-GPSpos(2*i,:);
        Odomline = rotpos(i+T/dt_o,1:2)-rotpos(i,1:2);
        rad = atan2(GPSline(2),GPSline(1))-atan2(Odomline(2),Odomline(1));
        if i+99<length(rotpos)
            rotpos(i:i+99,1:2) =...
                (rad2C(rad)*(rotpos(i:i+99,1:2)-rotpos(i,1:2))'+rotpos(i,1:2)')';
            plot(-rotpos(i:i+99,1),-rotpos(i:i+99,2),'LineWidth',2);
        else
            rotpos(i:length(rotpos),1:2) =...
                (rad2C(rad)*(rotpos(i:length(rotpos),1:2)-rotpos(i,1:2))'+rotpos(i,1:2)')';
            plot(-rotpos(i:length(rotpos),1),-rotpos(i:length(rotpos),2),'LineWidth',2);
        end
    end
end
figure;
hold on
plot(-GPSpos(:,1),-GPSpos(:,2),'LineWidth',2);
plot(-rotpos(:,1),-rotpos(:,2),'LineWidth',2);