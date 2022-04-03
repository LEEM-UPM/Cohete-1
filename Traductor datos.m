%% TRADUCTOR DATOS LEEMUR 2


%% Escritura en el script de arduino

% 'EE'
% tiempo            ms
% temperatura_BMP   ºC
% presion_BMP       Pa
% altura_BMP        m
% aceleracion_x     g
% aceleracion_y     g
% aceleracion_z     g
% altura_gps        m
% latitud_gps       º
% longitud_gps      º

%% Lectura de datos 

clc
clear

fileID = fopen("DATOS.TXT",'r');
h = 0;
paquetes_perdidos = 0;

while (~feof(fileID))
    h = h+1;
    check = fread(fileID,2,'char');

       if (check(1) == 'D' && check(2) == 'D')
         despegue = h; 
         check = fread(fileID,2,'char');
       end

       if (check(1) == 'A' && check(2) == 'A')
         apertura = h;
         check = fread(fileID,2,'char');
       end

             if (check(1) ~= 'E' || check(2) ~= 'E')
                 paquetes_perdidos = paquetes_perdidos + 1;
                 disp("Paquetes perdido");
             end    

                  while (check(1) ~= 'E' || check(2) ~= 'E')
                      check(1) = check(2);
                      check(2) = fread(fileID,1,'char');
                  end
            
             Data(h,1) = fread(fileID,1,'uint');
    for i=2:7
        Data(h,i) = fread(fileID,1,'single');
    end 
end
fclose(fileID);

%% Plot de los datos

tiempo           = Data(:,1);         
temperatura_BMP  = Data(:,2);  
presion_BMP      = Data(:,3); 
altura_BMP       = Data(:,4);  
aceleracion_x    = Data(:,5);  
aceleracion_y    = Data(:,6); 
aceleracion_z    = Data(:,7);  
altura_gps       = Data(:,8);  
% latitud_gps      = Data(:,9);  
% longitud_gps     = Data(:,10);  


Figure(1)

subplot(3,1,1)
plot(tiempo,temperatura_BMP)
title('Temperatura (º) BMP')
xlabel('tiempo (ms)')
ylabel('Temperatura (º)')

subplot(3,1,2)
plot(tiempo,presion_BMP)
title('Presion (Pa) BMP')
xlabel('tiempo (ms)')
ylabel('Presión (Pa)')

subplot(3,1,3)
plot(tiempo,altura_BMP)
hol on
plot(tiempo,altura_gps)
title('Atura (m)')
legend('Altura_BMP','Altura_GPS')
xlabel('tiempo (ms)')
ylabel('Altura (m)')

Figure(2)

subplot(3,1,1)
plot(tiempo,aceleracion_x)
title('Aceleración eje x (g) ADXL345')
xlabel('tiempo (ms)')
ylabel('Aceleración (g)')

subplot(3,1,2)
plot(tiempo,aceleracion_y)
title('Aceleración eje Y (g) ADXL345')
xlabel('tiempo (ms)')
ylabel('Aceleración (g)')

subplot(3,1,3)
plot(tiempo,aceleracion_z)
title('Aceleración eje Z (g) ADXL345')
xlabel('tiempo (ms)')
ylabel('Aceleración (g)')
