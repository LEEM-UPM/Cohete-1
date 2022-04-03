%% Traductor datos LEEMUR

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
