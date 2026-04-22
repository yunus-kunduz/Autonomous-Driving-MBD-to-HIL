function [Direksiyon, Gaz] = Surucu(FrenKomutu)
    Direksiyon = 0; % Düz ilerle.
    
    if FrenKomutu == 1
        Gaz = 0; % Beyin fren diyorsa hızı 0 m/s'ye (Tam duruş) düşür!
    else
        Gaz = 15; % Engel yoksa 15 m/s (~ 54 km/h) sabit hızla ilerle.
    end
end