function FrenKomutu = otonomFren(MesafeMatrisi)
    % Hafızada bir "Emin olma" sayacı tutuyoruz
    persistent eminOlmaSayaci
    if isempty(eminOlmaSayaci)
        eminOlmaSayaci = 0; 
    end

    FrenKomutu = 0; % Varsayılan: Frene basma

    % Eğer hedef 20 metreden yakınsa sayacı artır
    if MesafeMatrisi(1) > 0 && MesafeMatrisi(1) < 20 
        eminOlmaSayaci = eminOlmaSayaci + 1;
    else
        % Eğer o 1 saliselik hayalet hedef kaybolursa, sayacı hemen sıfırla!
        eminOlmaSayaci = 0; 
    end

    % SADECE ve SADECE hedefi aralıksız 3 adım (0.3 saniye) görürsen frene bas!
    if eminOlmaSayaci >= 3
        FrenKomutu = 1;
    end
end