function link_budget_demo()
    % ---- Inputs ----
    f      = 2.4e9;           % Hz
    Ptx_dBW= 13.01;           % dBW
    Gtx    = -10;             % dBi
    Ltx    = -8.04;           % dB  (negative = loss)
    Grx    = +31;             % dBi
    Lrx    = -5;              % dB  (feed loss, negative)
    Glna   = +30;             % dB
    d_nm   = 70;              % nm
    B      = 1e6;             % Hz (IF filter)
    Rb     = 6.5e6;           % b/s
    EbNo_req = 12.5;          % dB

    % ---- Calc ----
    c = physconst('LightSpeed');
    lambda = c/f;
    d_km   = d_nm*1852/1e3;
    FSPL   = 20*log10(d_km) + 20*log10(f/1e6) + 32.44;

    EIRP   = Ptx_dBW + Gtx + Ltx;
    Cin    = EIRP - FSPL + Grx + Lrx;
    C      = Cin + Glna;

    Nin    = -228.6 + 10*log10(B); % dBW
    N      = Nin + Glna;

    CN     = C - N;
    SNR    = CN - 10*log10(Rb/B);
    margin = SNR - EbNo_req;

    % ---- Display ----
    fprintf('EIRP: %.2f dBW\n', EIRP);
    fprintf('C   : %.2f dBW\n', C);
    fprintf('N   : %.2f dBW\n', N);
    fprintf('C/N : %.2f dB\n', CN);
    fprintf('SNR : %.2f dB  (margin %.2f dB)\n', SNR, margin);
end
