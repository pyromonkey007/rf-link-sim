% ---- Constants & inputs ----
K_BOLTZ     = 1.38064852e-23; K_BOLTZMANN= K_BOLTZ;          % J/K
noise_temp  = 231.2;   config.rf.noise_temperature_k=noise_temp; % K
B           = 1000000;  rx_cfg.bandwidth_hz= B;         % Hz
Rb          = 6.5e6;                         % b/s
EbNo_req    = 12.5;                          % dB

Ptx_dbw     = 13.01; tx_power_dbw= Ptx_dbw;       % dBW
Gtx_dbi     = -10;  tx_gain_dbi =  Gtx_dbi;       % dBi
Ltx_dB       = -8.04; tx_other_loss_db=Ltx_dB;   % (–dB for loss)

Grx_dbi     = 31;     rx_gain_dbi= Grx_dbi;    % dBi
Lrx_dB       = -5;     rx_loss_db= Lrx_dB;    % (–dB for feed loss)
Glna_dB     = 30;      rx_lna_gain_db= Glna_dB;% +dB

% ---- 5. EIRP ----
eirp_dbw = Ptx_dbw + Gtx_dbi + Ltx_dB

% ---- 6. Received power at demod input ----
total_loss_db = -142.5
rx_chain_gain_db = Grx_dbi + Lrx_dB + Glna_dB
prx_demod_dbw    = eirp_dbw ...
                   + total_loss_db ...      
                   + rx_chain_gain_db      % all receive‐chain gains/losses
link_status.prx_demod_dbw = prx_demod_dbw;

% ---- 7. Noise power at demod input ----
noise_floor_dbw    = 10*log10( K_BOLTZ * noise_temp * B )
noise_demod_dbw    = noise_floor_dbw + Glna_dB  % + IF gain here if you have one
link_status.noise_power_dbw = noise_demod_dbw;

% ---- 8. C/N and SNR ----
cn_db = prx_demod_dbw - noise_demod_dbw
link_status.cn_db = cn_db;

% Convert C/N to SNR at the bit‐rate
%   SNR = C/N – 10*log10(Rb/B)
snr_db = cn_db - 10*log10( Rb / B )
link_status.snr_db = snr_db;

% ---- 9. Link check ----
link_status.required_snr_db = 12.5;
if snr_db < EbNo_req
    link_status.status = 'FAIL'
    link_status.failure_reason = sprintf('SNR too low: %.1f < %.1f dB', snr_db, EbNo_req);
else
    link_status.status = 'OK'
end

%%%%%%%%%%%%%%%%%%
% ---- 5. EIRP ----
eirp_dbw = tx_power_dbw + tx_gain_dbi + tx_other_loss_db

% ---- 6. Received power at demod input ----
rx_chain_gain_db = rx_gain_dbi + rx_loss_db + rx_lna_gain_db;
received_power_dbw   = eirp_dbw ...
                   + total_loss_db ...      
                   + rx_chain_gain_db      % all receive‐chain gains/losses
received_power_dbm = received_power_dbw + 30;
link_status.received_power_dbm = received_power_dbm;

% ---- 7. Noise power at demod input ----
% Get bandwidth, using default if not specified for the receiver
% if ~isfield(rx_cfg, 'bandwidth_hz') || isempty(rx_cfg.bandwidth_hz) || rx_cfg.bandwidth_hz <= 0
%     bandwidth_hz = config.rf.default_bandwidth_hz;
%     if i_tx==1 && i_rx==1 % Warn only once per simulation start potentially
%         warning('[RF] Rx "%s" missing/invalid BW, using default %.1f MHz.', rx_id, bandwidth_hz/1e6);
%     end
% else
    bandwidth_hz = rx_cfg.bandwidth_hz;
% end
noise_temp_k = config.rf.noise_temperature_k; % System noise temp
noise_floor_dbw    = 10*log10( K_BOLTZMANN * noise_temp_k * bandwidth_hz );
noise_demod_dbw    = noise_floor_dbw + rx_lna_gain_db;  % + IF gain here if you have one
noise_demod_dbm = noise_demod_dbw  + 30;
link_status.noise_power_dbm = noise_demod_dbm;
link_status.bandwidth_hz = bandwidth_hz;

% ---- 8. C/N and SNR ----
cn_db = received_power_dbw - noise_demod_dbw

% Convert C/N to SNR at the bit‐rate
%   SNR = C/N – 10*log10(Rb/B)
Rb = 6.5e6;                         % b/s
snr_db = cn_db - 10*log10( Rb / bandwidth_hz )
link_status.snr_db = snr_db;
snr_db_eff = snr_db;


% % --- 8. Link Status Check vs Sensitivity ---
% rx_sensitivity_dbm = rx_cfg.sensitivity_dbm;
% link_status.rx_sensitivity_dbm = rx_sensitivity_dbm;

required_snr_db = 12.5 
link_status.required_snr_db = required_snr_db;
% Check if effective SNR meets the requirement ONLY if the status is currently OK
if snr_db_eff < required_snr_db && strcmp(link_status.status, 'OK')
    link_status.status = 'FAIL';
    link_status.failure_reason = append_reason(link_status.failure_reason, sprintf('SNR < Req (%.1f<%.1f)', snr_db_eff, required_snr_db));
    link_status.primary_failure_step = 'Signal Level';
end