%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dualBufferOctave_clean.m
%    One-octave-down prototype for bass guitar
%    – dual-buffer resampler, artefacts minimised –
%        • equal-power cross-fades
%        • 4-tap Lagrange interpolation
%        • pre-LPF to cut alias sources
%        • soft-drive + HPF/LPF + auto-normalise
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc

%% 0) --- Load a MONO bass file ------------------------------------------------
[x, Fs] = audioread('Jam.wav');   % <-- put any mono file in the same folder
x       = x(:,1);                 % force mono if file was stereo

%% 0.5) --- Pre-filter to reduce alias energy ---------------------------------
[dLP,cLP] = butter(3, 4000/(Fs/2), 'low');   % 4 kHz, 3rd-order
x = filter(dLP,cLP,x);

%% 1) --- User-tweakable parameters ------------------------------------------
ratio      = 0.5;     % 0.5 = one octave down
bufDurSec  = 0.10;    % 100 ms buffer → splice ≈ 10 Hz (below flutter range)
xfadeDur   = 0.010;   % 10 ms equal-power cross-fade

%% 2) --- Derived sizes -------------------------------------------------------
Nbuf         = round(bufDurSec*Fs);
xfadeSamples = round(xfadeDur *Fs);
buf          = zeros(Nbuf,1);

%% 3) --- Pointer initialisation ---------------------------------------------
wr  = 1;                      % write pointer (1-based)
rdA = 1;                      % read pointer A
rdB = round(Nbuf/2);          % read pointer B (half a buffer behind)
useA       = true;            % which pointer feeds the output?
xfadeCount = 0;               % counts DOWN during a cross-fade

%% 4) --- Output allocation ---------------------------------------------------
y = zeros(size(x));

%% 5) --- Processing loop -----------------------------------------------------
for n = 1:length(x)
    % (A) Write incoming sample
    buf(wrapIdx(wr, Nbuf)) = x(n);

    % (B) Read from active pointer (advance first, fractional allowed)
    if useA
        rdA     = rdA + ratio;
        ySample = bufInterpLagrange(buf, rdA);
    else
        rdB     = rdB + ratio;
        ySample = bufInterpLagrange(buf, rdB);
    end

    % (C) Check distance to write head; start fade if too close
    activePtr = useA * rdA + (~useA) * rdB;
    dist      = mod(activePtr - wr + Nbuf, Nbuf);
    if dist < xfadeSamples && xfadeCount == 0
        xfadeCount = xfadeSamples;   % start fade next sample
        useA       = ~useA;          % flip active pointer flag
    end

    % (D) Execute equal-power cross-fade (sqrt window)
    if xfadeCount > 0
        fadePos   = 1 - (xfadeCount / xfadeSamples);  % 0→1
        w2        = sqrt(fadePos);
        w1        = sqrt(1 - fadePos);
        y1        = bufInterpLagrange(buf, rdA);
        y2        = bufInterpLagrange(buf, rdB);
        ySample   = w1*y1 + w2*y2;
        xfadeCount = xfadeCount - 1;
    end

    y(n) = ySample;

    % (E) Increment & wrap write pointer
    wr = wr + 1;
end

%% 6) --- Post-processing chain ----------------------------------------------
% (i) soft drive
y = tanh(3*y);

% (ii) HPF to remove infrasonic rumble + LPF to tame top end
[bHP,aHP] = butter(2, 35/(Fs/2),   'high');
[bLP,aLP] = butter(3, 2000/(Fs/2), 'low');
y = filter(bHP,aHP,filter(bLP,aLP,y));

% (iii) Prevent clipping before saving / audition
pk = max(abs(y));
if pk > 1
    y = y / pk * 0.94;   % -0.5 dBFS head-room
end

%% 7) --- Audition & export ---------------------------------------------------
soundsc([x, y], Fs);          % L: dry, R: octave-down
audiowrite('oct_down.wav', y, Fs);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper: wrapIdx – branch-free 1-based index wrapping
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function w = wrapIdx(idx, N)
    w = mod(idx-1, N) + 1;   % keeps fractional part intact
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper: bufInterpLagrange – 4-tap Lagrange interpolation with wrap
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function s = bufInterpLagrange(buf, idx)
    N = numel(buf);
    idx = mod(idx-1, N) + 1;          % 1-based fractional index

    i0   = floor(idx) - 1;            % taps -1..+2
    frac = idx - floor(idx);

    % gather four wrapped taps
    taps = zeros(4,1);
    for k = 0:3
        taps(k+1) = buf( mod(i0+k-1, N) + 1 );
    end

    % 4-tap Lagrange coefficients
    c0 = -frac*(frac-1)*(frac-2)/6;
    c1 =  (frac+1)*(frac-1)*(frac-2)/2;
    c2 = -(frac+1)* frac   *(frac-2)/2;
    c3 =  (frac+1)* frac   *(frac-1)/6;

    s = c0*taps(1) + c1*taps(2) + c2*taps(3) + c3*taps(4);
end
