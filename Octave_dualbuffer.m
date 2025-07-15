%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dualBufferOctave_flutterFree.m
%     Octave-down prototype for bass guitar
%         • 180 ms buffer  (splice ≈ 5.6 Hz – below audible flutter)
%         • 6 ms raised-cosine equal-power cross-fade
%         • 4-tap Lagrange interpolation
%         • Pre-LPF (6 kHz) to curb aliasing without dulling transients
%         • Soft-drive + HPF/LPF + auto-normalise
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc

%% 0) --- Load a MONO bass file ----------------------------------------------
[x, Fs] = audioread('Chord Memory.wav');        % put any mono file in the same folder
x       = x(:,1);                      % force mono if the file was stereo

%% 0.5) --- Pre-filter (relaxed) ---------------------------------------------
[dLP,cLP] = butter(3, 6000/(Fs/2), 'low');   % 6 kHz, 3rd-order
x = filter(dLP,cLP,x);

%% 1) --- Main parameters -----------------------------------------------------
ratio      = 0.5;      % one octave down
bufDurSec  = 0.18;     % 180 ms buffer  → splices ≈ 5.6 Hz
xfadeDur   = 0.006;    % 6 ms raised-cosine cross-fade

%% 2) --- Derived sizes -------------------------------------------------------
Nbuf         = round(bufDurSec * Fs);
xfadeSamples = round(xfadeDur * Fs);
buf          = zeros(Nbuf,1);

%% 3) --- Pointer initialisation ---------------------------------------------
wr  = 1;                       % write pointer
rdA = 1;                       % read pointer A
rdB = round(Nbuf/2);           % read pointer B (half buffer behind)
useA       = true;             % which pointer is active?
xfadeCount = 0;                % counts DOWN during a cross-fade

%% 4) --- Output allocation ---------------------------------------------------
y = zeros(size(x));

%% 5) --- Processing loop -----------------------------------------------------
for n = 1:length(x)
    % (A) write incoming sample
    buf(wrapIdx(wr, Nbuf)) = x(n);

    % (B) read from active pointer
    if useA
        rdA     = rdA + ratio;
        ySample = bufInterpLagrange(buf, rdA);
    else
        rdB     = rdB + ratio;
        ySample = bufInterpLagrange(buf, rdB);
    end

    % (C) start a cross-fade if read head is getting close to write head
    activePtr = useA * rdA + (~useA) * rdB;
    dist      = mod(activePtr - wr + Nbuf, Nbuf);
    if dist < xfadeSamples && xfadeCount == 0
        xfadeCount = xfadeSamples;    % start fade next sample
        useA       = ~useA;           % flip active read head
    end

    % (D) raised-cosine equal-power cross-fade
    if xfadeCount > 0
        fadePos = 1 - (xfadeCount / xfadeSamples);      % 0 → 1
        w2      = sin(0.5*pi*fadePos);                  % √Hann ramp-up
        w1      = cos(0.5*pi*fadePos);                  % √Hann ramp-down
        y1      = bufInterpLagrange(buf, rdA);
        y2      = bufInterpLagrange(buf, rdB);
        ySample = w1*y1 + w2*y2;
        xfadeCount = xfadeCount - 1;
    end

    y(n) = ySample;

    % (E) increment & wrap write pointer
    wr = wr + 1;
end

%% 6) --- Post-processing chain ----------------------------------------------
% (i) soft-drive – lower gain for crisper transients
y = tanh(2*y);

% (ii) tidy low and high extremes
[bHP,aHP] = butter(2,   35/(Fs/2), 'high');   % infrasonic rumble guard
[bLP,aLP] = butter(3, 2000/(Fs/2), 'low');    % gentle low-pass
y = filter(bHP,aHP, filter(bLP,aLP, y));

% (iii) prevent clipping before playback / save
pk = max(abs(y));
if pk > 1
    y = y / pk * 0.94;   % leave ≈-0.5 dBFS headroom
end

%% 7) --- Audition & export ---------------------------------------------------
soundsc([x, y], Fs);          % L = dry,  R = octave-down
audiowrite('oct_down.wav', y, Fs);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper: wrapIdx – branch-free 1-based index wrap
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function w = wrapIdx(idx, N)
    w = mod(idx-1, N) + 1;    % keep fractional part intact
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper: bufInterpLagrange – 4-tap Lagrange interpolation with wrap
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function s = bufInterpLagrange(buf, idx)
    N   = numel(buf);
    idx = mod(idx-1, N) + 1;           % 1-based fractional index

    i0   = floor(idx) - 1;             % taps −1…+2
    frac = idx - floor(idx);

    % gather 4 wrapped taps
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
