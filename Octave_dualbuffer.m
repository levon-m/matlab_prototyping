%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dualBufferOctave.m
% Simple dual-buffer octave-down shifter (mono) – fixed & annotated
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc

% 0) --- Load a MONO bass file ------------------------------------------------
[x, Fs] = audioread('Jam.wav');   % <-- put any mono file in same folder
x       = x(:,1);                 % force mono if the file was stereo

% 1) --- User-tweakable parameters -------------------------------------------
ratio      = 0.5;    % 0.5 = one octave down
bufDurSec  = 0.04;   % 40 ms total buffer length
xfadeDur   = 0.01;   % 10 ms cross-fade

% 2) --- Derived sizes --------------------------------------------------------
Nbuf         = round(bufDurSec*Fs);         % samples in circular buffer
xfadeSamples = round(xfadeDur *Fs);
buf          = zeros(Nbuf,1);               % allocate buffer

% 3) --- Pointer initialisation ----------------------------------------------
wr  = 1;                     % write pointer  (1-based)
rdA = 1;                     % read pointer A (active first)
rdB = round(Nbuf/2);         % read pointer B (half buffer behind)
useA       = true;           % which pointer feeds the output?
xfadeCount = 0;              % counts DOWN during a cross-fade

% 4) --- Output allocation ----------------------------------------------------
y = zeros(size(x));

% 5) --- Processing loop ------------------------------------------------------
for n = 1:length(x)

    % (A) Write incoming sample
    buf(wrapIdx(wr, Nbuf)) = x(n);

    % (B) Read from the active pointer, AFTER wrapping it
    if useA
        rdA     = rdA + ratio;                 % advance first (fractional ok)
        ySample = bufInterp(buf, rdA);         % wrap happens inside bufInterp
    else
        rdB     = rdB + ratio;
        ySample = bufInterp(buf, rdB);
    end

    % (C) Check distance to write head and begin a cross-fade if needed
    activePtr = useA * rdA + (~useA) * rdB;    % whichever ptr is live
    dist      = mod(activePtr - wr + Nbuf, Nbuf);
    if dist < xfadeSamples && xfadeCount == 0
        xfadeCount = xfadeSamples;             % start fade next sample
        useA       = ~useA;                    % flip active ptr flag
    end

    % (D) Execute cross-fade (linear 0→1)
    if xfadeCount > 0
        fadePos  = 1 - (xfadeCount / xfadeSamples);
        y1       = bufInterp(buf, rdA);
        y2       = bufInterp(buf, rdB);
        ySample  = (1 - fadePos)*y1 + fadePos*y2;
        xfadeCount = xfadeCount - 1;
    end

    y(n) = ySample;

    % (E) Increment & wrap write pointer
    wr = wr + 1;
end

% 6) --- Post processing: low-pass + soft-drive for "growl" -------------------
[b,a] = butter(3, 2000/(Fs/2));   % 2 kHz 3rd-order low-pass
y     = filter(b,a,y);
y     = tanh(3*y);                % gentle saturation

% 7) --- Audition & export ----------------------------------------------------
soundsc([x, y], Fs);              % Left = dry; Right = octave-down
audiowrite('oct_down.wav', y, Fs);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper: wrapIdx  – branch-free 1-based index wrapping
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function w = wrapIdx(idx, N)
    w = mod(idx-1, N) + 1;        % keeps fractional part intact
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper: bufInterp – linear interpolation with internal wrap
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function s = bufInterp(buf, idx)
    N   = numel(buf);
    idx = mod(idx-1, N) + 1;      % 1-based wrap for fractional index

    i1   = floor(idx);
    frac = idx - i1;
    i2   = i1 + 1;
    if i2 > N, i2 = 1; end        % wrap upper edge

    s = (1-frac)*buf(i1) + frac*buf(i2);
end
