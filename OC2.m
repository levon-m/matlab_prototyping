function OC2
% • Live polarity-flip divider (–12 semitones) – no pre-render
% • Wet/Dry mix & sweepable low-pass (first-order IIR, fully tunable)
% • Test clip loops forever so you can tweak hands-free
% • Live waveform display (timescope)
% • Closing the GUI window stops playback cleanly
%
% Requires Audio Toolbox + DSP System Toolbox (any recent version)

%% 1) User settings -------------------------------------------------------
inFile   = "Chord Memory.wav";    % mono/stereo WAV @ 48 kHz
block    = 512;                    % buffer size
wetInit  = 0.8;                    % initial wet %
fcInit   = 6000;                   % Hz  LPF cutoff start
fcRange  = [100 12000];            % Hz  slOctider limits
fcStep   = 5;                      % Hz  minimum change that triggers update

%% 2) Streaming I/O objects ----------------------------------------------
reader = dsp.AudioFileReader( ...
            Filename        = inFile, ...
            SamplesPerFrame = block, ...
            PlayCount       = Inf);          % loop forever
Fs     = reader.SampleRate;

player = audioDeviceWriter( ...
            SampleRate = Fs, ...
            BufferSize = block);

%% 3) GUI -----------------------------------------------------------------
ui = uifigure("Name","Octave-Down RT", ...
              "Position",[50 50 380 160], ...
              "CloseRequestFcn",@(h,~)delete(h));   % delete → loop exits
g  = uigridlayout(ui,[3 2],"RowHeight",{'fit','fit','1x'},"Padding",8);

uilabel(g,"Text","Wet / Dry (%)","HorizontalAlignment","center");
wetS = uislider(g,"Limits",[0 1],"Value",wetInit,"MajorTicks",0:0.25:1);

uilabel(g,"Text","LPF Cutoff (Hz)","HorizontalAlignment","center");
cutS = uislider(g,"Limits",fcRange,"Value",fcInit, ...
                "MajorTicks",fcRange(1):3000:fcRange(2));

uilabel(g,"Text","Close window to stop playback", ...
           "HorizontalAlignment","center","FontAngle","italic");

%% 4) Waveform scope ------------------------------------------------------
scope = timescope("SampleRate",Fs, ...
                  "TimeSpan",0.5, ...
                  "TimeSpanSource","property", ...
                  "YLimits",[-1 1], ...
                  "Name","Output Waveform");

%% 5) Divider & filter state ---------------------------------------------
flipState  = 1;         % +1 / –1 toggle for divider
prevSample = 0;         % previous sample for zero-cross detection
fcCurrent  = fcInit;    % remember last cutoff
alpha      = exp(-2*pi*fcCurrent/Fs);     % LPF coefficient
lpfState   = 0;         % y[n-1] state for one-pole filter

%% 6) Streaming loop ------------------------------------------------------
disp("Streaming… close the GUI window to quit.");

while isvalid(ui)
    dry = reader();                           % read block
    if isempty(dry), break; end               % EOF safeguard

    if size(dry,2) > 1, dry = mean(dry,2); end % force mono
    dry = double(dry);

    % -- Polarity-flip divider (sample-by-sample) ------------------------
    wet = zeros(size(dry));
    for n = 1:block
        s = dry(n);
        if prevSample <= 0 && s > 0          % upward zero crossing?
            flipState = -flipState;
        end
        wet(n)   = flipState * s;
        prevSample = s;
    end

    % -- Check LPF slider & update coefficient if needed -----------------
    fcNew = cutS.Value;
    if abs(fcNew - fcCurrent) >= fcStep
        fcCurrent = fcNew;
        alpha     = exp(-2*pi*fcCurrent/Fs); % recompute coefficient
    end

    % -- Apply first-order IIR LPF ---------------------------------------
    for n = 1:block
        lpfState = alpha * lpfState + (1-alpha) * wet(n);
        wet(n)   = lpfState;
    end

    % -- Wet/Dry mix + peak-norm guard ----------------------------------
    mix = wetS.Value;
    out = (1-mix)*dry + mix*wet;
    out = out ./ max(1,max(abs(out)));        % avoid clipping

    % -- Play + display --------------------------------------------------
    player(out);
    scope(out);

    drawnow limitrate                        % process GUI events
end

%% 7) Cleanup -------------------------------------------------------------
release(reader);  release(player);  release(scope);
if isvalid(ui), delete(ui); end
disp("Stopped.");
end
