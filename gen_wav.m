function y = gen_wav(tone,rythm)
Fs = 8192;
freqs = [523,587,659,698,783,880,988,1046,1174,1318,1396,1566,1760,1976];
x = linspace(0,2 * pi * rythm,floor(Fs * rythm));
y = sin(freqs(tone) * x) .* (1 - x / (2 * pi * rythm));
end