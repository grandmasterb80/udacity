% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% specify radar
Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1500;             % Length of signal
t = (0:L-1)*T;        % Time vector


% 2-D Transform
% The 2-D Fourier transform is useful for processing 2-D signals and other 2-D data such as images.
% Create and plot 2-D data with repeated blocks.

%P = peaks(20);
X = randn(100,200);
XL = size(X,2);
for i = 1:XL
    v = i / XL
    JP = [2  14+v  25-(3*v)  53+5*v  71+12*v  95-5*v];
    JV = [4 3 2 5 8 3]
    for j = 1:6
        J1 = floor(JP(j))
        J2 = ceil(JP(j))
        w = JP(j) - J1;
        r = (randn + 4) / 5;
        X(J1,i) = X(J1,i) + JV(j) * r * (1 - w);
        X(J2,i) = X(J2,i) + JV(j) * r * w;
    end
end

imagesc(X);

% TODO : Compute the 2-D Fourier transform of the data.  
% Shift the zero-frequency component to the center of the output, and 
% plot the resulting 100-by-200 matrix, which is the same size as X.
M = size( X, 1 );
N = size( X, 2 );
signal_fft = fft2(X, M, N);
signal_fft = fftshift(signal_fft);
signal_fft = abs( signal_fft );
imagesc( signal_fft );


% Plot the noisy signal in the time domain. It is difficult to identify the frequency components by looking at the signal X(t). 
%title('Signal Corrupted with Zero-Mean Random Noise')
%xlabel('t (milliseconds)')
%ylabel('X(t)')
