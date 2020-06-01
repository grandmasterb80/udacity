% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% specify radar
Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1500;             % Length of signal
t = (0:L-1)*T;        % Time vector


% 2-D Transform
% The 2-D Fourier transform is useful for processing 2-D signals and other 2-D data such as images.
% Create and plot 2-D data with repeated blocks.

X = randn(100,200);
Nr = size( X, 1 );
Nd = size( X, 2 );
for i = 1:Nr
    v = i / Nr
    JP = [2  14+v  25-(3*v)  53+5*v  71+12*v  95-5*v];
    JV = [4 3 2 5 8 3]
    for j = 1:6
        J1 = floor( JP( j ) )
        J2 = ceil( JP( j ) )
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
signal_fft = fft2(X, Nr, Nd);
signal_fft = fftshift(signal_fft);
signal_fft = abs( signal_fft );
imagesc( signal_fft );


% Plot the noisy signal in the time domain. It is difficult to identify the frequency components by looking at the signal X(t). 
%title('Signal Corrupted with Zero-Mean Random Noise')
%xlabel('t (milliseconds)')
%ylabel('X(t)')
% Implement 1D CFAR using lagging cells on the given noise and target scenario.

% Close and delete all currently open figures
close all;

% TODO: Apply CFAR to detect the targets by filtering the noise.

% 1. Define the following:
% 1a. Training Cells
% 1b. Guard Cells 
% remark: the number refers to each side (e.g. 3 means 3 on the left and 3
% on the right).
Tr = 12
Td = 12
Gr = 4
Gd = 4
Wr = 2 * (Tr + Gr) + 1
Wd = 2 * (Td + Gd) + 1

% Offset : Adding room above noise threshold for desired SNR 
offset=3;

% Vector to hold threshold values 
threshold_cfar = zeros(Nr - Wr + 1, Nd - Wd + 1);

%Vector to hold final signal after thresholding
signal_cfar = zeros(Nr - Wr + 1, Nd - Wd + 1);

% 2. Slide window across the signal length
for i = 1:(Nr-Tr-Gr)
    for j = 1:(Nd-Td-Gd)
        % 2. - 5. Determine the noise threshold by measuring it within the training cells
        s1 = sum( sum( X( i:i+Tr+Gr, j:j+Td+Gd ) ) )
        s2 = sum( sum( X( i+Tr:i+Tr+Gr, j+Td:j+Td+Gd ) ) )
        noiseLevel = s1 - s2;
        threshold = ( noiseLevel / ( ( Tr + Gr ) * ( Td + Gd ) - Gr * Gd ) ) * offset
        threshold_cfar(i,j) = threshold;

        % 6. Measuring the signal within the CUT
        signal = X(i + Tr + Gr, j + Td + Gd)
        if (signal < threshold)
            signal = 0;
        end

        % 8. Filter the signal above the threshold
        signal_cfar(i,j) = signal;
    end
end




% plot the filtered signal
plot (cell2mat(signal_cfar),'g--');

% plot original sig, threshold and filtered signal within the same figure.
figure,plot(X);
hold on,plot(cell2mat(circshift(threshold_cfar,Gr)),'r--','LineWidth',2)
hold on, plot (cell2mat(circshift(signal_cfar,(Tr+Gr))),'g--','LineWidth',4);
legend('Signal','CFAR Threshold','detection')