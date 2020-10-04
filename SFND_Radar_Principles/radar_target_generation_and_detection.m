clear all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%
radar_max_range = 200;
radar_range_res = 1;

%speed of light = 3e8
c = 3e8;
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
target_range = 50
target_speed = 20

%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.

% Find the Bsweep of chirp for 1 m resolution
B = c / ( 2 * radar_range_res );
% Calculate the chirp time based on the Radar's Max Range
Tchirp = 5.5 * ( 2 * radar_max_range ) / c;
% slope of the chirp signal
slope = B / Tchirp;


%Operating carrier frequency of Radar 
fc = 77e9;             %carrier freq

                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd = 128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr = 1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t = linspace(0, Nd*Tchirp, Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx  = zeros(1,length(t)); %transmitted signal
Rx  = zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal
% Mix2 = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t = zeros(1,length(t));
td  = zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = target_range + target_speed * t(i);
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    tm = t(i); %t(1 + mod(i-1, Nr));
    td(i) = tm - 2 * r_t(i) / c;
    Tx(i) = cos(2 * pi * ( fc * t(i) + slope*t(i)*t(i) / 2 ));
    Rx(i) = cos(2 * pi * ( fc * td(i) + slope*td(i)*td(i) / 2 ));

    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i) * Rx(i);
%     Mix(i) = cos( 2 * pi * ( 2 * slope * r_t(i) / c * tm + 2 * fc * target_speed/c * tm ) );
end


%% RANGE MEASUREMENT

 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix_2D = transpose( reshape(Mix, Nr, Nd) );

 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
signal_fft = fft( Mix_2D( 1, : ), Nr );
signal_fft = signal_fft / max( signal_fft );

 % *%TODO* :
% Take the absolute value of FFT output
P2 = abs( signal_fft );

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
P1 = P2( 1 : Nr / 2 - 1 );

%plotting the range
figure ('Name','Range from First FFT')
subplot(2,1,1)

 % *%TODO* :
 % plot FFT output 
ff = (1:(Nr/2)-1) * c / ( 2 * B );
plot( ff, P1 )
title('fft(Mix(1,:))')
xlabel('dist|m')
ylabel('strength|A|')
axis ([0 200 0 1]);



%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM);

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
Tr = 4;
Td = 4;

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 2;
Gd = 2;
Wr = 2 * (Tr + Gr) + 1;
Wd = 2 * (Td + Gd) + 1;

% *%TODO* :
% offset the threshold by SNR value in dB
offset=30;

% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
% REMARK: we already use the target size for noise and signal to get an
% array with the same size as our input (RDM).
Nr2 = Nr / 2;
noise_level = zeros(Nr2, Nd);
signal_cfar = zeros(Nr2, Nd);

% *%TODO* :
% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
% CFAR
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.
for i = 1:(Nr2-Tr-Gr)
    % When addressing elements in our target matrix, we skip the first
    % elements, which are supposed to be '0'. They are in the matrix to get
    % a result with the same dimensions as the input.
    i_corr = i + Tr + Gr;
    for j = 1:(Nd-Td-Gd)
        j_corr = j + Td + Gd;
        % 2. - 5. Determine the noise threshold by measuring it within the training cells
        s1 = sum( sum( db2pow( RDM( i:i+Tr+Gr, j:j+Td+Gd ) ) ) );
        s2 = sum( sum( db2pow( RDM( i+Tr:i+Tr+Gr, j+Td:j+Td+Gd ) ) ) );
        threshold = pow2db( ( s1 - s2 ) / ( ( Tr + Gr ) * ( Td + Gd ) - Gr * Gd ) * offset );
        noise_level( i_corr, j_corr ) = threshold;

        % 6. Measuring the signal within the CUT
        signal = RDM( i_corr, j_corr );
        if (signal < threshold)
            signal = 0;
        else
            signal = 1;
        end

        % 8. Filter the signal above the threshold
        signal_cfar( i_corr, j_corr ) = signal;
    end
end




% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 

% REMARK: already done above by creating a bigger block where the elements
% that are supposed to be set to 0 here, are simply skipped. This is why,
% I use the i_corr (i corrected) and j_corr (j corrected) indicies to
% assigned values to the array.








% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure,surf(doppler_axis,range_axis,signal_cfar);
colorbar;


 
 