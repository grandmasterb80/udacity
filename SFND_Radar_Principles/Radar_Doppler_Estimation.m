% Doppler Velocity Calculation
c = 3*10^8;         %speed of light
frequency = 77e9;   %frequency in Hz

% TODO : Calculate the wavelength
lambda = c / frequency;

% TODO : Define the doppler shifts in Hz using the information from above 
fd = [3; -4.5; 11; -3;] * 1000 ;

% TODO : Calculate the velocity of the targets  fd = 2*vr/lambda
vr = fd * lambda / 2;

% TODO: Display results
disp(vr);


% 2. Range Estimation:
% Formulas for the range should use brackets (on the slides):
% range = c * chirp_time * bf / ( 2 * Bsweep );
%
% 3. Doppler Estimation:
% A sign in one number was missing ==> I had to watch the solution to find
% out that I used the wrong numbers.