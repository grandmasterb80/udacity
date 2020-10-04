% required physical constants
c = 300000000;
%299 792 458

% radar specs
max_range = 300;
range_res = 1;

% TODO : Find the Bsweep of chirp for 1 m resolution
Bsweep = c / ( 2 * range_res );

% TODO : Calculate the chirp time based on the Radar's Max Range
chirp_time = 5.5 * ( 2 * max_range ) / c;

% TODO : define the frequency shifts (given in MHz ==> convert to Hz)
bf = [0; 1.1; 13; 24] * 1000000;

calculated_range = c * chirp_time * bf / ( 2 * Bsweep );

% Display the calculated range
disp(calculated_range);
