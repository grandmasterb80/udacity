filter = trackingKF('MotionModel', model, 'State', state, 'MeasurementModel', measurementModel, 'StateCovariance', stateCovrariance, 'MeasurementNoise', measurementNoise)
H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1]
z  = H * x
x = H' * z
state = H' * detection.Measurement
stateCovariance =H' * detection.MeasurementNoise*H
