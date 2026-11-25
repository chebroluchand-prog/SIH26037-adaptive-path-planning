function gpsMeasurement = simulateGPSSensor(egoState, refLat, refLon, params)
% SIMULATEGPSSENSOR Lightweight GPS sensor receiver simulator with noise & dropouts.
% Converts true ego local pose (X, Y) to noisy global GPS coordinates (Lat, Lon).
%
% Inputs:
%   egoState - [x, y, yaw, v, w] true ego vehicle state
%   refLat   - Reference origin latitude (degrees)
%   refLon   - Reference origin longitude (degrees)
%   params   - Parameter struct with fields:
%                .sigmaGPS (default: 1.5 m)
%                .dropout  (default: false)
%
% Outputs:
%   gpsMeasurement - Struct with fields:
%                      .lat        - Measured latitude (degrees)
%                      .lon        - Measured longitude (degrees)
%                      .alt        - Altitude (meters, default: 542.0)
%                      .xNoisy     - Re-converted noisy East position (m)
%                      .yNoisy     - Re-converted noisy North position (m)
%                      .isValid    - Boolean indicating fix validity
%                      .status     - String status ('VALID' or 'DROPOUT')

if nargin < 4, params = struct(); end
if ~isfield(params, 'sigmaGPS'), params.sigmaGPS = 1.5; end % 1.5m standard deviation
if ~isfield(params, 'dropout'),  params.dropout  = false; end

% True local pose
xTrue = egoState(1);
yTrue = egoState(2);

% Convert true local pose to true GPS coordinates
[latTrue, lonTrue] = localENUToGPS(xTrue, yTrue, refLat, refLon);

% Check dropout flag
if params.dropout
    gpsMeasurement.lat     = NaN;
    gpsMeasurement.lon     = NaN;
    gpsMeasurement.alt     = NaN;
    gpsMeasurement.xNoisy  = NaN;
    gpsMeasurement.yNoisy  = NaN;
    gpsMeasurement.isValid = false;
    gpsMeasurement.status  = "DROPOUT";
    return;
end

% Translate Cartesian noise (meters) to Lat/Lon noise (degrees)
R = 6378137; % Earth radius in meters
nE = params.sigmaGPS * randn(); % East noise in meters
nN = params.sigmaGPS * randn(); % North noise in meters

dLatNoise = rad2deg(nN / R);
dLonNoise = rad2deg(nE / (R * cos(deg2rad(refLat))));

latNoisy = latTrue + dLatNoise;
lonNoisy = lonTrue + dLonNoise;

% Re-convert back to local ENU for local navigation pipeline
[xNoisy, yNoisy] = gpsToLocalENU(latNoisy, lonNoisy, refLat, refLon);

gpsMeasurement.lat     = latNoisy;
gpsMeasurement.lon     = lonNoisy;
gpsMeasurement.alt     = 542.0; % Nominal altitude (e.g. Hyderabad mean sea level)
gpsMeasurement.xNoisy  = xNoisy;
gpsMeasurement.yNoisy  = yNoisy;
gpsMeasurement.isValid = true;
gpsMeasurement.status  = "VALID";

end
