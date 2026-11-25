function [lat, lon] = localENUToGPS(xLocal, yLocal, refLat, refLon)
% LOCALENUTOGPS Convert local ENU Cartesian (X, Y) in meters to global GPS coordinates (Lat, Lon).
% Inverse of gpsToLocalENU.m using standard WGS84 approximation.
%
% Inputs:
%   xLocal, yLocal - Local East and North offsets in meters
%   refLat, refLon - Reference origin GPS coordinates (degrees)
%
% Outputs:
%   lat, lon - Converted global GPS coordinates (degrees)

if nargin < 4
    error('localENUToGPS requires 4 inputs: xLocal, yLocal, refLat, refLon');
end

R = 6378137; % Earth radius in meters
dLat = yLocal / R;
dLon = xLocal / (R * cos(deg2rad(refLat)));

lat = refLat + rad2deg(dLat);
lon = refLon + rad2deg(dLon);

end
