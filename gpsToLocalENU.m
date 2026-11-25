function [xLocal, yLocal] = gpsToLocalENU(lat, lon, refLat, refLon)
% GPSTOLOCALENU Convert global GPS coordinates (Lat, Lon) to local ENU Cartesian (X, Y) in meters.
% Uses standard WGS84 spherical/equirectangular approximation relative to reference origin [refLat, refLon].
%
% Inputs:
%   lat, lon   - Target global GPS coordinates (degrees)
%   refLat, refLon - Reference origin GPS coordinates (degrees)
%
% Outputs:
%   xLocal - Local East offset in meters
%   yLocal - Local North offset in meters

if nargin < 4
    error('gpsToLocalENU requires 4 inputs: lat, lon, refLat, refLon');
end

R = 6378137; % Earth radius in meters (WGS84 semi-major axis)
dLat = deg2rad(lat - refLat);
dLon = deg2rad(lon - refLon);

xLocal = R * dLon * cos(deg2rad(refLat));
yLocal = R * dLat;

end
