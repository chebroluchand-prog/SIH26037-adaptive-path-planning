function [z, dzdx, dzdy] = getTerrainElevation(x, y, mode)
% GETTERRAINELEVATION Lightweight terrain surface elevation and gradient function.
% Inputs:
%   x, y - Local ENU Cartesian coordinates (m)
%   mode - 'flat' (default, z=0) or 'sloped' (z = 0.02 * x)
% Outputs:
%   z    - Surface elevation (m)
%   dzdx - Partial derivative w.r.t x
%   dzdy - Partial derivative w.r.t y

if nargin < 3 || isempty(mode), mode = 'flat'; end

switch lower(mode)
    case 'sloped'
        z = 0.02 * x;
        dzdx = 0.02 * ones(size(x));
        dzdy = zeros(size(y));
    case 'rolling'
        z = 0.5 * sin(0.05 * x) + 0.01 * x;
        dzdx = 0.025 * cos(0.05 * x) + 0.01;
        dzdy = zeros(size(y));
    otherwise % 'flat'
        z = zeros(size(x));
        dzdx = zeros(size(x));
        dzdy = zeros(size(y));
end
end
