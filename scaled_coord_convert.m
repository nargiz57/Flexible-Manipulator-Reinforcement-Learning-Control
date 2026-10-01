function out = scaled_coord_convert(in, scl, direction)
% SCALED_COORD_CONVERT  Convert a 6x1 state between physical and scaled
%                       modal coordinates.
%
%   direction = "to_scaled"   : in is physical x,   out is scaled x_s
%   direction = "to_physical" : in is scaled x_s,    out is physical x
%
%   scl.s1, scl.s2 must match assumed_mode_rhs_scaled_mfile.m.

S = [1; scl.s1; scl.s2];   % applies to positions 1,3,5 (theta, eta1, eta2)
                            % and identically to velocities 2,4,6.
Svec = [S(1); S(1); S(2); S(2); S(3); S(3)];

switch direction
    case "to_scaled"
        out = in ./ Svec;
    case "to_physical"
        out = in .* Svec;
    otherwise
        error("direction must be ""to_scaled"" or ""to_physical"".");
end

end
