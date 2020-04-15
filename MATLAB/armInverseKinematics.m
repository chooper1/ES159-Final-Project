%% Inverse Kinematics Function

function q = armInverseKinematics(ee_pos)
    % prepare array
    q = zeros(2, 1);

    % link lengths and values
    L1 = 1;
    L2 = 1;
    px = ee_pos(1);
    py = ee_pos(2);
    
    c2 = (px^2 + py^2 - L1^2 - L2^2) / (2*L1*L2);
    s2 = sqrt(1-c2^2);
    q(2) = atan2(s2, c2); %plus or minus
    
    q(1) = atan2(py,px) - atan2(L2*s2, L1 + L2*c2);
    
    % convert to degrees
    q = rad2deg(q);
    
    % set top and bottom joint angle limits
    top_limit = [180, 180];
    bottom_limit = [-180, 0];
    
    % if any one of the entries is not within limits, throw an error
    for i = 1:2
        if (q(i) > top_limit(i))
            q(i) = q(i) - 360;
        end
        if (q(i) < bottom_limit(i))
            q(i) = q(i) + 360;
        end
    end
    
    for i = 1:2
        if (q(i) > top_limit(i) || q(i) < bottom_limit(i))
            error('IK result is not within bounds!')
        end
    end
end
