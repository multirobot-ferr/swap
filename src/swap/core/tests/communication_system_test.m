function communication_system_test
% Testing how the communication system will work 
% 
% The function tests in different locations (x_test,y_test) how
% the inflation of the position works. 

close all
tests = 10000;
angle = [+pi:-pi/360:-pi];

x_test = -8;
y_test = -8;
r_test = 1;
step = 0.5;

while (tests)
    
    % Moving the testing position to different locations
    x_test = x_test + step;
    if (x_test > 10)
        x_test = -10;
        y_test = y_test + step;
    end
    disp(['Testing an object in (' num2str(x_test) ',' num2str(y_test) ') with radius ' num2str(r_test)])
    
    x_ob = x_test;
    y_ob = y_test;
    r_ob = r_test;
    far_away = true;
    laser = 10*ones(size(angle));
    
    if (sqrt(x_ob^2 + y_ob^2) <= r_ob)
        % Requested because if the circle invades the area of the other
        % robot (should never happen), the laser does some wird things
        far_away = false;
        disp('It is not far away! (Should never happen)')
    end
    
    % Finding out the angle that points closer to the obstacle
    [~,ang_ob_idx] = min(abs(angle - atan2(y_ob,x_ob)));
    
    if (far_away)
        for j = [-1 +1] 
            idx = ang_ob_idx;
            guard = length(angle);

            while (guard > 0)
                % The system should stop by far earlier than this number
                guard = guard - 1;

                % Normal resolution (m is not infinite)
                if (angle(idx) ~= 0)
                    % m is the value of the line that goes in that direction
                    % y = m*x + b
                    m = tan(angle(idx));

                    % Since the line is centered in the (0,0), b = 0
                    % We solve the followin ecuation system
                    % y = m*x
                    % (x - x_ob)^2 + (y - y_ob)^2 = r_ob^2
                    % -------------------------------------
                    % (1 + m^2)*x^2 + (-2*x_ob - 2*y_ob*m)*x + (-r_ob^2 + y_ob^2 + x_ob^2) = 0

                    % Solving as a*x^2 + b*x + c = 0
                    a = (1 + m^2);
                    b = (-2*x_ob - 2*y_ob*m);
                    c = -r_ob^2 + y_ob^2 + x_ob^2;

                    discriminant = b^2 - 4*a*c;
                    if (discriminant >= 0)
                        % There are two hitting points on the circle
                        for i = [-1 +1]
                            x_hit = (-b - i*sqrt(discriminant))/(2*a);
                            y_hit = m*x_hit;

                            d = sqrt(x_hit^2 + y_hit^2);
                            laser(idx) = min([d laser(idx)]);
                        end
                    else
                        % The circle will be not hitten anymore
                        % Exists the while
                        break;
                    end
                else % m is infinite in y = m*x + b
                    % Solving the ecuations
                    % y = 0
                    % (x - x_ob)^2 + (y - y_ob)^2 = r_ob^2
                    % -------------------------------------
                    % x^2 + (-2*x_ob)*x + (x_ob^2 + y_ob^2 - r_ob^2) = 0

                    a = 1;
                    b = -2*x_ob;
                    c = x_ob^2 + y_ob^2 - r_ob^2;

                    discriminant = b^2 - 4*a*c;
                    if (discriminant >= 0)
                        for i = [-1 +1]
                            x_hit = (-b - i*sqrt(discriminant))/(2*a);    % Positive values is allways far away from where the object is
                            y_hit = 0;

                            d = x_hit; %sqrt(x_hit^2 + y_hit^2);
                            laser(idx) = min([d laser(idx)]);
                        end
                    else
                        % Exists the while
                        break;
                    end
                end

                idx = inf_idx( idx + j, length(angle));
            end
        end
    else % not far away
        % This should never happen, so we are only create a safety system
        % that introduces some information on the system
       
        laser(ang_ob_idx) = sqrt(x_ob^2 + y_ob^2) - r_ob;
    end
        
    plot([laser.*cos(angle) laser(1)*cos(angle(1))], ...
         [laser.*sin(angle) laser(1)*sin(angle(1))], 'b',       ... Laser representation
         x_ob, y_ob, 'ro',                                      ... Object representation
         0, 0, 'rx',                                            ... Center represenation
         10*cos(angle(ang_ob_idx)), 10*sin(angle(ang_ob_idx)), 'rx')    %   Orientation where the object is located
    xlim([-10.1 10.1])
    ylim([-10.1 10.1])

    tests = tests - 1;
    if (tests)
        drawnow
        pause(0.1)
    end

end


end

function idx = inf_idx(idx, size_idx)
    if idx < 1
        idx = size_idx;
    end
    if idx > size_idx
        idx = idx - size_idx;
    end
end

