clear all; clc; close all;
save_res = 1; % Do you want to save results as pic/… ?
% Signal direction
alpha_s = deg2rad(30);
beta_s = deg2rad(0);
% Jammer direction
beta_j = deg2rad(0);
% Probe signal direction
alpha = deg2rad(-180:2:180);
beta = deg2rad(-90:3:90);
% One AM's radiation pattern
Fa = ones(length(beta), 1) * (1 + cos(alpha - pi/2)).^2;
[alpha_mesh, beta_mesh] = meshgrid( alpha, beta );
[xa, ya, za] = sph2cart(beta_mesh, alpha_mesh, Fa);
% Plot RP for one AM
figure(1);
surf(xa, ya, za);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Radiation pattern for one AM');
axis equal
% Memory allocation
F = nan(length(beta), length(alpha));
q_dB = 3; % jam/noise, dB
q = 10^(q_dB/10); % ... in absolute value
% Figure for common radiation pattern
figure(2)
pos = get(gcf, 'Position'); pos(3) = 800; set(gcf, 'Position', pos);
for alpha_j = deg2rad(0:5:90)
    C = H(alpha_j, beta_j);
    Dn = q * C * C' + eye(4);
    Hf = H(alpha_s, beta_s);
    beta_w = Dn \ Hf / (Hf' * (Dn \ Hf));
    for a = 1:length(alpha)
        for b = 1:length(beta)
            U = beta_w' * H( alpha(a), beta(b) );
            F(b, a) = abs(U)^2;
        end
    end
    F = Fa .* F;
    [x, y, z] = sph2cart(beta_mesh, alpha_mesh, F);
    b0 = ceil(length(beta)/2); % index for beta = 0
    Fb0 = F(b0, :);
    subplot(1,2,1)
    surf(x, y, z)
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal
    subplot(1,2,2)
    polar(alpha, Fb0); % Radiation pattern
    hold on
    polar( [alpha_s alpha_s], [0 max(Fb0)], 'g'); % Line to signal
    polar( [alpha_j alpha_j], [0 max(Fb0)], 'r'); % Line to jam
    hold off
    drawnow
        if save_res % Save figure(2) to png
        s = sprintf('DN_alpha_j_%03.0f.png', round(rad2deg(alpha_j)));
        saveas(gcf, s, 'png');
        fprintf('Figure is saved at %s\n', s)
        end
end
