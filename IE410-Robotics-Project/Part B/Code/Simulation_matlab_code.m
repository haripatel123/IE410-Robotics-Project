%% =========================================================================
%  Jansen 8-bar Gait Trainer Simulation
%  IE410 — Introduction to Robotics  |  Winter 2026
%
%  Reference: Shin et al. 2018 (JMR) and Jadav et al. (sn-article)
%  Mechanism:  8-bar, 12-link modified Jansen mechanism
%
%  Topology (reading the ternary / parallelogram structure correctly):
%   Upper four-bar : P0-P1-P2-P3      links L1, L2, L3, L4
%   Lower four-bar : P0-P1-P5-P3      links L1, L7, L8, L4
%   Ternary triangle (L3-L5-L6)       vertices P3, P2, P2t
%       L3 = P3–P2  (upper follower, shared)
%       L5 = P2–P2t
%       L6 = P3–P2t   ← second anchor is P3, NOT P5
%   Parallelogram (L6-L8-L9-L10)      vertices P3, P2t, P6, P5
%       L6  = P3–P2t   (shared with triangle)
%       L10 = P2t–P6  ← link from P2t is L10, NOT L6
%       L9  = P5–P6
%       L8  = P5–P3   (shared with lower four-bar)
%   Foot triangle (L9-L11-L12)        vertices P5, P6, PE
%       L11 = P5–PE
%       L12 = P6–PE
% =========================================================================
clear; clc; close all;

%% =========================================================================
%  1. LINK LENGTHS  (cm) — from Jadav et al. Table 3 / Shin et al. Fig. 4
% ==========================================================================
L1  = 11.0;   % crank            (adjustable)
L2  = 45.0;
L3  = 36.0;
L4  = 33.0;   % ground link      (adjustable — stride length)
L5  = 48.5;
L6  = 41.5;
L7  = 60.5;
L8  = 41.5;   % lower follower   (adjustable — step height)
L9  = 42.0;
L10 = 43.0;
L11 = 26.5;
L12 = 54.5;

Ls = struct('L1',L1,'L2',L2,'L3',L3,'L4',L4,'L5',L5,'L6',L6, ...
            'L7',L7,'L8',L8,'L9',L9,'L10',L10,'L11',L11,'L12',L12);

%% =========================================================================
%  2. FIXED PIVOTS
% ==========================================================================
P0 = [0; 0];      % left  fixed pivot (crank base)
P3 = [L4; 0];     % right fixed pivot (ground link end)

%% =========================================================================
%  3. SOLVE KINEMATICS OVER ONE FULL REVOLUTION
% ==========================================================================
N         = 720;
theta_arr = linspace(0, 2*pi, N+1);
theta_arr(end) = [];

PE_traj = nan(2, N);
all_pos = cell(N, 1);

fprintf('Computing kinematics (%d steps)...\n', N);
for k = 1:N
    try
        pos = jansen_kinematics(theta_arr(k), Ls, P0, P3);
        PE_traj(:,k) = pos.PE;
        all_pos{k}   = pos;
    catch
        % singular/impossible configuration → stays NaN
    end
end
nValid = sum(~any(isnan(PE_traj)));
fprintf('Done.  Valid frames: %d / %d\n\n', nValid, N);

%% =========================================================================
%  4. FOOT-POINT TRAJECTORY
% ==========================================================================
figure('Name','Foot-Point Trajectory','Color','w','Position',[100 100 800 600]);
plot(PE_traj(1,:), PE_traj(2,:), 'b-', 'LineWidth', 2.5); hold on;
plot(PE_traj(1,1), PE_traj(2,1), 'ro', 'MarkerSize', 12, 'MarkerFaceColor','r');
xlabel('X (cm)','FontSize',14); ylabel('Y (cm)','FontSize',14);
title('Jansen Mechanism — Foot-Point Trajectory','FontSize',16);
legend('Foot trajectory','Start point','Location','best');
axis equal; grid on; set(gca,'FontSize',12);
saveas(gcf,'foot_trajectory.png');
fprintf('Saved: foot_trajectory.png\n');

%% =========================================================================
%  5. AXIS LIMITS  (for animation & snapshots)
% ==========================================================================
all_x = []; all_y = [];
for k = 1:N
    if ~isempty(all_pos{k})
        c = [[P0, P3], struct2coords(all_pos{k})];
        all_x = [all_x, c(1,:)]; %#ok
        all_y = [all_y, c(2,:)]; %#ok
    end
end
xlims = [min(all_x)-15, max(all_x)+15];
ylims = [min(all_y)-15, max(all_y)+15];

%% =========================================================================
%  6. ANIMATION  (GIF)
% ==========================================================================
fig_anim = figure('Name','Animation','Color','w','Position',[100 100 900 700]);
nf  = 90;
fi  = round(linspace(1, N, nf));
gif = 'mechanism_animation.gif';

fprintf('Generating animation (%d frames)...\n', nf);
for k = 1:nf
    idx = fi(k);
    pos = all_pos{idx};
    if isempty(pos), continue; end

    clf; hold on;
    plot(PE_traj(1,:), PE_traj(2,:), '-', 'LineWidth', 1, 'Color', [0.6 0.6 1]);
    draw_jansen(pos, P0, P3);
    plot(pos.PE(1), pos.PE(2), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    plot(P0(1), P0(2), 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
    plot(P3(1), P3(2), 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
    plot(PE_traj(1,1:idx), PE_traj(2,1:idx), 'r-', 'LineWidth', 1.8);

    xlabel('X (cm)','FontSize',12); ylabel('Y (cm)','FontSize',12);
    title(sprintf('Jansen Mechanism  —  \\theta = %.1f°', ...
          rad2deg(theta_arr(idx))), 'FontSize', 14);
    axis equal; grid on; xlim(xlims); ylim(ylims);
    drawnow;

    fr = getframe(fig_anim);
    [im, cm_] = rgb2ind(frame2im(fr), 256);
    if k == 1
        imwrite(im, cm_, gif, 'gif', 'Loopcount', inf, 'DelayTime', 0.04);
    else
        imwrite(im, cm_, gif, 'gif', 'WriteMode', 'append', 'DelayTime', 0.04);
    end
end
fprintf('Saved: %s\n', gif);

%% =========================================================================
%  7. SNAPSHOT SEQUENCE  (6 crank angles)
% ==========================================================================
figure('Name','Snapshots','Color','w','Position',[50 50 1400 500]);
snap_deg = [0 60 120 180 240 300];
for s = 1:6
    subplot(2,3,s); hold on;
    idx = max(1, round(snap_deg(s)/360*N) + 1);
    pos = all_pos{idx};
    if isempty(pos)
        title(sprintf('%d° — no solution', snap_deg(s)));
        continue;
    end
    plot(PE_traj(1,:), PE_traj(2,:), '-', 'LineWidth', 0.8, 'Color', [0.7 0.7 1]);
    draw_jansen(pos, P0, P3);
    plot(pos.PE(1), pos.PE(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot(P0(1), P0(2), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    plot(P3(1), P3(2), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    title(sprintf('\\theta = %d°', snap_deg(s)), 'FontSize', 12);
    axis equal; grid on; xlim(xlims); ylim(ylims);
end
sgtitle('Mechanism at Six Crank Positions', 'FontSize', 16);
saveas(gcf, 'mechanism_snapshots.png');
fprintf('Saved: mechanism_snapshots.png\n');

%% =========================================================================
%  8. LINK LENGTH VARIATION  (L4 = stride length,  L8 = step height)
% ==========================================================================
figure('Name','Link Variation','Color','w','Position',[100 100 1200 500]);

% --- Vary L4 (stride length) ---
subplot(1,2,1); hold on;
L4_vals = [27 30 33 36 39];
cmap = lines(numel(L4_vals));
lgd4 = {};
for j = 1:numel(L4_vals)
    Lm = Ls; Lm.L4 = L4_vals(j);
    P3m = [Lm.L4; 0];
    PE_m = nan(2, N);
    for k = 1:N
        try
            p = jansen_kinematics(theta_arr(k), Lm, P0, P3m);
            PE_m(:,k) = p.PE;
        catch, end
    end
    if sum(~any(isnan(PE_m))) > 10
        plot(PE_m(1,:), PE_m(2,:), '-', 'LineWidth', 2, 'Color', cmap(j,:));
        lgd4{end+1} = sprintf('L_4 = %d cm', L4_vals(j)); %#ok
    end
end
xlabel('X (cm)'); ylabel('Y (cm)');
title('Effect of L_4 on Stride Length', 'FontSize', 14);
legend(lgd4, 'Location', 'best'); axis equal; grid on;

% --- Vary L8 (step height) ---
subplot(1,2,2); hold on;
L8_vals = [36 38.5 41.5 44 47];
cmap = lines(numel(L8_vals));
lgd8 = {};
for j = 1:numel(L8_vals)
    Lm = Ls; Lm.L8 = L8_vals(j);
    P3m = [Lm.L4; 0];
    PE_m = nan(2, N);
    for k = 1:N
        try
            p = jansen_kinematics(theta_arr(k), Lm, P0, P3m);
            PE_m(:,k) = p.PE;
        catch, end
    end
    if sum(~any(isnan(PE_m))) > 10
        plot(PE_m(1,:), PE_m(2,:), '-', 'LineWidth', 2, 'Color', cmap(j,:));
        lgd8{end+1} = sprintf('L_8 = %.1f cm', L8_vals(j)); %#ok
    end
end
xlabel('X (cm)'); ylabel('Y (cm)');
title('Effect of L_8 on Step Height', 'FontSize', 14);
legend(lgd8, 'Location', 'best'); axis equal; grid on;
saveas(gcf, 'link_length_variation.png');
fprintf('Saved: link_length_variation.png\n');

%% =========================================================================
%  9. GAIT TRAJECTORY COMPARISON
% ==========================================================================
t_r = linspace(0, 2*pi, 300);
x_r = 22*cos(t_r);
y_r = 10*sin(t_r) + 10*max(sin(t_r), 0);   % arch on top, flat on bottom

valid = ~any(isnan(PE_traj));
PEx = PE_traj(1,:) - mean(PE_traj(1, valid));
PEy = PE_traj(2,:) - mean(PE_traj(2, valid));
xr  = x_r - mean(x_r);
yr  = y_r - mean(y_r);

figure('Name','Gait Comparison','Color','w','Position',[100 100 900 600]);
plot(xr, yr, 'k--', 'LineWidth', 2); hold on;
plot(PEx, PEy, 'b-', 'LineWidth', 2);
xlabel('X (cm)', 'FontSize', 14); ylabel('Y (cm)', 'FontSize', 14);
title('Simulated vs. Reference Human Gait Trajectory', 'FontSize', 16);
legend('Reference meta-trajectory (Shin et al.)', 'Simulated Jansen mechanism', ...
       'Location', 'best');
axis equal; grid on; set(gca, 'FontSize', 12);
saveas(gcf, 'gait_comparison.png');
fprintf('Saved: gait_comparison.png\n');

%% =========================================================================
%  10. X, Y vs GAIT CYCLE %
% ==========================================================================
gp = linspace(0, 100, N);
figure('Name','XY vs Gait Cycle','Color','w','Position',[100 100 900 500]);
subplot(2,1,1);
plot(gp, PE_traj(1,:), 'b-', 'LineWidth', 2);
xlabel('Gait Cycle (%)'); ylabel('X (cm)');
title('End-Effector X over Gait Cycle'); grid on;
subplot(2,1,2);
plot(gp, PE_traj(2,:), 'r-', 'LineWidth', 2);
xlabel('Gait Cycle (%)'); ylabel('Y (cm)');
title('End-Effector Y over Gait Cycle'); grid on;
saveas(gcf, 'trajectory_vs_gait_cycle.png');
fprintf('Saved: trajectory_vs_gait_cycle.png\n');

%% =========================================================================
%  11. METRICS
% ==========================================================================
xv     = PE_traj(1, valid);
yv     = PE_traj(2, valid);
x_span = max(xv) - min(xv);
y_span = max(yv) - min(yv);
ar     = polyarea(xv, yv);

fprintf('\n=== TRAJECTORY METRICS ===\n');
fprintf('  X-span (stride length) : %.2f cm\n', x_span);
fprintf('  Y-span (step height)   : %.2f cm\n', y_span);
fprintf('  Enclosed area (AUCT)   : %.2f cm^2\n', ar);
fprintf('==========================\n\n');
fprintf('All figures saved. Simulation complete.\n');


%% =========================================================================
%%  LOCAL FUNCTIONS
%% =========================================================================

function pos = jansen_kinematics(theta2, L, P0, P3)
%JANSEN_KINEMATICS  Solves the full 8-bar Jansen mechanism.
%
%  CORRECT TOPOLOGY (Shin et al. 2018, Jadav et al.):
%
%  Upper four-bar  (L1-L2-L3-L4):
%    P0 → P1 : L1  crank
%    P1 → P2 : L2  upper coupler
%    P3 → P2 : L3  upper follower
%    P0 → P3 : L4  ground (fixed)
%
%  Lower four-bar  (L1-L7-L8-L4):
%    P0 → P1 : L1  same crank
%    P1 → P5 : L7  lower coupler
%    P3 → P5 : L8  lower follower
%    P0 → P3 : L4  same ground
%
%  Ternary triangle  (L3-L5-L6)   ← vertices: P3, P2, P2t
%    P3 → P2  : L3  (shared with upper four-bar)
%    P2 → P2t : L5
%    P3 → P2t : L6  ← second anchor is P3 (fixed pivot), NOT P5
%
%  "Parallelogram"  (L6-L8-L9-L10) ← vertices: P3, P2t, P6, P5
%    P3  → P2t : L6   (shared with ternary triangle)
%    P2t → P6  : L10  ← link from P2t is L10, NOT L6
%    P5  → P6  : L9
%    P5  → P3  : L8   (shared with lower four-bar)
%
%  Foot triangle  (L9-L11-L12)  ← vertices: P5, P6, PE
%    P5 → PE : L11
%    P6 → PE : L12

    % --- crank tip ---
    P1 = P0 + L.L1 * [cos(theta2); sin(theta2)];

    % --- upper four-bar  (P2 above the ground line) ---
    P2 = cci(P1, L.L2, P3, L.L3, +1);

    % --- lower four-bar  (P5 below the ground line) ---
    P5 = cci(P1, L.L7, P3, L.L8, -1);

    % --- ternary triangle vertex  (P2t) ---
    %   FIX: second anchor is P3 (fixed pivot) with radius L6
    %        NOT P5 with L10 as the original buggy code had
    %   Branch +1 places P2t to the right of P2→P3, towards the
    %   working area of the mechanism (between the two four-bars).
    P2t = cci(P2, L.L5, P3, L.L6, +1);

    % --- parallelogram corner  (P6) ---
    %   FIX: radius from P2t is L10 (not L6 as the original buggy code had)
    %   Branch +1 gives the configuration consistent with a parallelogram:
    %   P6 ≈ P5 + (P2t − P3),  i.e. P3→P2t ∥ P5→P6
    P6 = cci(P2t, L.L10, P5, L.L9, +1);

    % --- end-effector / foot  (PE) ---
    %   Pick the lower (more negative Y) of the two solutions so the
    %   foot hangs below the mechanism throughout the cycle.
    PEa = cci(P5, L.L11, P6, L.L12, +1);
    PEb = cci(P5, L.L11, P6, L.L12, -1);
    PE  = pick_lower(PEa, PEb);

    pos.P1  = P1;
    pos.P2  = P2;
    pos.P2t = P2t;
    pos.P5  = P5;
    pos.P6  = P6;
    pos.PE  = PE;
end

% -------------------------------------------------------------------------
function P = cci(C1, r1, C2, r2, sign_branch)
%CCI  Circle-circle intersection (two-circle intersection point).
%
%  sign_branch = +1 → solution to the LEFT  of direction C1→C2
%  sign_branch = -1 → solution to the RIGHT of direction C1→C2
%  (left/right defined by rotating C1→C2 by 90° counter-clockwise)

    d = norm(C2 - C1);

    % Guard: circles must intersect
    if d > r1 + r2 + 1e-9 || d < abs(r1 - r2) - 1e-9
        error('No CCI: circles do not intersect (d=%.4f, r1=%.4f, r2=%.4f)', ...
              d, r1, r2);
    end
    if d < 1e-12
        error('No CCI: coincident centres');
    end

    % Clamp d into the valid range for numerical stability near singularities
    d = min(d, r1 + r2 - 1e-12);
    d = max(d, abs(r1 - r2) + 1e-12);

    a  = (r1^2 - r2^2 + d^2) / (2*d);
    h  = sqrt(max(r1^2 - a^2, 0));

    ex = (C2 - C1) / d;
    ey = [-ex(2); ex(1)];   % 90° CCW of ex

    Pm = C1 + a * ex;
    P  = Pm + sign_branch * h * ey;
end

% -------------------------------------------------------------------------
function P = pick_lower(Pa, Pb)
%PICK_LOWER  Return whichever point has the smaller (more negative) Y.
    if Pa(2) <= Pb(2)
        P = Pa;
    else
        P = Pb;
    end
end

% -------------------------------------------------------------------------
function coords = struct2coords(pos)
%STRUCT2COORDS  Flatten all joint positions from the pos struct into a 2×N matrix.
    fields = fieldnames(pos);
    coords = [];
    for k = 1:numel(fields)
        coords = [coords, pos.(fields{k})]; %#ok
    end
end

% -------------------------------------------------------------------------
function draw_jansen(pos, P0, P3)
%DRAW_JANSEN  Draw all 12 links with colour-coded sub-mechanisms.
%
%  Colour scheme:
%   Grey   — L4          ground link
%   Red    — L1          crank
%   Blue   — L2, L3      upper four-bar
%   Green  — L7, L8      lower four-bar
%   Orange — L5,L6,L9,L10   ternary triangle + parallelogram
%   Purple — L11, L12    foot triangle → end-effector

    RED  = [0.85 0.10 0.10];
    GREY = [0.40 0.40 0.40];
    BLUE = [0.10 0.20 0.90];
    GRN  = [0.05 0.65 0.10];
    ORG  = [0.90 0.45 0.00];
    PUR  = [0.60 0.00 0.80];
    LW   = 2.5;

    % L4  ground link  (P0 – P3)
    lk(P0,       P3,       GREY, LW);
    % L1  crank  (P0 – P1)
    lk(P0,       pos.P1,   RED,  LW);
    % L2  upper coupler  (P1 – P2)
    lk(pos.P1,   pos.P2,   BLUE, LW);
    % L3  upper follower  (P3 – P2)  [also a side of the ternary triangle]
    lk(P3,       pos.P2,   BLUE, LW);
    % L7  lower coupler  (P1 – P5)
    lk(pos.P1,   pos.P5,   GRN,  LW);
    % L8  lower follower  (P3 – P5)  [also a side of the parallelogram]
    lk(P3,       pos.P5,   GRN,  LW);
    % L5  ternary triangle  (P2 – P2t)
    lk(pos.P2,   pos.P2t,  ORG,  LW);
    % L6  ternary triangle / parallelogram  (P3 – P2t)
    %   FIX: was lk(pos.P5, pos.P2t, …) — WRONG anchor.
    %        L6 connects P3 (fixed pivot) to P2t, not P5 to P2t.
    lk(P3,       pos.P2t,  ORG,  LW);
    % L10 parallelogram  (P2t – P6)
    lk(pos.P2t,  pos.P6,   ORG,  LW);
    % L9  parallelogram / foot triangle  (P5 – P6)
    lk(pos.P5,   pos.P6,   ORG,  LW);
    % L11 foot triangle  (P5 – PE)
    lk(pos.P5,   pos.PE,   PUR,  LW);
    % L12 foot triangle  (P6 – PE)
    lk(pos.P6,   pos.PE,   PUR,  LW);

    % Joint markers
    all_pts = [P0, P3, pos.P1, pos.P2, pos.P2t, pos.P5, pos.P6, pos.PE];
    plot(all_pts(1,:), all_pts(2,:), 'o', 'MarkerSize', 7, ...
         'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.3 0.3 0.3]);
end

% -------------------------------------------------------------------------
function lk(A, B, col, lw)
    plot([A(1) B(1)], [A(2) B(2)], '-', 'Color', col, 'LineWidth', lw);
end