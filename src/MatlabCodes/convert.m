close all;
open traj_circle.fig;
%grid on;

h = findobj(gca, 'Type', 'line');
set(h,'MarkerSize',5)
set(gca,'Color','none');
%set(gcf, 'Color', 'w') 
export_fig('traj_circle', '-pdf', '-transparent' );