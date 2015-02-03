close all;
open rs_ls.fig;
%grid on;

h = findobj(gca, 'Type', 'line');
set(h,'MarkerSize',5)
set(gca,'Color','none');
%set(gcf, 'Color', 'w') 
export_fig('rs_ls', '-pdf', '-transparent' );