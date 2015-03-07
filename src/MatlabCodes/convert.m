close all;
open random3.fig;
%grid on;

h = findobj(gca, 'Type', 'line');
set(h,'MarkerSize',5)
set(gca,'Color','none');
%set(gcf, 'Color', 'w') 
export_fig('random3', '-pdf', '-transparent' );