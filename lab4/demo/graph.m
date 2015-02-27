tab = [
695.198 459.649 475.393 269.161 199.672
];


tab2 = { 'serial' 'OMP' 'Pthreads' 'Vector' 'Cuda' };
figure
tab(1,:)=tab(1,1)./tab(1,:);
bar(tab);
set(gca, 'XTick', 1:5, 'XTickLabel', tab2);


hold on;('Speedup of OpenMP and Pthreads');
ylabel('Speedup against serial'); % y-axis label
print('-dpng','file2.png');

