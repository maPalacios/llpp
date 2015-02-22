#!/bin/zsh
echo "tab = [" > graph.m
./demo --timing-mode hugeScenario.xml | grep -Eo '[0-9]{1,4}\.[0-9]{1,4}' | tr -d '\n'  >> graph.m
echo " " | tr -d '\n' >> graph.m
./demo --timing-mode --openmp --np 4  hugeScenario.xml| grep -Eo '[0-9]{1,4}\.[0-9]{1,4}' | tr -d '\n'  >> graph.m
echo " " | tr -d '\n' >> graph.m
./demo --timing-mode --pthreads --np 2  hugeScenario.xml| grep -Eo '[0-9]{1,4}\.[0-9]{1,4}'  | tr -d '\n'  >> graph.m
echo " " | tr -d '\n' >> graph.m
./demo --timing-mode --vector  hugeScenario.xml| grep -Eo '[0-9]{1,4}\.[0-9]{1,4}'  | tr -d '\n'  >> graph.m
echo " " | tr -d '\n' >> graph.m
./demo --timing-mode --cuda  hugeScenario.xml| grep -Eo '[0-9]{1,4}\.[0-9]{1,4}'   >> graph.m

echo "];" >> graph.m
echo "

tab2 = { 'serial' 'OMP' 'Pthreads' 'Vector' 'Cuda' };
figure
tab(1,:)=tab(1,1)./tab(1,:);
bar(tab);
set(gca, 'XTick', 1:5, 'XTickLabel', tab2);


hold on;('Speedup of OpenMP and Pthreads');
ylabel('Speedup against serial'); % y-axis label
print('-dpng','file2.png');
" >> graph.m
