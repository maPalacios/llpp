#!/bin/zsh
echo "tab = [" > graph.m
./demo --timing-mode | grep -Eo '[0-9]{1,4}\.[0-9]{1,4}' | tr -d '\n'  >> graph.m
echo " " | tr -d '\n' >> graph.m
./demo --timing-mode --openmp --np 4 | grep -Eo '[0-9]{1,4}\.[0-9]{1,4}' | tr -d '\n'  >> graph.m
echo " " | tr -d '\n' >> graph.m
./demo --timing-mode --pthreads --np 2 | grep -Eo '[0-9]{1,4}\.[0-9]{1,4}'  | tr -d '\n'  >> graph.m
echo " " | tr -d '\n' >> graph.m
./demo --timing-mode --vector | grep -Eo '[0-9]{1,4}\.[0-9]{1,4}'  | tr -d '\n'  >> graph.m
echo " " | tr -d '\n' >> graph.m
./demo --timing-mode --cuda | grep -Eo '[0-9]{1,4}\.[0-9]{1,4}'   >> graph.m

echo "];" >> graph.m
echo "

tab2 = { 'serial' 'OMP' 'Pthreads' 'Vector' 'Cuda' };
figure
tab(1,:)=tab(1,1)./tab(1,:);
bar(tab);
set(gca, 'XTick', 1:5, 'XTickLabel', tab2);


hold on;('Speedup of OpenMP and Pthreads');
ylabel('Speedup against serial'); % y-axis label
print('-dpng','file1.png');
" >> graph.m
