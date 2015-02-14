#!/bin/bash
export LD_LIBRARY_PATH=.
echo "serial = " | tr -d '\n' > graph.m
    ./demo --timing-mode | grep -Eo '[0-9]{1,4}\.[0-9]{1,4}' | tr -d '\n'  >> graph.m
echo ";" >> graph.m

echo "tab = [" >> graph.m
for i in {1..8}
    do
    echo "$i " | tr -d '\n' >> graph.m
    ./demo --timing-mode --openmp --np $i | grep -Eo '[0-9]{1,4}\.[0-9]{1,4}' | tr -d '\n'  >> graph.m
    echo " " | tr -d '\n' >> graph.m
    ./demo --timing-mode --pthreads --np $i | grep -Eo '[0-9]{1,4}\.[0-9]{1,4}'   >> graph.m
    
    done
echo "];" >> graph.m
echo "
tab(:,2)=serial./tab(:,2);
tab(:,3)=serial./tab(:,3);
plot(tab(:,1),tab(:,2),'-o', tab(:,1),tab(:,3),'-*');
legend('openMP', 'Pthreads');
%p=polyfit(tab(:,1),tab(:,2),1);
hold on;

title('Speedup of OpenMP and Pthreads')
xlabel('Number of Processes') % x-axis label
ylabel('Speedup against serial') % y-axis label
%p=polyfit(tab(:,1),tab(:,3),1);
pause;" >> graph.m
