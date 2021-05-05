%mex COPTIMFLAGS="-O3"  planner.cpp
map_ind = 1;
source_ind = 1;
start_ind = 1;


maps = ["map3.png", "map4.png", "map5.png"];
sources = {[105, 35; 50, 100], [70, 30; 60, 125]};
starts = {[30, 20], [165, 165], [40, 10], [120, 40]};

tic();
runtest(starts{start_ind}, maps(map_ind), sources{source_ind});
toc()