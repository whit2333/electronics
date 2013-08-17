#!/usr/bin/gnuplot
# plots data output by sfgeiger
# file is cpm_sfgeiger.dat
f(xx)= m*xx + bb
m = 0.0
bb = 25.0
set   autoscale                        # scale axes automatically
unset log                              # remove any log-scaling
unset label                            # remove any previous labels
set xtic auto                          # set xtics automatically
set ytic auto                          # set ytics automatically
set title "Geiger counter"
set xlabel "Time"
set ylabel "cpm"
set xdata time
set timefmt "%s"
#      set key 0.01,100
#      set label "Yield Point" at 0.003,260
#      set arrow from 0.0028,250 to 0.003,280
#      set xr [0.0:0.022]
set yr [0:75]
fit f(x) "cpm_sfgeiger.dat" using 1:2 via bb
set terminal png size 900,500
set output "sfgeiger.png"
plot    "cpm_sfgeiger.dat" using 1:2 title 'cpm' with lines, f(x) 

#, \
#            "force.dat" using 1:3 title 'Beam' with points
