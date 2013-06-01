# plots data output by sfgeiger
# file is cpm_sfgeiger.dat
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
      set terminal png
      set output "sfgeiger.png"
      plot    "cpm_sfgeiger.dat" using 1:2 title 'cpm' with linespoints 
#, \
#            "force.dat" using 1:3 title 'Beam' with points
