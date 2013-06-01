Ps(T) = a*exp(b*T/(c+T))
a = 6.112
b = 17.67
c = 243.5
Pa(T,RH) = RH*Ps(T)/100.0
Tdp(T,RH) = c*log(Pa(T,RH)/a)/(b-log(Pa(T,RH)/a))

# plots data output by sfgeiger
# file is cpm_sfgeiger.dat
      set   autoscale                        # scale axes automatically
      unset log                              # remove any log-scaling
      unset label                            # remove any previous labels
      set xtic auto                          # set xtics automatically
      set ytic auto                          # set ytics automatically
      set title "Weather Station"
      set xlabel "temp"
      set ylabel ""
      set xdata time
      set timefmt "%s"

set style line 1 lc rgb "#5F9EA0" lt 1
set style line 2 lc rgb "#5F9EA0" lt 1
set style line 3 lc rgb "#5F9EA0" lt 1
set style line 4 lc rgb "#5F9EA0" lt 1
set style line 5 lc rgb "#D2691E" lt 1
set style line 6 lc rgb "#5F9EA0" lt 1

#      set key 0.01,100
#      set label "Yield Point" at 0.003,260
#      set arrow from 0.0028,250 to 0.003,280
#      set xr [0.0:0.022]
#      set yr [0:35]
      #set terminal png
      #set output "station_weather.png"
      plot   "station_weather.dat" using 3:4 title 'temp1' with lines,\
             "station_weather.dat" using 3:($6*0.295299) title 'pressure' with lines, \
             "station_weather.dat" using 3:8 title 'temp2' with lines, \
             "station_weather.dat" using 3:(Tdp($4,$10)) title 'Dew point' with lines, \
             "station_weather.dat" using 3:10 title 'RH' with lines ls 5, \
             "station_weather.dat" using 3:12 title 'temp3' with lines ls 6 
