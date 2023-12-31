n_05 = mean([6.97 , 6.84 , 7.06])
n_10 = mean([14.76 , 14.94 , 15.03])
n_15 = mean([25.42 , 24.39 , 23.18])
n_20 = mean([31.27 , 31.05 , 32.89])
n_25 = mean([37.71 , 37.62 , 38.07])
n_30 = mean([46.75 , 47.97 , 47.07])
n_35 = mean([ 56.79, 56.88  , 56.56 ])
n_40 = mean([ 64.97, 64.75 ,65.11 ])

y = deg2rad([n_05 , n_10 , n_15 , n_20 ,n_25 , n_30 , n_35 , n_40]);
u = [0.05 , 0.10 , 0.15 , 0.20 , 0.25 , 0.30 , 0.35 , 0.40]*100 ;

hold on 
plot(u,y ,'x')
hold on 
[r,m,b] = regression(u,y)

%plot(regression(x,y))