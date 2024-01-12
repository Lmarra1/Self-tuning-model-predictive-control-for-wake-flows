%% Part of the code to provide online plot of the MPC results
colr   = ["k","r","b"];                       % Colors for plots 

hFig = figure(10);
hFig.Position = [92.2000  169.0000  344.8000  520.0000];

subplot 311
hold on
plot(t_History(  Ts/dt*(j-1)+1:Ts/dt*j + 1), a_History(1,Ts/dt*(j-1)+1:Ts/dt*j + 1),"k-");
plot(t_History(  Ts/dt*(j-1)+1:Ts/dt*j + 1), cstar_History(1,Ts/dt*(j-1)+1:Ts/dt*j + 1),"r-");

if AddNoise
    plot(t_History(  Ts/dt*(j-1)+1:Ts/dt*j + 1), c_History(1,Ts/dt*(j-1)+1:Ts/dt*j + 1),"linewidth",1.5,"Color",[0.4660 0.6740 0.1880 0.3]);
end

if UseLPR && T>=StartLPR
    plot(t_History(  Ts/dt*(j-1)+1:Ts/dt*j + 1), c_LPR_History(1,Ts/dt*(j-1)+1:Ts/dt*j + 1),"b--");
end

switch Case
    case "Tuning"
        title("Current BO iteration")
    case "Control"
        title("Real time control results")
end


ylim([1 4])
hold off
ylabel("$C_d$")
set(gca,'xticklabel',"");


subplot 312
hold on
plot(t_History(  Ts/dt*(j-1)+1:Ts/dt*j + 1), a_History(2,Ts/dt*(j-1)+1:Ts/dt*j + 1),"k-");
plot(t_History(  Ts/dt*(j-1)+1:Ts/dt*j + 1), cstar_History(2,Ts/dt*(j-1)+1:Ts/dt*j + 1),"r-");
if AddNoise
    plot(t_History(  Ts/dt*(j-1)+1:Ts/dt*j + 1), c_History(2,Ts/dt*(j-1)+1:Ts/dt*j + 1),"linewidth",1.5,"Color",[0.4660 0.6740 0.1880 0.3]);
end

if UseLPR && T>=StartLPR
    plot(t_History(  Ts/dt*(j-1)+1:Ts/dt*j + 1), c_LPR_History(2,Ts/dt*(j-1)+1:Ts/dt*j + 1),"b--");
end

ylim([-2 2])
hold off
ylabel("$C_l$")
set(gca,'xticklabel',"");


subplot 313
hold on
for bi = 1:3
    bline(bi) = plot(t_History(  Ts/dt*(j-1)+1:Ts/dt*j + 1), b_History(bi,Ts/dt*(j-1)+1:Ts/dt*j + 1),"Color",colr(bi));
end
ylim([-1 1])
ylabel("$b_i$")
xlabel("$t$")
legend(bline, ["$b_1$","$b_2$","$b_3$"],"Location","northwest","NumColumns",3);
legend boxoff
hold off


drawnow
