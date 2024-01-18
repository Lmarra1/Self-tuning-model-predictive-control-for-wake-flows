%% Part of the code to provide online plot of the MPC results



colr   = [0.4940 0.1840 0.5560;
    0.4660 0.6740 0.1880;
    0.3010 0.7450 0.9330];     % Colors for plots

if any(MPCPlotVerbose == [1 2])
    StartIndexPlot = round(Ts/dt)*(j-1)+1;
else
    StartIndexPlot = 1 ;
end
EndIndexPlot = Ts_dt*j + 1;

% change limits to plot
hFig = figure(10);
if T == Ts || T == Duration
    hFig.Units = 'centimeters';
    hFig.Position = [5, 3, 14, 15];
    PLOT1 = tiledlayout(3,1);
end

% Cd plot
nexttile(1); hold on
p(1)=plot(t_History( StartIndexPlot:EndIndexPlot),  cstar_History(1,StartIndexPlot:EndIndexPlot),"r.-");
p(2)=plot(t_History( StartIndexPlot:EndIndexPlot),  a_History(1,StartIndexPlot:EndIndexPlot),"k-");
if AddNoise
    p(3)=plot(t_History( StartIndexPlot:EndIndexPlot), c_History(1,StartIndexPlot:round(Ts/dt)*j + 1),"linewidth",1.5,"Color",[0.9290 0.6940 0.1250]);
end
if UseLPR && T>=StartLPR
    p(4)=plot(t_History( StartIndexPlot:EndIndexPlot), c_LPR_History(1,StartIndexPlot:EndIndexPlot),"b-");
end
ylabel("$C_d$")
set(gca,'xticklabel',"");
xlim([0 t_History(EndIndexPlot)]);
ylim([min(Cd_ref) - 0.1 max(max(Cd_ref) + 0.1,4)])

if T == Ts || T == Duration
    fill([0 StartControl StartControl 0], [-100 -100 100 100], [0.4660 0.6740 0.1880], "faceAlpha",0.2,"EdgeColor","none");
end
xline(StartControl,"k--","linewidth",1.5)

if AddNoise
    if T>=StartLPR && UseLPR
        leg = legend(p,["Reference","Ideal","Noisy","LPR"],"Location","southwest","NumColumns",1);
        legend boxoff
    else
        leg = legend("Reference","Ideal","Noisy","Location","southwest","NumColumns",1);
        legend boxoff
    end
else
    leg = legend("Reference","Ideal","Location","southwest","NumColumns",1);
    legend boxoff
end
hold off



% Cl plot
nexttile(2); hold on
plot(t_History( StartIndexPlot:EndIndexPlot), a_History(2,StartIndexPlot:EndIndexPlot),"k-");
plot(t_History( StartIndexPlot:EndIndexPlot), cstar_History(2,StartIndexPlot:EndIndexPlot),"r.-");
if AddNoise
    plot(t_History( StartIndexPlot:EndIndexPlot), c_History(2,StartIndexPlot:EndIndexPlot),"linewidth",1.5,"Color",[0.9290 0.6940 0.1250]);
end
if UseLPR && T>=StartLPR
    plot(t_History( StartIndexPlot:EndIndexPlot), c_LPR_History(2,StartIndexPlot:EndIndexPlot),"b-");
end
ylabel("$C_l$")
set(gca,'xticklabel',"");
xlim([0 t_History(EndIndexPlot)]); ylim([-2 2]);
if T == Ts || T == Duration
    fill([0 StartControl StartControl 0], [-100 -100 100 100], [0.4660 0.6740 0.1880], "faceAlpha",0.2,"EdgeColor","none");
end
xline(StartControl,"k--","linewidth",1.5)
ylim([min(min(Cl_ref) - 0.1,-1) max(max(Cl_ref) + 0.1, 1)])
hold off


nexttile(3); hold on
for bi = 1:3
    bline(bi) = plot(t_History(  StartIndexPlot:EndIndexPlot), b_History(bi,StartIndexPlot:EndIndexPlot),"Color",colr(bi,:));
end
if t_History(StartIndexPlot)>=StartControl
    fill([t_History([StartIndexPlot, EndIndexPlot]), flip(t_History([StartIndexPlot, EndIndexPlot]))], UB(1) + [0 0 10 10], [0.6350 0.0780 0.1840],"faceAlpha",0.2,"EdgeColor","none");
    fill([t_History([StartIndexPlot, EndIndexPlot]), flip(t_History([StartIndexPlot, EndIndexPlot]))], LB(1) + [0 0 -10 -10], [0.6350 0.0780 0.1840],"faceAlpha",0.2,"EdgeColor","none");
end
if T == Ts || T == Duration
    fill([0 StartControl StartControl 0], [-100 -100 100 100], [0.4660 0.6740 0.1880], "faceAlpha",0.2,"EdgeColor","none");
end
ylabel("$b_i$")
xlabel("$t [c.u.]$")
xlim([0 t_History(EndIndexPlot)]); ylim([LB(1) UB(1)] + (UB(1) - LB(1))/5 * [-1 1])
xline(StartControl,"k--","linewidth",1.5)
legend(bline, ["$b_1$","$b_2$","$b_3$"],"Location","northwest","NumColumns",2);
legend boxoff
hold off

title(PLOT1,"Real time control results","interpreter","latex")
PLOT1.TileSpacing = "compact";
PLOT1.Padding = "compact";





%%
if MPCPlotVerbose == 2
    hFig = figure(11);
    if T == Ts || T == Duration
        hFig.Units = 'centimeters';
        hFig.Position = [20, 6.5, 10, 8];
        PLOT2 = tiledlayout(1,2);
    end

    nexttile(1)
    theta = 2*pi*(0:100)/100; % Define circumference

    xCyl1 = -1.299 + 0.5 * cos(theta);
    yCyl1 = 0.5 * sin(theta);
    xCyl = 0.5 * cos(theta);
    yCyl2 = 0.75 + 0.5 * sin(theta);
    yCyl3 = -0.75 + 0.5 * sin(theta);

    fill(xCyl1, yCyl1, 'k', 'LineWidth', 2);hold on;
    fill(xCyl , yCyl2 , 'k', 'LineWidth', 2);
    fill(xCyl , yCyl3 , 'k', 'LineWidth', 2);

%     plot([])

    axis equal off

    arrowLength1 = (b_History(1, EndIndexPlot) - LB(1))/(UB(1) - LB(1))*300 +30;
    arrowLength2 = (b_History(2, EndIndexPlot) - LB(1))/(UB(1) - LB(1))*300 +30;
    arrowLength3 = (b_History(3, EndIndexPlot) - LB(1))/(UB(1) - LB(1))*300 +30;

    % Calculate angles for circular arrows
    R = 0.7;

    angles = [linspace(180, arrowLength1, 50);...
        linspace(180, arrowLength2, 50);...
        linspace(180, arrowLength3, 50)];

    arrowX(1,:) = -1.299 + R.*cosd(angles(1,:));
    arrowY(1,:) = R.*sind(angles(1,:));

    arrowX(2,:) = R.*cosd(angles(2,:));
    arrowY(2,:) = 0.75 + R.*sind(angles(2,:));

    arrowX(3,:) = R.*cosd(angles(3,:));
    arrowY(3,:) = -0.75 + R.*sind(angles(3,:));

    % Plot the cylinders
    fill(xCyl1, yCyl1, 'k', 'LineWidth', 2);
    fill(xCyl, yCyl2, 'k', 'LineWidth', 2);
    fill(xCyl, yCyl3, 'k', 'LineWidth', 2);



    % Plot circular arrows
    for ip = 1:3
        plot(arrowX(ip,:), arrowY(ip,:), 'r', 'LineWidth', 2);
    end


    for ip = 1:3
        if round(angles(ip,end)) ~= 180
            plot(arrowX(ip,end), arrowY(ip,end), 'r>', 'markersize', 5);
        end
    end

    plot(-1.299 + [0 -0.7], [0 0], "k-", "linewidth", 1.5)
    plot(-1.299 + [0 +0.7*cosd(30)], [0 0.7*sind(30)], "k-", "linewidth", 1.5)
    plot(-1.299 + [0 +0.7*cosd(30)], [0 -0.7*sind(30)], "k-", "linewidth", 1.5)

    plot([0 -0.7], 3/4 + [0 0], "k-", "linewidth", 1.5)
    plot([0 +0.7*cosd(30)], 3/4 + [0 0.7*sind(30)], "k-", "linewidth", 1.5)
    plot([0 +0.7*cosd(30)], 3/4 + [0 -0.7*sind(30)], "k-", "linewidth", 1.5)

    plot([0 -0.7], -3/4 + [0 0], "k-", "linewidth", 1.5)
    plot([0 +0.7*cosd(30)], -3/4 + [0 0.7*sind(30)], "k-", "linewidth", 1.5)
    plot([0 +0.7*cosd(30)], -3/4 + [0 -0.7*sind(30)], "k-", "linewidth", 1.5)

    hold off




    nexttile(2)
    if exist("tx")
        delete(tx)
    end
    axis off
    xlim([0 10])
    ylim([-5 5])
    tx(1) = text(0, 0, "$b_1 = " + string(round(b_History(1, EndIndexPlot),2)) + "$");
    tx(2) = text(3, 3, "$b_2 = " + string(round(b_History(2, EndIndexPlot),2)) + "$");
    tx(3) = text(3, -3, "$b_3 = " + string(round(b_History(3, EndIndexPlot),2)) + "$");
    hold off


    title(PLOT2,"Control actuation","interpreter","latex")
    PLOT2.TileSpacing = "compact";
    PLOT2.Padding = "compact";

end


% Refresh the plot before proceeding
drawnow






