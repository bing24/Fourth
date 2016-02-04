% barzintest2.m
close all
clear all
clear class


% FigHandle=figure(1);
% set(FigHandle, 'Position', [600, 0, 1600, 800]);
sim=Simulation();

sim.addOperatingRobots(4);
sim.addChargingRobots(2);
sim.setImageScale(.01);
sim.setSimulationTime(400); % switch to 200 for circle scenario



temp_var=2;
% sim.list_of_operating_robots(1).setTrajectoryBarzinEdition(-temp_var*2,0,'center');
% sim.list_of_operating_robots(2).setTrajectoryBarzinEdition(temp_var*2,0,'center');
% sim.list_of_operating_robots(3).setTrajectoryBarzinEdition(-temp_var*2,-temp_var*3,'center');
% sim.list_of_operating_robots(4).setTrajectoryBarzinEdition(temp_var*2,-temp_var*3,'center');
% lawn_mower=[-1*temp_var 1*temp_var 1*temp_var -1*temp_var -1*temp_var 1*temp_var 1*temp_var -1*temp_var -1*temp_var 1*temp_var 1*temp_var -1*temp_var;
% 			temp_var*1.5/2.5 temp_var*1.5/2.5 2*temp_var*1.5/2.5 2*temp_var*1.5/2.5 3*temp_var*1.5/2.5 3*temp_var*1.5/2.5 4*temp_var*1.5/2.5 4*temp_var*1.5/2.5 5*temp_var*1.5/2.5 5*temp_var*1.5/2.5 6*temp_var*1.5/2.5 6*temp_var*1.5/2.5];



count=1;
for i=1:2:40
	if count==1
lawn_mower(1,i)=temp_var+.5;
lawn_mower(1,i+1)=-temp_var+.5;
count=0;
else
	lawn_mower(1,i)=-temp_var+.5;
lawn_mower(1,i+1)=temp_var+.5;
count=1;
end

lawn_mower(2,i)=temp_var/2*i;
lawn_mower(2,i+1)=temp_var/2*i;
end
lawn_mower=lawn_mower;
sim.list_of_operating_robots(1).trajectory_x=lawn_mower(2,:);
sim.list_of_operating_robots(2).trajectory_x=lawn_mower(2,:);
sim.list_of_operating_robots(3).trajectory_x=lawn_mower(2,:);
sim.list_of_operating_robots(4).trajectory_x=lawn_mower(2,:);
sim.list_of_operating_robots(1).trajectory_y=lawn_mower(1,:);
sim.list_of_operating_robots(2).trajectory_y=lawn_mower(1,:)+5;
sim.list_of_operating_robots(3).trajectory_y=lawn_mower(1,:)+10;
sim.list_of_operating_robots(4).trajectory_y=lawn_mower(1,:)+15;

% Uncomment below to switch to second scenario (circle)

% 				travel_length=2*pi;
%                 number_of_trajectory_steps=100;
%                 number_of_divergences=15;
%                 radius=5+rand;
%                 divergence_range=1;
%                 divergence_steps=linspace(0,2*pi,number_of_divergences);
%                 discrete_steps=linspace(0,2*pi,number_of_trajectory_steps);
%                 random_values=radius+divergence_range*randn([1 number_of_divergences]);
%                 random_values(end)=random_values(1);
%                 fitted_values=spline(divergence_steps,[0 random_values 0],discrete_steps);
%                 x_array=fitted_values.*cos(discrete_steps);
%                 y_array=fitted_values.*sin(discrete_steps);
%                 normalization_coef=sum(sqrt(diff(x_array).^2+diff(y_array).^2));
                
%                 whole_trajectory_x=travel_length/normalization_coef*x_array*100+0;
%                 whole_trajectory_y=travel_length/normalization_coef*y_array*100+0;

% sim.list_of_operating_robots(1).trajectory_x=whole_trajectory_x(13:end-1);
% sim.list_of_operating_robots(2).trajectory_x=[whole_trajectory_x(39:end-1) whole_trajectory_x(1:38)];
% sim.list_of_operating_robots(3).trajectory_x=[whole_trajectory_x(64:end-1) whole_trajectory_x(1:63)];
% sim.list_of_operating_robots(4).trajectory_x=[whole_trajectory_x(89:end-1) whole_trajectory_x(1:88)];
% sim.list_of_operating_robots(1).trajectory_y=whole_trajectory_y(13:end-1) ;
% sim.list_of_operating_robots(2).trajectory_y=[whole_trajectory_y(39:end-1) whole_trajectory_y(1:38)];
% sim.list_of_operating_robots(3).trajectory_y=[whole_trajectory_y(64:end-1) whole_trajectory_y(1:63)];
% sim.list_of_operating_robots(4).trajectory_y=[whole_trajectory_y(89:end-1) whole_trajectory_y(1:88)];


% close all
%% Set initial positions and speed of charging robots using ChargingRobot.m 

% for i=1:length(sim.list_of_charging_robots)
% 	setpos(sim.list_of_charging_robots(i),i*2,i*4);
% 	setspeed(sim.list_of_charging_robots(i),10000);
% end
setpos(sim.list_of_charging_robots(1),0,3);
      setspeed(sim.list_of_charging_robots(1),100);
setpos(sim.list_of_charging_robots(2),0,13);
      setspeed(sim.list_of_charging_robots(2),100);
% setpos(sim.list_of_charging_robots(1),50+50,0);
% 	setspeed(sim.list_of_charging_robots(1),2);
% setpos(sim.list_of_charging_robots(2),-60-50,0);
% 	setspeed(sim.list_of_charging_robots(2),2);


setTimeStep(sim,1);
setChargingTime(sim,5); %switch to 15 for circle
a=sim.list_of_operating_robots(1);

plan(sim,'LKH','Distance');


sim.plot()
set(gca,'XTick',[])
set(gca,'YTick',[])
 % pause (3)
 % sim.simulate()