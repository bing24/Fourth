
% barzintest2.m
close all
clear all
clear class

for x=[10 100 1000]
% FigHandle=figure(1);
% set(FigHandle, 'Position', [600, 0, 1600, 800]);
sim=Simulation();

sim.addOperatingRobots(4);
sim.addChargingRobots(2);
sim.setImageScale(.005);
sim.setSimulationTime(200);
travel_length=2*pi;
                number_of_trajectory_steps=100;
                number_of_divergences=15;
                radius=5+rand;
                divergence_range=1;
                divergence_steps=linspace(0,2*pi,number_of_divergences);
                discrete_steps=linspace(0,2*pi,number_of_trajectory_steps);
                random_values=radius+divergence_range*randn([1 number_of_divergences]);
                random_values(end)=random_values(1);
                fitted_values=spline(divergence_steps,[0 random_values 0],discrete_steps);
                x_array=fitted_values.*cos(discrete_steps);
                y_array=fitted_values.*sin(discrete_steps);
                normalization_coef=sum(sqrt(diff(x_array).^2+diff(y_array).^2));
                
                whole_trajectory_x=travel_length/normalization_coef*x_array+0;
                whole_trajectory_y=travel_length/normalization_coef*y_array+0;
sim.list_of_operating_robots(1).trajectory_x=whole_trajectory_x(13:38)
sim.list_of_operating_robots(1).trajectory_y=whole_trajectory_y(13:38);
a=sim.list_of_operating_robots(1);
obj=a;
obj.charging=[];
            position_matrix = [obj.trajectory_x' obj.trajectory_y']*x;
            distance_between_points = diff(position_matrix,1);
            dist_from_vertex_to_vertex = hypot(distance_between_points(:,1), distance_between_points(:,2));
            cumulative_dist_along_path = [0; cumsum(dist_from_vertex_to_vertex,1)];
            dist_steps = 0:obj.max_speed:obj.max_speed*obj.simulation_time; %linspace(0, travel_length, number_of_timesteps)
            new_points = interp1(cumulative_dist_along_path, position_matrix, dist_steps)
            obj.trajectory_x=new_points(:,1)';
            obj.trajectory_y=new_points(:,2)';
            obj.power_level=1-mod([0:obj.simulation_time],obj.battery_life)/obj.battery_life;
            alert=(obj.power_level<obj.alert_level & obj.power_level>=obj.critical_level);
            for i=1:length(find(obj.power_level==1)) % for each charging window that exists
                temp_alert=find(alert);
                temp_alert=temp_alert(find(temp_alert<=obj.battery_life*i & temp_alert>obj.battery_life*(i-1)));
                indeces_to_plot=temp_alert;
                end
figure
                plot(a.trajectory_x,a.trajectory_y)
            end