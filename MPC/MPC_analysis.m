% MPC Data Analysis
%%
for i = 1:length(LOG_log)
cmd_law = LOG_log{i}{1}{1};
state_instant= LOG_log{i}{1}{2};

figure(1)
subplot(4,1,1)
plot(i:i+10,state_instant(1,:),'o-');hold on
subplot(4,1,2)
plot(i:i+10,state_instant(2,:),'o-');hold on
subplot(4,1,3)
plot(i:i+10,state_instant(3,:),'o-');hold on
subplot(4,1,4)
plot(i:i+10,state_instant(4,:),'o-');hold on    
   
end