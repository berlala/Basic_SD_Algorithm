% 
figure(4)
x_state  =  X_log(1,:);
y_state  = X_log(2,:);
v_state = U_log(3,:);

 for i=1:length(v_state) 
     plot(x_state(i),y_state(i),'.','MarkerSize',15,'color',[v_state(i)/max(v_state) 0.3 v_state(i)/max(v_state)]);
     hold on; 
 end
