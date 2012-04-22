Z=meas([1:9 13:15],:);
W=meas(10:12,:);
[state, estimation]=state_estimation_2(Z,W);

%%
%plot(t(1:size(state,2)),state(1:3,:))
subplot(2,1,1);plot(t(1:size(state,2)),meas(1:2,1:size(state,2))-[47+24/60;8+32/60]*ones(1,size(state,2)));title('position');legend('N','E');
subplot(2,1,2);plot(t(1:size(state,2)),state(1:2,:)-[47+24/60;8+32/60]*ones(1,size(state,2)));title('estimation');legend('Nest','Eest');
