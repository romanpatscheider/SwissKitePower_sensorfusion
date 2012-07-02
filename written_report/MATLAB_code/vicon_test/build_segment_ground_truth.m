function[segment_Z,segment_time]=build_segment_ground_truth(startTime, endTime, pos_vel, time);

time1=time(time<endTime);
segment_time=time1(time1>startTime);

segment_Z1=pos_vel(:,time<endTime);
segment_Z=segment_Z1(:,time1>startTime);

