%building segments
function[segment_Z,segment_counter,segment_time]=build_segment(startTime, endTime, Z,counter, time);

time1=time(time<endTime);
segment_time=time1(time1>startTime);

segment_Z1=Z(:,time<endTime);
segment_Z=segment_Z1(:,time1>startTime);

counter1=counter(:,time<endTime);
segment_counter=counter1(:,time1>startTime);
