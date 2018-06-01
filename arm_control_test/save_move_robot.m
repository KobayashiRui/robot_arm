hold off
 show(robot,q0,'PreservePlot',false,'Frames','off');
 hold on
 p = plot3(end_link_Position(1,1), end_link_Position(1,2), end_link_Position(1,3));
 hold on
 for k = 1:size(qInterp,1)
     show(robot, qInterp(k,:), 'PreservePlot', false,'Frames','off');
     p.XData(k) = end_link_Position(k,1);
     p.YData(k) = end_link_Position(k,2);
     p.ZData(k) = end_link_Position(k,3);
     M(k) = getframe;
     waitfor(r);
 end
 myvideo = VideoWriter('arm_circle.avi');
 myvideo.Quality = 100;
 myvideo.FrameRate = 15;
 open(myvideo)
 writeVideo(myvideo,M);
 close(myvideo);