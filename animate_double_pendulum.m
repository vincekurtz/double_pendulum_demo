function [] = animate_double_pendulum(t_sim, q_sim, save_video)
    % Animate the motion of our simple double pendulum model
    %
    % Parameters:
    %   t_sim  : a 1xT*dt vector of timestamps
    %   q_sim  : a 2xT*dt vector of states q = [theta1;theta2;]

    % Check whether the save_vido argument is supplied: default is false
    if ~exist('save_video','var')
        save_video = false;
    end

    % Optional: save video
    if save_video
        dt = t_sim(2) - t_sim(1);
        myVideo = VideoWriter('double_pendulum_animation');
        myVideo.FrameRate = 1/(10*dt);  % we only save every 10th frame
        open(myVideo)
    end

    arm_width = 0.1;
    arm_length = 1;  % our model assumes arms of unit length
    
    % Ground frame
    ground = plot(gca, [-15 15],[0 0],'k','LineWidth',2);
    frame0 = hgtransform(gca);

    % First arm
    frame1 = hgtransform(frame0);
    arm1 = rectangle('Parent', frame1);
    arm1.Position = [-arm_width/2 -arm_width/2 arm_length+arm_width, arm_width];
    arm1.FaceColor = 'red';

    % Second arm
    frame2 = hgtransform(frame1);
    frame2.Matrix = makehgtform('translate',[1 0 0]);
    arm2 = rectangle('Parent', frame2);
    arm2.Position = [-arm_width/2 -arm_width/2 arm_length+arm_width, arm_width];
    arm2.FaceColor = 'red';

    axis equal
    xlim([-2.5,2.5])
    ylim([-2.5,2.5])

    for t = 1:length(t_sim)-1
        tic
        dt = t_sim(t+1) - t_sim(t);

        theta1 = q_sim(t,1);
        theta2 = q_sim(t,2);

        frame1.Matrix = makehgtform('zrotate',theta1);
        frame2.Matrix = makehgtform('translate',[1 0 0],'zrotate',theta2);

        pause(dt-toc)

        if save_video & mod(t,10)==0
            frame = getframe(gcf);
            writeVideo(myVideo, frame);
        end
    end

    if save_video
        close(myVideo)
    end

end

