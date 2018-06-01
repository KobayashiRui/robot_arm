robot = importrobot('my_arm_2.urdf');
robot.DataFormat = 'row'; %�f�[�^�`����z��ɕύX

%�v����
q0 = homeConfiguration(robot); %�����p���̃f�[�^

gik = robotics.GeneralizedInverseKinematics('RigidBodyTree',robot,'ConstraintInputs',{'position'});

%����I�u�W�F�N�g
positionTgt = robotics.PositionTarget('end_link');
qWaypoints =zeros(73, 3);
%qWaypoints�ɐV�����f�[�^��ۑ�����=> gik�ɂ���ċ��߂��w��ʒu�ł̊֐ߊp�x
 for k = 2:73
k
x = -0.05*sind((k-2)*5);
z = 0.15 + 0.05*cosd((k-2)*5);
y = 0.15;
positionTgt.TargetPosition = [x,y,z];
[qWaypoints(int32(k),:),solutionInfo] = gik(qWaypoints(int32(k-1),:),positionTgt);
 end
%�������ꂽ�O���̉��� ���ԓ_����}���Ċ��炩�ȋO���𐶐�
framerate = 15; %Hz�̐ݒ�
r = robotics.Rate(framerate); %15Hz�ŋO������
tFinal = 10;
tWaypoints = [0,linspace(tFinal/2,tFinal,size(qWaypoints,1)-1)];
%qWaypoints���^�����鎞�_���w�肷��
%����̏ꍇ=>�X�^�[�g��0, ����5s����J�n���A�I����10s�ƂȂ��Ă��� =>�����ԂɑΉ����m�F
%5��10�̊Ԃ𓙊Ԋu�Ŗ��߂邽��linspace���g�p���Ă���

numFrames = tFinal * framerate;
%1s��15��̃t���[�������邽��=> ���t���[����=�b��*Hz 
qInterp = pchip(tWaypoints,qWaypoints',linspace(0,tFinal,numFrames))';
%tWaypoints: pchip��x���W=>y�̒l���^���炦�鎞�_��\��
%qWaypoints': pchip��y���W(x�̒����Ƃ��낦�邽�ߓ]�u���Ă���)=>����͊e�֐߂̊p�x
%linspace(0,tFinal,numFrames) 
%: 0~10�͈̔͂�numFrames�̐��̗v�f�𓙊Ԋu�ō쐬���� ���̒l��pchip��x���W�ƂȂ�

end_link_Position = zeros(numFrames, 3);
%�e�t���[���ł�end_link�̃|�W�V���������߂�
for k = 1:numFrames
    end_link_Position(k,:) = tform2trvec(getTransform(robot,qInterp(k,:),'end_link'));
    %getTransform : robot��qInterp(k,:)�̊p�x��^�����Ƃ���end_link�̐�΍��W�̕ϊ��𓾂�
    %tform2trvec : 4*4�̃f�[�^��ϊ����Ĉʒu�̏��ɕς���
    %=> �x�[�X���W����end_link�̕ϊ��f�[�^����ʒu�f�[�^��3��̃x�N�g���ɂ���
end

figure;
show(robot, q0, 'PreservePlot', false);
hold on
p = plot3(end_link_Position(1,1), end_link_Position(1,2), end_link_Position(1,3));
hold on
for k = 1:size(qInterp,1)
    show(robot, qInterp(k,:), 'PreservePlot', false);
    p.XData(k) = end_link_Position(k,1);
    p.YData(k) = end_link_Position(k,2);
    p.ZData(k) = end_link_Position(k,3);
    waitfor(r);
end
hold off