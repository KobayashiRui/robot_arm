robot = importrobot('my_arm_2.urdf');
robot.DataFormat = 'row'; %データ形式を配列に変更

%計画問題
q0 = homeConfiguration(robot); %初期姿勢のデータ

gik = robotics.GeneralizedInverseKinematics('RigidBodyTree',robot,'ConstraintInputs',{'position'});

%制約オブジェクト
positionTgt = robotics.PositionTarget('end_link');
qWaypoints =zeros(73, 3);
%qWaypointsに新しいデータを保存する=> gikによって求めた指定位置での関節角度
 for k = 2:73
k
x = -0.05*sind((k-2)*5);
z = 0.15 + 0.05*cosd((k-2)*5);
y = 0.15;
positionTgt.TargetPosition = [x,y,z];
[qWaypoints(int32(k),:),solutionInfo] = gik(qWaypoints(int32(k-1),:),positionTgt);
 end
%生成された軌道の可視化 中間点を内挿して滑らかな軌道を生成
framerate = 15; %Hzの設定
r = robotics.Rate(framerate); %15Hzで軌道する
tFinal = 10;
tWaypoints = [0,linspace(tFinal/2,tFinal,size(qWaypoints,1)-1)];
%qWaypointsが与えられる時点を指定する
%今回の場合=>スタートは0, 次は5sから開始し、終了は10sとなっている =>実時間に対応を確認
%5と10の間を等間隔で埋めるためlinspaceを使用している

numFrames = tFinal * framerate;
%1sに15回のフレームがあるため=> 総フレーム数=秒数*Hz 
qInterp = pchip(tWaypoints,qWaypoints',linspace(0,tFinal,numFrames))';
%tWaypoints: pchipのx座標=>yの値が与えらえる時点を表す
%qWaypoints': pchipのy座標(xの長さとそろえるため転置している)=>今回は各関節の角度
%linspace(0,tFinal,numFrames) 
%: 0~10の範囲でnumFramesの数の要素を等間隔で作成する この値はpchipのx座標となる

end_link_Position = zeros(numFrames, 3);
%各フレームでのend_linkのポジションを求める
for k = 1:numFrames
    end_link_Position(k,:) = tform2trvec(getTransform(robot,qInterp(k,:),'end_link'));
    %getTransform : robotにqInterp(k,:)の角度を与えたときのend_linkの絶対座標の変換を得る
    %tform2trvec : 4*4のデータを変換して位置の情報に変える
    %=> ベース座標からend_linkの変換データから位置データの3列のベクトルにする
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