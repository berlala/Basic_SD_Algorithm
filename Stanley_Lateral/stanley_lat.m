function steercmd = stanley_lat(RefPos,CurPos,curvelocity)
% RefPos: reference path (x,y,heading[rad])
% CurPos: current position(x,y,heading[rad])
% curvelocity: velocity[m/s]
% steercmd: [rad]

Ksoft = 1;
gain=20; % gain factor for lateral error to steering 
wheelbase=2.7;

%to the interval [0 2*pi] such that zero maps to
%zero and 2*pi maps to 2*pi
twoPiRefPos = cast(2*pi, 'like', RefPos(3));
twoPiCurPos = cast(2*pi, 'like', CurPos(3));

positiveInputRefPos = (RefPos(3) > 0);
positiveInputCurPos = (CurPos(3) > 0);

thetaRef = mod(RefPos(3), twoPiRefPos);
thetaCur = mod(CurPos(3), twoPiRefPos);

positiveInputRefPos = ((thetaRef == 0) & positiveInputRefPos);
thetaRef = thetaRef + twoPiRefPos*positiveInputRefPos;

positiveInputCurPos = ((thetaCur == 0) & positiveInputCurPos);
thetaCur = thetaCur + twoPiCurPos*positiveInputCurPos;

RefPos(3)=thetaRef ;
CurPos(3)=thetaCur;

%This function ensures that angError is in the range [-pi,pi).

angError =CurPos(3)-RefPos(3);

piVal = cast(pi, 'like', angError);

twoPi = cast(2*pi, 'like', angError);

positiveInput = (angError+piVal)> 0;

theta = mod((angError+piVal), twoPi);

positiveInput = ((theta == 0) & positiveInput);
theta = theta + twoPi*positiveInput;

theta=theta-piVal;
angError=theta;

%rearPoseToFrontPose Transform pose from rear wheel to front wheel
tHat = [cos(RefPos(3)), sin(RefPos(3))];
CurPos(:, 1) = CurPos(:, 1) + wheelbase * cos(CurPos(3));
CurPos(:, 2) = CurPos(:, 2) + wheelbase * sin(CurPos(3));

d = CurPos(1:2) - RefPos(1:2);

% Tracking error vector
posError = -(d(1)*tHat(2) - d(2)*tHat(1));  %lateral Error / position error

delta = -(angError + atan(gain * posError/(Ksoft+curvelocity)));

%delta=delta*180/pi;

delta = sign(delta) * min(abs(delta), 35/180*pi);

steercmd = delta;
