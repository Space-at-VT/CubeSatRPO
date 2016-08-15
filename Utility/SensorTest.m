%Sensor Setup for three rangefinders with diffused lasers

%Centerline: x-axis
center = [1;0;0];

rotateY = R2(2);
rotateX = R1(120);

sensor1 = rotateY*center;
sensor2 = rotateX*sensor1;
sensor3 = rotateX*sensor2;
 
% figure(1),clf;
% hold on;
% plot3([0 sensor1(1)],[0 sensor1(2)],[0 sensor1(3)]);
% plot3([0 sensor2(1)],[0 sensor2(2)],[0 sensor2(3)]);
% plot3([0 sensor3(1)],[0 sensor3(2)],[0 sensor3(3)]);

target = R2(13)*R3(15)*R1(4)*[200;0;0];


FOV = 90;

targetUnit = target/norm(target);



while FOV > 4
    angle1 = acosd(dot(sensor1, targetUnit));
    angle2 = acosd(dot(sensor2, targetUnit));
    angle3 = acosd(dot(sensor3, targetUnit));
    
    canSee1 = angle1 < FOV/2;
    canSee2 = angle2 < FOV/2;
    canSee3 = angle3 < FOV/2;
    
    theta = FOV/2;
    
    if FOV<10
        theta = 1;
    end
    
    
    FOV
    angle1
    angle2
    angle3
    errorAngle = acosd(dot(center, targetUnit))
        
    if canSee1 && canSee2 && canSee3
        FOV = FOV - 1;
        continue
    end
    
    if ~canSee1 && ~canSee2 && ~canSee3
        disp('error!');
        break;
    end
    
    if ~canSee1
        rotateAxis = cross(sensor1, center);
        rotateAxis = rotateAxis/norm(rotateAxis);
        Rmatrix = cosd(theta)*eye(3)+sind(theta)*skew(rotateAxis) + (1-cosd(theta))*(rotateAxis*rotateAxis');
        sensor1 = Rmatrix*sensor1;
        sensor2 = Rmatrix*sensor2;
        sensor3 = Rmatrix*sensor3;
        center = Rmatrix*center;
        continue;
    end
    
    if ~canSee2
        rotateAxis = cross(sensor2, center);
        rotateAxis = rotateAxis/norm(rotateAxis);
        Rmatrix = cosd(theta)*eye(3)+sind(theta)*skew(rotateAxis) + (1-cosd(theta))*(rotateAxis*rotateAxis');
        sensor1 = Rmatrix*sensor1;
        sensor2 = Rmatrix*sensor2;
        sensor3 = Rmatrix*sensor3;
        center = Rmatrix*center;
        continue;
    end
    
    if ~canSee3
        rotateAxis = cross(sensor3, center);
        rotateAxis = rotateAxis/norm(rotateAxis);
        Rmatrix = cosd(theta)*eye(3)+sind(theta)*skew(rotateAxis) + (1-cosd(theta))*(rotateAxis*rotateAxis');
        sensor1 = Rmatrix*sensor1;
        sensor2 = Rmatrix*sensor2;
        sensor3 = Rmatrix*sensor3;
        center = Rmatrix*center;
        continue;
    end
    

end

errorAngle = acosd(dot(center, targetUnit))
