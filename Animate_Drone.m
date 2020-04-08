%Setup
global Drone;
Drone.vertices = [0.5   0   0;
                  -0.5  0   0;
                  0     1   0];
Drone.faces = [1 2 3];
Drone.color = [0.6350 0.0780 0.1840];
Drone.rotation = [0 0 0];
Drone.position = [0 0 0];

%Import rotation and translation vectors
trans = zeros(3, 100);
rots = zeros(3, 100);
rots(3, :) = pi/6;

%Animation parameters
nframes = 100;
timestep = 0.05
time = 0;

%Animate
figure();
view(3);
axis([-10 10 -10 10 -10 10]);
xlabel("x");
ylabel("y");
zlabel("z");

for i = 1:nframes
    move(trans(:, i), rots(:, i));
    update;
    time = time + timestep;
    title(sprintf("t = %f", time));
    pause(0.1);
end

function update
    cla;
    curr = getGlobalDrone;
    patch("Faces", curr.faces, "Vertices", curr.vertices, 'FaceColor', 'blue');

end

function move(trans, rots)
    curr = getGlobalDrone;
    %move first
    curr.position = curr.position + trans;
    curr.vertices = curr.vertices + trans;
    %rotate
    curr.rotation = curr.rotation + rots;
    curr.vertices = ([cos(rots(1))*cos(rots(2)) cos(rots(1))*sin(rots(2))*sin(rots(3))-sin(rots(1))*cos(rots(3)) cos(rots(1))*sin(rots(2))*cos(rots(3))+sin(rots(1))*sin(rots(3));
                     sin(rots(1))*cos(rots(2)) sin(rots(1))*sin(rots(2))*sin(rots(3))+cos(rots(1))*cos(rots(3)) sin(rots(1))*sin(rots(2))*cos(rots(3))-cos(rots(1))*sin(rots(3));
                     -sin(rots(2)) cos(rots(2))*sin(rots(3)) cos(rots(2))*cos(rots(3))]*curr.vertices')';
    setGlobalDrone(curr);
end

function setGlobalDrone(val)
global Drone
Drone = val;
end

function r = getGlobalDrone
global Drone
r = Drone;
end