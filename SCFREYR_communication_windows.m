% https://www.mathworks.com/help/aerotbx/ug/satellite-scenario-overview.html

%Scenario
startTime = datetime("yesterday", "TimeZone", "local");
stopTime = startTime + days(4);
sampleTime = 60; %s

sc = satelliteScenario(startTime, stopTime, sampleTime);

%SC-FREYR
semiMajorAxis                   = 6786000;
eccentricity                    = 0.01;
inclination                     = 50;
rightAscensionOfAscendingNode   = 95;
argumentOfPeriapsis             = 93;
trueAnomaly                     = 203;

scfreyr = satellite(sc, ...
                    semiMajorAxis, eccentricity, inclination, rightAscensionOfAscendingNode, argumentOfPeriapsis, trueAnomaly, ...
                    Name="SC-FREYR", ...
                    OrbitPropagator="sgp4");

%SC-FREYR Zethane
zethaneCam = conicalSensor(scfreyr, ...
                        MaxViewAngle=45, ...
                        MountingAngles=[90; 0; 0], ...
                        Name="cam");

%Loyola ground station
lat = 45.45810;
lon = -73.64031;
gs = groundStation(sc, lat, lon, Name="Loyola ground station");

ac = access(scfreyr, gs);

intvls = accessIntervals(ac)
[pos,velocity] = states(scfreyr);

%run sim
show(scfreyr)
coordinateAxes(scfreyr)
groundTrack(scfreyr, LeadTime=3600)
fieldOfView(zethaneCam)
pointAt(scfreyr, 'nadir')
play(sc, PlaybackSpeedMultiplier=200)
