% Dialog zum auswählen der Datei
[filename, path] = uigetfile();
% Pfad und Dateinamen zusammen setzen
file = fullfile(path, filename);
% Datei mit ; getrennt einlesen
data = dlmread(file, ";");

stop = 150;

% Erste Zeile ignorieren und die Spalten einzeln speichern
t = data(2:stop, 1);
ia = data(2:stop,2);
ib = data(2:stop,3);
va = data(2:stop,4);
vb = data(2:stop,5);
hall_angle = data(2:stop,6)*pi/127;
vdc = data(2:stop,7);


function error = observer(x)
stop = 150;
r = x(1);
l = x(2);
% Back EMF, Integrator
fa = zeros(stop-1,1);
fb = zeros(stop-1,1);
for i = 1:stop-2
fa(i + 1) = fa(i) + (va(i) - r*ia(i)); 
fb(i + 1) = fb(i) + (vb(i) - r*ib(i));
endfor
fba = fa - (max(fa) + min(fa))/2;
fbb = fb - (max(fb) + min(fb))/2;
fa = fba - l*ia;
fb = fbb - l*ib;

phi = atan2(fa,fb);

error = mean(sin(hall_angle) .* cos(phi) + cos(hall_angle) .* sin(phi));
end 

fsolve(@observer, [0.1,1e-4]);


