% Dialog zum auswählen der Datei
[filename, path] = uigetfile();
% Pfad und Dateinamen zusammen setzen
file = fullfile(path, filename);
% Datei mit ; getrennt einlesen
data = dlmread(file, ";");

% Erste Zeile ignorieren und die Spalten einzeln speichern
t = data(2:end, 1);
ia = data(2:end,2);
ib = data(2:end,3);
va = data(2:end,4);
vb = data(2:end,5);
hall_angle = data(2:end,6);
vdc = data(2:end,7);

% Einmal plotten bitte
subplot(2,1,1);
plot(t,va,t,vb);
legend("i_{alpha}","i_{beta}");
subplot(2,1,2);
plot(t,ia,t,ib);
legend("v_{alpha}","v_{beta}");

