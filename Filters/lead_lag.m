% Lead Lag Part
%%
clear;
%% a)
f_1 = 5;
f_2 =9;
HC = tf([1/(2*pi*f_1) 1],[1/(2*pi*f_2) 1]);% Choose P and D 
om=logspace(-1,2,300);
[mag,phase,wout]=bode (HC,om);

Magdb=20*log10(mag);
a(1,:)=Magdb(1,1,:);
b(1,:)=phase(1,1,:);

hz1=wout./(2*pi);

figure
subplot(2,1,1)
semilogx (hz1,a);
ylabel ('Magnitude[dB]');
xlabel ('Frequency[Hz]');
grid on;
subplot(2,1,2)
semilogx (hz1,b);
ylabel ('Phase[deg]');
xlabel ('Frequency[Hz]');
grid on;

phase_max = sqrt(f_1*f_2)
%%
figure(2)
margin(HC)