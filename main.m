clc,clear;
% close all;
% angle grid
% theta=-pi/2:pi/100:pi/2;
theta=linspace(-pi/2,pi/2,100);
%desired beampattern
p_d=ones(1,length(theta))*0.1;
p_d(length(theta)/2-2:length(theta)/2+2)=1;
N=10;%antenna number
P=1;% transmit power
A=exp(1i*2*(0:N-1)'*sin(theta));%steering vector set

%cvx solver
cvx_begin quiet
variable C(N,N) complex semidefinite hermitian
variable a nonnegative
sum=0;%MSE
for ii=1:length(theta)
    sum = sum + square_pos(abs(a*p_d(ii)-A(:,ii)'*C*A(:,ii)));
end
minimize (sum/N)
subject to
%     diag(C)==P/N*ones(N,1);% power per antenna
    real(trace(C))<=P;% power per antenna
cvx_end
%%
%optimized beampattern
p = zeros(1,length(theta));
for ii=1:length(theta)
    
    p(ii)=real(A(:,ii)'*C*A(:,ii))/a;
end
figure
hold on
plot(theta*180/pi,db(p_d),'b-','linewidth',1.5);
plot(theta*180/pi,db(p),'r-.','linewidth',1.5);
box on
grid on
legend('Desired beampattern','Optimized beampattern')
xlabel('Angle (degree)');
ylabel('Beampattern (dB)');
save('beampattern','p')

figure
hold on
plot(theta*180/pi,p_d,'b-','linewidth',1.5);
plot(theta*180/pi,p,'r-.','linewidth',1.5);
box on
grid on
legend('Desired beampattern','Optimized beampattern')
xlabel('Angle (degree)');
ylabel('Beampattern');
save('beampattern','p')