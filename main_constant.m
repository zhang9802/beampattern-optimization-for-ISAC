clc,clear;
close all;
P = load('beampattern.mat');
p_d = P.p;
% angle grid
% theta=-pi/2:pi/100:pi/2;
theta=linspace(-pi/2,pi/2,100);
%desired beampattern
% p_d=ones(1,length(theta))*0.1;
% p_d(length(theta)/2-2:length(theta)/2+2)=1;
N=10;%antenna number
P=1;% transmit power
A=exp(1i*2*(0:N-1)'*sin(theta));%steering vector set
iter_num=10;
C1 = zeros(N,N,iter_num+1);
C0= exp(1i*2*pi*rand(N,N));
rho = 0.1;
delta=1e-2;
for iter=1:iter_num
%cvx solver
cvx_begin quiet
variable C(N,N) complex 
variable a nonnegative
variable E(N,N) nonnegative
variable F(N,N) nonnegative
sum1=0;%MSE
for ii=1:length(theta)
    sum1 = sum1 + square_pos(a*sqrt(p_d(ii))-norm(A(:,ii)'*C));
end
minimize (sum1 +rho*sum(sum(E)) + rho*sum(sum(F)))
subject to
for i=1:N
    for k=1:N
         square_pos(abs(C(i,k)))<=1+E(i,k);
        abs(C0(i,k))^2-real(2*C0(i,k)'*C(i,k))<=F(i,k)-1;
    end
end
cvx_end
    tf = strcmp(cvx_status,'Solved');
    if tf == 1  
%         err=norm(Phi0-Psi,'fro');
%         if err<=delta && max(max(A))<=delta  
        
%         if norm(A,'fro')<=delta  
        if max(max(E))<=delta && max(max(F))<=delta
            break;
        end
        rho=min(rho*1.1,2);
        C0 = C;
%         C0=exp(1i*angle(C));
    else
        break;
    end

disp([cvx_optval,iter,rho])
end
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