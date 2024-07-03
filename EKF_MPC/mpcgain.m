function [Phi_Phi,Phi_F,Phi_R,A_e, B_e,C_e]=mpcgain(Ap,Bp,Cp,Nc,Np);
[m1,n1]=size(Cp);
[n1,n_in]=size(Bp);
A_e=eye(n1+m1,n1+m1);
A_e(1:n1,1:n1)=Ap;
A_e(n1+1:n1+m1,1:n1)=Cp*Ap;
B_e=zeros(n1+m1,n_in);
B_e(1:n1,:)=Bp;
B_e(n1+1:n1+m1,:)=Cp*Bp;
C_e=zeros(m1,n1+m1);
C_e(:,n1+1:n1+m1)=eye(m1,m1);

n=n1+m1;
h(1:2,:)=C_e;
F(1:2,:)=C_e*A_e;
for k=2:(2*Np-1)
    if rem(k,2)==0
    h(k+1:k+2,:)=h(k-1:k,:)*A_e;
    F(k+1:k+2,:)=F(k-1:k,:)*A_e;
    end
end
v=h*B_e;
Phi=zeros(2*Np,2*Nc); %declare the dimension of Phi
Phi(:,1:2)=v ;% first 2 column  column of Phi
for i=2:(2*Nc-1)
    if rem(i,2)==0
    Phi(:,i+1)=[zeros((i/2),1);v(1:(2*Np)-(i/2),1)]; 
    Phi(:,i+2)=[zeros((i/2),1);v(1:(2*Np)-(i/2),2)];%Toeplitz matrix
    end
end
BarRs=ones(2*Np,2);
Phi_Phi= Phi'*Phi;
Phi_F= Phi'*F;
Phi_R=Phi'*BarRs;
