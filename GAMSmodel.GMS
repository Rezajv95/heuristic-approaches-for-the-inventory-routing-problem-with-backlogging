sets
i nodes /depo,c1,c2,c3,c4/
t time perods /1*3/
v vehicle /v1*v2/
alias(i,j,k,l)
;

parameters f(v,t),c(i,j),pi(i),h(i),q(v),cap(i),d(i,t);

f(v,t)=uniformint(10,30);
pi(i)$(ord(i)>=2)=uniform(5,10);
h(i)$(ord(i)>=2)=uniform(3,7);
q(v)=uniformint(100,150);
cap(i)$(ord(i)>=2)=uniformint(200,300);
d(i,t)$(ord(i)>=2)=uniformint(50,70);

loop(i,loop(j,
c(i,j)=uniform(50,100);
c(j,i)=c(i,j);
c(i,i)=0;
););

variable z;
binary variable x(i,j,t,v);
positive variable y(i,j,t,v),inv(i,t),b(i,t);

equations
obj
co1
co2
co3
co4
co5
co6
co7
;

obj ..z=e=sum(t,(sum((j,v),f(v,t)*x('depo',j,t,v)))+sum((i,j,v),c(i,j)*x(i,j,t,v))
         +sum(i$(ord(i)>=2),h(i)*inv(i,t)+pi(i)*b(i,t)));
co1(i,t,v) .. sum(j$(ord(j)ne ord(i)),x(i,j,t,v))=l=1;
co2(i,t,v) .. sum(k,x(i,k,t,v))-sum(l,x(l,i,t,v))=e=0;
co3(i,j,t,v) .. y(i,j,t,v)-q(v)*x(i,j,t,v)=l=0;
co4(i,t,v)$(ord(i)>=2) ..sum(l,y(l,i,t,v))-sum(k,y(i,k,t,v))=g=0;
co5(i)$(ord(i)>=2) .. -inv(i,'1')+b(i,'1')+sum(v,(sum(l,y(l,i,'1',v))-sum(k,y(i,k,'1',v))))=e=d(i,'1');
co6(i,t)$(ord(i)>=2 and ord(t)>=2) .. inv(i,t-1)-b(i,t-1)-inv(i,t)+b(i,t)+sum(v,(sum(l,y(l,i,t,v))-sum(k,y(i,k,t,v))))=e=d(i,t);
co7(i,t)$(ord(i)>=2) .. inv(i,t)=l=cap(i);

x.fx(i,i,t,v)=0

model IRP /all/;
option optca=0,optcr=0,mip=cplex,reslim=3600;

solve IRP using mip min z;

display x.l,y.l,inv.l,b.l;
