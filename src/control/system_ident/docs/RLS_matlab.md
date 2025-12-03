# 系统辨识之递推最小二乘

## 经典RLS：
``` matlab
function th = RLS(yk,yk_1,yk_2,uk_1,uk_2)
persistent th0 P0
if isempty(th0)
    th0=zeros(4,1);
    P0=eye(4)*2e3;
end
fai=[yk_1,yk_2,uk_1,uk_2]';

th=th0+P0*fai/(fai'*P0*fai+1)*(yk-fai'*th0);
P0=P0-P0*(fai*fai')*P0/(fai'*P0*fai+1);
th0=th;

end
```

## 平方根RLS：

相比于经典RLS，平方根RLS的优势在于：
1. 计算量更小，因为平方根RLS只需要计算一次平方根，而经典RLS需要计算两次平方根。
2. 稳定性更好，因为平方根RLS的增益矩阵K是一个单位矩阵，而经典RLS的增益矩阵K是一个非单位矩阵。

除此之外，计算结果与经典RLS相同。

``` matlab

function th = RLSsqrt(yk,yk_1,yk_2,uk_1,uk_2)
persistent th0 S0
if isempty(th0)
    th0=zeros(4,1);
    S0=eye(4)*sqrt(2e3);
end
fai=[yk_1,yk_2,uk_1,uk_2]';%构造向量fai
f=S0'*fai;
g=1/(f'*f+1);
a=1/(1+sqrt(g));
K=S0*f*g;
th=th0+K*(yk-fai'*th0);
S0=S0*(eye(4)-g*a*(f*f'));
th0=th;

end
```


## 指数遗忘因子RLS：

``` matlab

function th = RLSEXP(yk,yk_1,yk_2,uk_1,uk_2)
%选取遗忘因子的准则：rho=1-1/N,N为想要记忆的历史数据长度
%比如N=100，则rho=0.99；
persistent th0 P0 rho
if isempty(th0)
    th0=zeros(4,1);
    P0=eye(4)*2e3;
    rho=0.99;
end
fai=[yk_1,yk_2,uk_1,uk_2]';

th=th0+P0*fai/(fai'*P0*fai+rho)*(yk-fai'*th0);
P0=(P0-P0*(fai*fai')*P0/(fai'*P0*fai+1))/rho;
th0=th;

end
```

## 变量遗忘因子RLS：

相比于指数遗忘因子RLS，变量遗忘因子RLS的优势在于：
1. 可以根据当前的预测误差动态调整遗忘因子，从而更好地记忆历史数据。
2. 可以避免遗忘因子过小导致的模型不稳定性。

通常可以根据当前的预测误差ek动态调整遗忘因子rho，具体方法如下：
``` matlab
rho=1-(1-fai'*K)*ek^2/Sigma;
```
其中，Sigma是观察噪声方差，N是记忆步长。

``` matlab
function [th,rho] = RLSVAR(yk,yk_1,yk_2,uk_1,uk_2)

persistent th0 P0 Sigma rhomin
if isempty(th0)
    th0=zeros(4,1);
    P0=eye(4)*2e3;
    N=500;      %一般记忆步长
    Nmin=50;    %最小记忆步长
    sig02=0.02; %观察噪声方差
    Sigma=sig02*N;
    rhomin=1-1/Nmin;
end
fai=[yk_1,yk_2,uk_1,uk_2]';
ek=(yk-fai'*th0);           %计算本步预测误差
K=P0*fai/(fai'*P0*fai+1);   %计算增益
th=th0+K*ek;                %修正估计
rho=1-(1-fai'*K)*ek^2/Sigma;%计算遗忘因子
if rho<rhomin               %防止遗忘因子过小
    rho=rhomin;
end
P0=(P0-P0*(fai*fai')*P0/(fai'*P0*fai+1))/rho;%更新协方差
th0=th;%更新记忆数据

end

```
