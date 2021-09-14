function obs=genObsPoly(v1,v2,n)

    for i=1:n-1
        obs(i,1)=(i*v1(1)+(n-i)*v2(1))/n;
        obs(i,2)=(i*v1(2)+(n-i)*v2(2))/n;
    end
end