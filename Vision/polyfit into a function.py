import numpy as np
x=[1,2,3,0,-1,-2]
y=[-0.31,60.38,483.07,1,4.31,-50.38]
deg=5
polyfit=np.polyfit(x=x,y=y,deg=deg)
print(polyfit)
string='0'
polyfit=list(polyfit)
for i in polyfit:
    string += '+'+str(i)+'*x**'+str((deg-polyfit.index(i)))
distance = lambda x:eval(string)
print(string)
print(distance(5))
