import numpy as np
import matplotlib.pyplot as plt

x = [5,15,25,35,45]
y = [0.713,0.936,0.949,0.954,0.961]

plt.plot(x,y,'-ro')

plt.xlabel('number of polygon edges')
plt.ylabel('${A_{approx}}/{A_{actual}}$',rotation=90)
#plt.xlim(0,50)
plt.xticks(np.arange(0, 55, 5))
plt.grid()
plt.show()
