import matplotlib.pyplot as plt
import interp
from interp import Point
import math

p1 = Point(0, 0)
p2 = Point(1, 1)

qh = Point(1, .2)

ch1 = Point(.7, .1)
ch2 = Point(.3, .9)

STEPS = 10

step_array = [(1/STEPS) * i for i in range(STEPS+1)]

lerp = interp.nlerp(p1, p2, step_array, planar=True)
qlerp = interp.nquad_bez(p1, qh, p2, step_array, planar=True)
clerp = interp.ncubic_bez(p1, ch1, ch2, p2, step_array, planar=True)

# Plot the function
plt.plot(*lerp, color="blue", label="LERP")
# Connect endpoints

plt.plot(*qlerp, color='orange', label="Quad")
plt.plot([p1.x, qh.x, p2.x], [p1.y, qh.y, p2.y], color="orange", linestyle="--")  
plt.scatter([qh.x], [qh.y], color="orange")  #

plt.plot(*clerp, color='green', label="Cubic")
plt.plot([p1.x, ch1.x, ch2.x, p2.x], [p1.y, ch1.y, ch2.y, p2.y], color="green", linestyle="--")
plt.scatter([ch1.x, ch2.x], [ch1.y, ch2.y], color="green") 

plt.scatter([p1.x, p2.x], [p1.y, p2.y], color="blue")

def cosine(x):
    return (1-math.cos(x*math.pi))/2

cos = interp.ncosine_ease(step_array, planar=True)
plt.plot(*cos, color="red", label="Cosine")

plt.xlabel("Input (x)")
plt.ylabel("Interpolated Output (y)")
plt.title("Interpolation Function Visualization")
plt.legend()
plt.grid()
plt.show()
