import pyexotica as exo
from pyexotica.publish_trajectory import *

exo.Setup.init_ros()
solver = exo.Setup.load_solver(
    '{hsr123}/resources/rrt/rrt.xml')

solution = solver.solve()

plot(solution)

publish_trajectory(solution, 3.0, solver.get_problem())