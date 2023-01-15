#from TSP_Model import Model
import VRP_Model as md
import Solver as gls
import Tabu_Search as tb

m = md.Model()
m.BuildModel()
tabu_solver = tb.Solver(m)
gls_solver = gls.Solver(m)
tabu_solution = tabu_solver.solve()
gls_solver.Initialize(m)
gls_solver.penalize_weight = 0.06
gls_solver.sol = tabu_solution
sol = gls_solver.solve2()
gls_solver.sol = sol
gls_solver.penalize_weight = 0.5
gls_solver.solve2()




