from pyomo.environ import *
import random

random.seed(0)  # برای تولید مقادیر تصادفی تکرارپذیر

model = ConcreteModel()

# Sets
model.i = Set(initialize=['depo', 'c1', 'c2', 'c3', 'c4'])
model.t = Set(initialize=['1', '2', '3'])
model.v = Set(initialize=['v1', 'v2'])

# Parameters
def f_init(model, v, t):
    return random.uniform(10, 30)
model.f = Param(model.v, model.t, initialize=f_init)

def c_init(model, i, j):
    if i == j:
        return 0
    return random.uniform(50, 100)
model.c = Param(model.i, model.i, initialize=c_init)

def pi_init(model, i):
    if i != 'depo':
        return random.uniform(5, 10)
    return 0
model.pi = Param(model.i, initialize=pi_init)

def h_init(model, i):
    if i != 'depo':
        return random.uniform(3, 7)
    return 0
model.h = Param(model.i, initialize=h_init)

def q_init(model, v):
    return random.uniform(100, 150)
model.q = Param(model.v, initialize=q_init)

def cap_init(model, i):
    if i != 'depo':
        return random.uniform(200, 300)
    return 0
model.cap = Param(model.i, initialize=cap_init)

def d_init(model, i, t):
    if i != 'depo':
        return random.uniform(50, 70)
    return 0
model.d = Param(model.i, model.t, initialize=d_init)

# Variables
model.x = Var(model.i, model.i, model.t, model.v, within=Binary)
model.y = Var(model.i, model.i, model.t, model.v, within=NonNegativeReals)
model.inv = Var(model.i, model.t, within=NonNegativeReals)
model.b = Var(model.i, model.t, within=NonNegativeReals)

# Objective function
def obj_rule(model):
    return sum(model.f[v, t] * model.x['depo', j, t, v] for j in model.i for v in model.v for t in model.t) + \
           sum(model.c[i, j] * model.x[i, j, t, v] for i in model.i for j in model.i for t in model.t for v in model.v) + \
           sum(model.h[i] * model.inv[i, t] + model.pi[i] * model.b[i, t] for i in model.i for t in model.t if i != 'depo')
model.obj = Objective(rule=obj_rule, sense=minimize)

# Constraints
def co1_rule(model, i, t, v):
    if i != 'depo':
        return sum(model.x[i, j, t, v] for j in model.i if j != i) == 1
    return Constraint.Skip
model.co1 = Constraint(model.i, model.t, model.v, rule=co1_rule)

def co2_rule(model, i, t, v):
    return sum(model.x[i, k, t, v] for k in model.i) - sum(model.x[l, i, t, v] for l in model.i) == 0
model.co2 = Constraint(model.i, model.t, model.v, rule=co2_rule)

def co3_rule(model, i, j, t, v):
    return model.y[i, j, t, v] >= model.q[v] * model.x[i, j, t, v]
model.co3 = Constraint(model.i, model.i, model.t, model.v, rule=co3_rule)

# Fix x[i, i, t, v] to be 0 (no loops)
def fix_x_rule(model, i, t, v):
    return model.x[i, i, t, v] == 0
model.fix_x = Constraint(model.i, model.t, model.v, rule=fix_x_rule)

# Solve the model
solver = SolverFactory('cplex')
results = solver.solve(model)

# Display the results
for i in model.i:
    for j in model.i:
        for t in model.t:
            for v in model.v:
                if value(model.x[i, j, t, v]) > 0:
                    print(f"x[{i},{j},{t},{v}] = {value(model.x[i, j, t, v])}")

for i in model.i:
    for j in model.i:
        for t in model.t:
            for v in model.v:
                if value(model.y[i, j, t, v]) > 0:
                    print(f"y[{i},{j},{t},{v}] = {value(model.y[i, j, t, v])}")

for i in model.i:
    for t in model.t:
        print(f"inv[{i},{t}] = {value(model.inv[i, t])}")

for i in model.i:
    for t in model.t:
        print(f"b[{i},{t}] = {value(model.b[i, t])}")
