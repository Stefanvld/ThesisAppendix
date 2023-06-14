import gurobipy as gp
from gurobipy import GRB
import pandas as pd
import math

# Define Haversine formula
def CalculateDistance(LOC1: list, LOC2: list):
    # Convert latitude and longitude from degrees to radians
    lat1, lon1 = math.radians(LOC1[0]), math.radians(LOC1[1])
    lat2, lon2 = math.radians(LOC2[0]), math.radians(LOC2[1])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = (math.sin(dlat / 2) ** 2) + math.cos(lat1) * math.cos(lat2) * (math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Earth's mean radius in kilometers (6371 km)
    R = 6371
    distance = R * c

    return distance

def order_flows(flows):
    ordered_flows = {}

    for boat, flow in flows.items():
        # Create dict
        flow_dict = {f[0]: f for f in flow}

        # Find starting point
        start = None
        for f in flow:
            if "SDP" in f[0]:
                start = f.copy()  # Create copy
                break
        else:
            # In case no starting point found
            print(f"No 'SDP' starting point found for boat {boat}.")
            continue

        ordered_flow = [start]
        while len(ordered_flow) < len(flow):
            current = ordered_flow[-1]
            next_flow = flow_dict.get(current[1])
            if next_flow:
                ordered_flow.append(next_flow)
            else:
                # In case no matching flow
                print(f"No matching flow found for boat {boat} after {current}.")
                break

        ordered_flows[boat] = ordered_flow

    return ordered_flows


def order_routes(routes): 
    # And sort them
    ordered_routes = {}

    for x in routes:
        ordered_routes[x] = []
        for y in routes[x]:
            # Identify start
            s = [z for z in routes[x] if z[1] == y[0]]

            if len(s) == 0:
                start = y
                ordered_routes[x].append(start[0])

        while len(ordered_routes[x]) != len(routes[x])+1:
            # Find last index
            current_length = len(ordered_routes[x])-1
            current_pos = ordered_routes[x][current_length]

            # Find next index
            next_index = [z for z in routes[x] if z[0] == current_pos][0][1]
            ordered_routes[x].append(next_index)

    return(ordered_routes)

def OptimizeRoutes(optimization_input, time_limit):
    # Derive locations in one dict
    locations = {x:optimization_input['Shipments'][x]['Loc'] for x in optimization_input['Shipments']}
    locations.update({x:optimization_input['SupplyPoints'][x] for x in optimization_input['SupplyPoints']})
    locations.update({x:optimization_input['Depots']['Starting'][x]['Loc'] for x in optimization_input['Depots']['Starting']})
    locations.update({x:optimization_input['Depots']['Ending'][x]['Loc'] for x in optimization_input['Depots']['Ending']})

    locations

    # Create model
    model = gp.Model()

    # Create cost matrix C
    shipments = [x for x in optimization_input['Shipments']] 
    sdepots = [x for x in optimization_input['Depots']['Starting']]
    edepots = [x for x in optimization_input['Depots']['Ending']]
    supplypoints = [x for x in optimization_input['SupplyPoints']]
    indexes = shipments + sdepots + edepots + supplypoints

    C = {}
    for i in indexes:
        for j in indexes:
            C[i, j] = CalculateDistance(locations[i], locations[j])

    # Extract supplyboats
    supplyboats = [x for x in optimization_input['SupplyBoats']]
    supplyboats

    # Create decision variable matrix X
    X = {}
    for s in supplyboats:
        for i in indexes:
            for j in indexes:
                X[i, j, s] = model.addVar(vtype=gp.GRB.BINARY)

    # Objective function
    model.setObjective(gp.quicksum(X[i,j,s] * C[i,j] for i in indexes for j in indexes for s in supplyboats),gp.GRB.MINIMIZE)

    # Visit each shipment location only once
    model.addConstrs(gp.quicksum(X[i,j,s] for s in supplyboats for i in indexes if i !=j) == 1 for j in shipments)
    model.addConstrs(gp.quicksum(X[i,j,s] for s in supplyboats for j in indexes if i !=j) == 1 for i in shipments)

    # Ensure continuity
    model.addConstrs(gp.quicksum(X[i,h,s] for i in indexes) - gp.quicksum(X[h,j,s] for j in indexes) == 0 for s in supplyboats for h in shipments + supplypoints)

    # Do not allow 1-node cycles
    model.addConstrs(gp.quicksum(X[i,i,s] for i in indexes) == 0 for s in supplyboats)

    # Do not allow subtours
    u = {}
    for i in shipments+supplypoints:
        for s in supplyboats:
            u[i, s] = model.addVar(vtype=GRB.INTEGER, lb=0, ub=len(shipments+supplypoints)-1, name=f'u_{i}_{s}')

    for i in shipments+supplypoints:
        for s in supplyboats:
            model.addConstr(u[i,s] >= 1)
            model.addConstr(u[i,s] <= len(shipments+supplypoints)-1)

        for j in shipments+supplypoints:
            if i != j:
                for s in supplyboats:
                    model.addConstr(u[i,s] - u[j,s] + len(shipments+supplypoints) * X[i,j,s] <= len(shipments+supplypoints) - 1)

    # The boats must start and end at a SDP/EDP
    model.addConstrs(gp.quicksum(X[i,j,s] for i in sdepots for j in indexes) == 1 for s in supplyboats)
    model.addConstrs(gp.quicksum(X[i,j,s] for j in sdepots for i in indexes) == 0 for s in supplyboats)
    model.addConstrs(gp.quicksum(X[i,j,s] for i in edepots for j in indexes) == 0 for s in supplyboats)
    model.addConstrs(gp.quicksum(X[i,j,s] for j in edepots for i in indexes) == 1 for s in supplyboats)

    # Workload-balancing constraint
    model.addConstrs(gp.quicksum(X[i,j,s] * optimization_input['Shipments'][i]['WaterAmount'] for i in shipments for j in indexes) <= optimization_input['WB_percentage']*sum([optimization_input['Shipments'][x]['WaterAmount'] for x in optimization_input['Shipments']]) for s in supplyboats)

    # Capacity per depot
    model.addConstrs(gp.quicksum(X[i,j,s] for j in indexes for s in supplyboats) <= optimization_input['Depots']['Starting'][i]['Cap'] for i in sdepots)
    model.addConstrs(gp.quicksum(X[i,j,s] for i in indexes for s in supplyboats) <= optimization_input['Depots']['Ending'][j]['Cap'] for j in edepots)

    # Let SDP = EDP for every boat
    model.addConstrs(gp.quicksum(X['SDP{i}'.format(i=str(i+1)),j,s] for j in indexes) == gp.quicksum(X[j,'EDP{i}'.format(i=str(i+1)),s] for j in indexes) for i in range(1, len(sdepots)) for s in supplyboats) 

    # Add Y variables for water flow
    Y = {}
    for i in indexes:
        for j in indexes:
            for s in supplyboats:
                Y[i,j,s] = model.addVar(vtype=gp.GRB.CONTINUOUS)

    # Y cannot be more than the individual capacity of a waterboat
    model.addConstrs(Y[i,j,s] <= optimization_input['SupplyBoats'][s]*X[i,j,s] for i in indexes for j in indexes for s in supplyboats)

    # Y cannot be less than zero. 
    model.addConstrs(Y[i,j,s] >= 0 for i in indexes for j in indexes for s in supplyboats)

    # Subtract the amount delivered at point i 
    model.addConstrs(gp.quicksum(Y[j,i,s] for j in indexes if i != j for s in supplyboats) - gp.quicksum(Y[i,j,s] for j in indexes if i != j for s in supplyboats) == optimization_input['Shipments'][i]['WaterAmount'] for i in shipments)

    # Refill tank at supply points and depots
    model.addConstrs(Y[i,j,s] == optimization_input['SupplyBoats'][s]*X[i,j,s] for i in supplypoints+sdepots+edepots for j in indexes for s in supplyboats)

    if time_limit != None:
        model.params.timeLimit = time_limit
    
    model.update()
    model.optimize()

    # Print solution
    print("Optimal solution found. Distance: {dist} km.".format(dist=round(model.objVal,2)))

    # Extract routes from solution
    routes = {}
    for x in supplyboats:
        routes[x] = []

    for x in X:
        if X[x].X > 0.5: # > 0.5 because == 1 might cause problems due to Python's way of storing integers
            routes[x[2]].append([x[0],x[1]])

    # Try to sort them
    # The reason that this is put in a try statement, is that we don't want the calculations to be interrupted if an error occurs in one of the order functions whilst the optimization function still runs perfectly fine. 
    try:
        ordered_routes = order_routes(routes)

    except:
        ordered_routes = []
   
    
    # Extract flow from the solution
    flow = {x:[] for x in supplyboats}
    for z in Y:
        if X[z].X > 0.5: # We check this using X rather than Y, because this is more precise
            flow[z[2]].append([z[0],z[1],Y[z].X])

    # Try to order flow as well
    try:
        ordered_flows = order_flows(flow)
        for x in ordered_flows:
            ordered_flows[x] = [y[2] for y in ordered_flows[x]] # to only show numbers
            ordered_flows[x] = [int(z) for z in ordered_flows[x]] # to keep it tidy
        

    except:
        ordered_flows = []

    # Finally, we'll add the ordered_routes but using AIS points rather than the location names
    try:
        ais_routes = ordered_routes.copy()
        for x in ais_routes:
            ais_routes[x] = [locations[y] for y in ais_routes[x]]

    except:
        ais_routes = {}
    
    return(model.objVal, model.Runtime, routes, flow, ordered_routes, ordered_flows, ais_routes)