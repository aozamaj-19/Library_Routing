# pip install ortools
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import math

# -----------------------------
# Helper: haversine distance -> minutes
# -----------------------------
def haversine_km(a, b):
    R = 6371.0
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    return 2 * R * math.asin(math.sqrt(h))

def build_time_matrix(coords, avg_kph=40):
    n = len(coords)
    mat = [[0]*n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                km = haversine_km(coords[i], coords[j])
                mat[i][j] = int((km / avg_kph) * 60)  # minutes
    return mat

# -----------------------------
# Build model
# -----------------------------
def create_data_model():
    data = {}

    # Depot = central library
    data["names"] = [
        "Central Library", "Branch A", "Branch B", "Branch C", "Branch D"
    ]

    # Coordinates (lat, lon) for each library
    data["locations"] = [
        (40.345, -74.563),  # Central
        (40.351, -74.570),  # Branch A
        (40.349, -74.585),  # Branch B
        (40.357, -74.580),  # Branch C
        (40.360, -74.572),  # Branch D
    ]

    # Build travel-time matrix (minutes)
    data["time_matrix"] = build_time_matrix(data["locations"], avg_kph=35)

    # Number of vans
    data["num_vehicles"] = 2
    # Depot index
    data["depot"] = 0

    # Demands (bins of books to deliver)
    data["demands"] = [0, 5, 3, 4, 6]

    # Vehicle capacities (bins per van)
    data["vehicle_capacities"] = [10, 10]

    # Library open hours: minutes from 8:00 AM
    # e.g., open 8:00–16:00 -> (0, 480)
    data["time_windows"] = [
        (0, 480),  # Central depot
        (60, 300), # Branch A open 9:00–13:00
        (0, 420),  # Branch B open 8:00–15:00
        (120, 480),# Branch C open 10:00–16:00
        (0, 480),  # Branch D open all day
    ]

    return data

# -----------------------------
# Solve routing
# -----------------------------
def solve_vrp(data):
    manager = pywrapcp.RoutingIndexManager(len(data["time_matrix"]),
                                           data["num_vehicles"], data["depot"])
    routing = pywrapcp.RoutingModel(manager)

    # Time callback
    def time_callback(from_index, to_index):
        return data["time_matrix"][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit_cb_idx = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_cb_idx)

    # Add time dimension
    routing.AddDimension(
        transit_cb_idx,
        30,    # allow waiting
        600,   # max route time
        False, # don't force start at 0
        "Time"
    )
    time_dim = routing.GetDimensionOrDie("Time")

    # Add time windows
    for node, window in enumerate(data["time_windows"]):
        index = manager.NodeToIndex(node)
        time_dim.CumulVar(index).SetRange(window[0], window[1])

    # Capacity constraints
    def demand_callback(from_index):
        return data["demands"][manager.IndexToNode(from_index)]

    demand_cb_idx = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_cb_idx,
        0,
        data["vehicle_capacities"],
        True,
        "Capacity"
    )

    # Search params
    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_params.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_params.time_limit.FromSeconds(5)

    solution = routing.SolveWithParameters(search_params)
    if not solution:
        print("No solution found")
        return

    # Print solution
    time_dim = routing.GetDimensionOrDie("Time")
    total_time = 0
    for v in range(data["num_vehicles"]):
        index = routing.Start(v)
        plan_output = f"\nVan {v} route:\n"
        route_time = 0
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            t = solution.Min(time_dim.CumulVar(index))
            plan_output += f"  {data['names'][node]} at {t} min\n"
            index = solution.Value(routing.NextVar(index))
        node = manager.IndexToNode(index)
        t = solution.Min(time_dim.CumulVar(index))
        plan_output += f"  {data['names'][node]} at {t} min\n"
        route_time += t
        print(plan_output)
        total_time += route_time
    print(f"\nTotal travel time: {total_time} minutes")

# -----------------------------
# Run example
# -----------------------------
if __name__ == "__main__":
    data = create_data_model()
    solve_vrp(data)
