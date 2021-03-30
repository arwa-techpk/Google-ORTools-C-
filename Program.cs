using Google.OrTools.ConstraintSolver;
using Google.Protobuf.WellKnownTypes;
using System;
using System.Linq;

namespace ConsoleApp_ORtool
{
    class Program
    {
        //dimensions are used to add more conditions
        /*https://developers.google.com/optimization/routing/dimensions*/
        static void Main(string[] args)
        {
            Console.WriteLine("Google OR Tools Demo!");
            VRP_LongestSingleRoute();
            //CapacityConstraints();
            //PickupsandDeliveries();
            //VRPTWs();
            //DepotLoadUnload(); // solutions can't find in console app
            //Penalties();
            //InitialRoute();
            //StartEnd();
        }

        #region Vehicle Routing Problem
        /// <summary>
        ///   Print the solution.
        /// </summary>
         static void VRP_LongestSingleRoute()
        {
            Console.WriteLine("VRP_LongestSingleRoute!");
            // Instantiate the data problem.
            ORDataModel data = new ORDataModel();

        // public RoutingIndexManager(int num_nodes, int num_vehicles, int depot);
        //public RoutingIndexManager(int num_nodes, int num_vehicles, int[] starts, int[] ends);

        // Create Routing Index Manager
        RoutingIndexManager manager = new RoutingIndexManager(
            data.DistanceMatrix.GetLength(0),
            data.VehicleNumber,
            data.Depot);


        // Create Routing Model.
        RoutingModel routing = new RoutingModel(manager);

        // Create and register a transit callback.
        int transitCallbackIndex = routing.RegisterTransitCallback(
            (long fromIndex, long toIndex) => {
                    // Convert from routing variable Index to distance matrix NodeIndex.
                    var fromNode = manager.IndexToNode(fromIndex);
                var toNode = manager.IndexToNode(toIndex);
                return data.DistanceMatrix[fromNode, toNode];
            }
            );

        // Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

            // Add Distance constraint.
            routing.AddDimension(transitCallbackIndex, 0, 3000,
                true,  // start cumul to zero
                "Distance");
            RoutingDimension distanceDimension = routing.GetMutableDimension("Distance");
        distanceDimension.SetGlobalSpanCostCoefficient(100);

            // Setting first solution heuristic.
            RoutingSearchParameters searchParameters =
              operations_research_constraint_solver.DefaultRoutingSearchParameters();
        searchParameters.FirstSolutionStrategy =
              FirstSolutionStrategy.Types.Value.PathCheapestArc;

            // Solve the problem.
            Assignment solution = routing.SolveWithParameters(searchParameters);

        // Print solution on console.
        PrintSolutionPathCheapest(data, routing, manager, solution);
    }
    static void PrintSolutionPathCheapest(
            in ORDataModel data,
            in RoutingModel routing,
            in RoutingIndexManager manager,
            in Assignment solution)
        {
            // Inspect solution.
            long maxRouteDistance = 0;
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                Console.WriteLine("Route for Vehicle {0}:", i);
                long routeDistance = 0;
                var index = routing.Start(i);
                while (routing.IsEnd(index) == false)
                {
                    Console.Write("{0} -> ", manager.IndexToNode((int)index));
                    var previousIndex = index;
                    index = solution.Value(routing.NextVar(index));
                    routeDistance += routing.GetArcCostForVehicle(previousIndex, index, 0);
                }
                Console.WriteLine("{0}", manager.IndexToNode((int)index));
                Console.WriteLine("Distance of the route: {0}m", routeDistance);
                maxRouteDistance = Math.Max(routeDistance, maxRouteDistance);
            }
            Console.WriteLine("Maximum distance of the routes: {0}m", maxRouteDistance);
        }
        #endregion

        #region Capacity Constraints
        /// <summary>
        ///   Print the solution.
        /// </summary>
        static void PrintSolutionCapacityConstraints(
            in ORDataModel data,
            in RoutingModel routing,
            in RoutingIndexManager manager,
            in Assignment solution)
        {
            // Inspect solution.
            long totalDistance = 0;
            long totalLoad = 0;
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                Console.WriteLine("Route for Vehicle {0}:", i);
                long routeDistance = 0;
                long routeLoad = 0;
                var index = routing.Start(i);
                while (routing.IsEnd(index) == false)
                {
                    long nodeIndex = manager.IndexToNode(index);
                    routeLoad += data.Demands[nodeIndex];
                    Console.Write("{0} Load({1}) -> ", nodeIndex, routeLoad);
                    var previousIndex = index;
                    index = solution.Value(routing.NextVar(index));
                    routeDistance += routing.GetArcCostForVehicle(previousIndex, index, 0);
                }
                Console.WriteLine("{0}", manager.IndexToNode((int)index));
                Console.WriteLine("Distance of the route: {0}m", routeDistance);
                totalDistance += routeDistance;
                totalLoad += routeLoad;
            }
            Console.WriteLine("Total distance of all routes: {0}m", totalDistance);
            Console.WriteLine("Total load of all routes: {0}m", totalLoad);
        }

        public static void CapacityConstraints( )
        {
            // Instantiate the data problem.
            ORDataModel data = new ORDataModel();

            // Create Routing Index Manager
            RoutingIndexManager manager = new RoutingIndexManager(
                data.DistanceMatrix.GetLength(0),
                data.VehicleNumber,
                data.Depot);

            // Create Routing Model.
            RoutingModel routing = new RoutingModel(manager);

            // Create and register a transit callback.
            int transitCallbackIndex = routing.RegisterTransitCallback(
              (long fromIndex, long toIndex) => {
          // Convert from routing variable Index to distance matrix NodeIndex.
          var fromNode = manager.IndexToNode(fromIndex);
                  var toNode = manager.IndexToNode(toIndex);
                  return data.DistanceMatrix[fromNode, toNode];
              }
            );

            // Define cost of each arc.
            routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

            // Add Capacity constraint.
            int demandCallbackIndex = routing.RegisterUnaryTransitCallback(
              (long fromIndex) => {
          // Convert from routing variable Index to demand NodeIndex.
          var fromNode = manager.IndexToNode(fromIndex);
                  return data.Demands[fromNode];
              }
            );
            routing.AddDimensionWithVehicleCapacity(
              demandCallbackIndex, 0,  // null capacity slack
              data.VehicleCapacities,   // vehicle maximum capacities
              true,                      // start cumul to zero
              "Capacity");

            // Setting first solution heuristic.
            RoutingSearchParameters searchParameters =
              operations_research_constraint_solver.DefaultRoutingSearchParameters();
            searchParameters.FirstSolutionStrategy =
              FirstSolutionStrategy.Types.Value.PathCheapestArc;

            // Solve the problem.
            Assignment solution = routing.SolveWithParameters(searchParameters);

            // Print solution on console.
            PrintSolutionCapacityConstraints(data, routing, manager, solution);
        }
        #endregion CVRP

        #region Vehicle Routing with Pickups and Deliveries
        //a VRP in which each vehicle picks up items at various locations and drops them off at others.
        /// <summary>
        ///   Print the solution.
        /// </summary>
        static void PrintSolutionPickupsandDeliveries(
            in ORDataModel data,
            in RoutingModel routing,
            in RoutingIndexManager manager,
            in Assignment solution)
        {
            long totalDistance = 0;
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                Console.WriteLine("Route for Vehicle {0}:", i);
                long routeDistance = 0;
                var index = routing.Start(i);
                while (routing.IsEnd(index) == false)
                {
                    Console.Write("{0} -> ", manager.IndexToNode((int)index));
                    var previousIndex = index;
                    index = solution.Value(routing.NextVar(index));
                    routeDistance += routing.GetArcCostForVehicle(previousIndex, index, 0);
                }
                Console.WriteLine("{0}", manager.IndexToNode((int)index));
                Console.WriteLine("Distance of the route: {0}m", routeDistance);
                totalDistance += routeDistance;
            }
            Console.WriteLine("Total Distance of all routes: {0}m", totalDistance);
        }

        public static void PickupsandDeliveries()
        {
            // Instantiate the data problem.
            ORDataModel data = new ORDataModel();

            // Create Routing Index Manager
            RoutingIndexManager manager = new RoutingIndexManager(
                data.DistanceMatrix.GetLength(0),
                data.VehicleNumber,
                data.Depot);


            // Create Routing Model.
            RoutingModel routing = new RoutingModel(manager);

            // Create and register a transit callback.
            int transitCallbackIndex = routing.RegisterTransitCallback(
                (long fromIndex, long toIndex) => {
            // Convert from routing variable Index to distance matrix NodeIndex.
            var fromNode = manager.IndexToNode(fromIndex);
                    var toNode = manager.IndexToNode(toIndex);
                    return data.DistanceMatrix[fromNode, toNode];
                }
                );

            // Define cost of each arc.
            routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

            // Add Distance constraint.
            routing.AddDimension(transitCallbackIndex, 0, 3000,
                true,  // start cumul to zero
                "Distance");
            RoutingDimension distanceDimension = routing.GetMutableDimension("Distance");
            distanceDimension.SetGlobalSpanCostCoefficient(100);

            // Define Transportation Requests.
            Solver solver = routing.solver();
            for (int i = 0; i < data.PickupsDeliveries.GetLength(0); i++)
            {
                long pickupIndex = manager.NodeToIndex(data.PickupsDeliveries[i][0]);
                long deliveryIndex = manager.NodeToIndex(data.PickupsDeliveries[i][1]);
                routing.AddPickupAndDelivery(pickupIndex, deliveryIndex);
                solver.Add(solver.MakeEquality(
                      routing.VehicleVar(pickupIndex),
                      routing.VehicleVar(deliveryIndex)));
                solver.Add(solver.MakeLessOrEqual(
                      distanceDimension.CumulVar(pickupIndex),
                      distanceDimension.CumulVar(deliveryIndex)));
            }

            // Setting first solution heuristic.
            RoutingSearchParameters searchParameters =
              operations_research_constraint_solver.DefaultRoutingSearchParameters();
            searchParameters.FirstSolutionStrategy =
              FirstSolutionStrategy.Types.Value.PathCheapestArc;

            // Solve the problem.
            Assignment solution = routing.SolveWithParameters(searchParameters);

            // Print solution on console.
            PrintSolutionPickupsandDeliveries(data, routing, manager, solution);
        }

        #endregion

        //Many vehicle routing problems involve scheduling visits to customers who are only available during specific time windows.
        #region Vehicle Routing Problem with Time Windows VRPTWs
        /// <summary>
        ///   Print the solution.
        /// </summary>
        static void PrintSolutionVRPTWs(
            in ORDataModel data,
            in RoutingModel routing,
            in RoutingIndexManager manager,
            in Assignment solution)
        {
            RoutingDimension timeDimension = routing.GetMutableDimension("Time");
            // Inspect solution.
            long totalTime = 0;
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                Console.WriteLine("Route for Vehicle {0}:", i);
                var index = routing.Start(i);
                while (routing.IsEnd(index) == false)
                {
                    var timeVar = timeDimension.CumulVar(index);
                    Console.Write("{0} Time({1},{2}) -> ",
                        manager.IndexToNode(index),
                        solution.Min(timeVar),
                        solution.Max(timeVar));
                    index = solution.Value(routing.NextVar(index));
                }
                var endTimeVar = timeDimension.CumulVar(index);
                Console.WriteLine("{0} Time({1},{2})",
                    manager.IndexToNode(index),
                    solution.Min(endTimeVar),
                    solution.Max(endTimeVar));
                Console.WriteLine("Time of the route: {0}min", solution.Min(endTimeVar));
                totalTime += solution.Min(endTimeVar);
            }
            Console.WriteLine("Total time of all routes: {0}min", totalTime);
        }

        public static void VRPTWs()
        {
            // Instantiate the data problem.
            ORDataModel data = new ORDataModel();

            // Create Routing Index Manager
            RoutingIndexManager manager = new RoutingIndexManager(
                data.TimeMatrix.GetLength(0),
                data.VehicleNumber,
                data.Depot);

            // Create Routing Model.
            RoutingModel routing = new RoutingModel(manager);

            // Create and register a transit callback.
            int transitCallbackIndex = routing.RegisterTransitCallback(
                (long fromIndex, long toIndex) => {
            // Convert from routing variable Index to distance matrix NodeIndex.
            var fromNode = manager.IndexToNode(fromIndex);
                    var toNode = manager.IndexToNode(toIndex);
                    return data.TimeMatrix[fromNode, toNode];
                }
                );

            // Define cost of each arc.
            routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

            // Add Distance constraint.
            routing.AddDimension(
                transitCallbackIndex, // transit callback
                30, // allow waiting time
                30, // vehicle maximum capacities
                false,  // start cumul to zero
                "Time");
            RoutingDimension timeDimension = routing.GetMutableDimension("Time");
            // Add time window constraints for each location except depot.
            for (int i = 1; i < data.TimeWindows.GetLength(0); ++i)
            {
                long index = manager.NodeToIndex(i);
                timeDimension.CumulVar(index).SetRange(
                    data.TimeWindows[i, 0],
                    data.TimeWindows[i, 1]);
            }
            // Add time window constraints for each vehicle start node.
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                long index = routing.Start(i);
                timeDimension.CumulVar(index).SetRange(
                    data.TimeWindows[0, 0],
                    data.TimeWindows[0, 1]);
            }

            // Instantiate route start and end times to produce feasible times.
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                routing.AddVariableMinimizedByFinalizer(
                    timeDimension.CumulVar(routing.Start(i)));
                routing.AddVariableMinimizedByFinalizer(
                    timeDimension.CumulVar(routing.End(i)));
            }

            // Setting first solution heuristic.
            RoutingSearchParameters searchParameters =
              operations_research_constraint_solver.DefaultRoutingSearchParameters();
            searchParameters.FirstSolutionStrategy =
              FirstSolutionStrategy.Types.Value.PathCheapestArc;

            // Solve the problem.
            Assignment solution = routing.SolveWithParameters(searchParameters);

            // Print solution on console.
            PrintSolutionVRPTWs(data, routing, manager, solution);
        }
        #endregion

        #region capacitated vehicle routing problem Depot load unload
        /// <summary>
        ///   Print the solution.
        /// </summary>
        static void PrintSolutionDepotLoadUnload(
            in ORDataModel data,
            in RoutingModel routing,
            in RoutingIndexManager manager,
            in Assignment solution)
        {
            RoutingDimension timeDimension = routing.GetMutableDimension("Time");
            // Inspect solution.
            long totalTime = 0;
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                Console.WriteLine("Route for Vehicle {0}:", i);
                var index = routing.Start(i);
                while (routing.IsEnd(index) == false)
                {
                    var timeVar = timeDimension.CumulVar(index);
                    Console.Write("{0} Arrival Time(minimum time: {1}, maximum time: {2}) -> ",
                        manager.IndexToNode(index),
                        solution.Min(timeVar),
                        solution.Max(timeVar));
                    index = solution.Value(routing.NextVar(index));
                }
                var endTimeVar = timeDimension.CumulVar(index);
           //     var BreakIntervalsOfVehicle = solution.IntervalVarContainer();
               
                Console.WriteLine("{0} Arrival Time(minimum total time: {1}, maximum total time: {2})",
                    manager.IndexToNode(index),
                    solution.Min(endTimeVar),
                    solution.Max(endTimeVar));
                Console.WriteLine("Time of the route: {0}min", solution.Min(endTimeVar));
                totalTime += solution.Min(endTimeVar);

                Console.WriteLine("");
            }
            Console.WriteLine("Total time of all routes: {0}min", totalTime);
        }

        public static void DepotLoadUnload()
        {
            // Instantiate the data problem.
            ORDataModel data = new ORDataModel();


            // Create Routing Index Manager
            // [START index_manager]
            RoutingIndexManager manager = new RoutingIndexManager(
                data.TimeMatrix.GetLength(0),
                data.VehicleNumber,
                data.Depot);
            // [END index_manager]

            // Create Routing Model.
            // [START routing_model]
            RoutingModel routing = new RoutingModel(manager);
            // [END routing_model]

            // Create and register a transit callback.
            // [START transit_callback]
            int transitCallbackIndex = routing.RegisterTransitCallback(
                (long fromIndex, long toIndex) => {
            // Convert from routing variable Index to distance matrix NodeIndex.
            var fromNode = manager.IndexToNode(fromIndex);
                    var toNode = manager.IndexToNode(toIndex);
                    return data.TimeMatrix[fromNode, toNode];
                }
                );
            // [END transit_callback]

            // Define cost of each arc.
            // [START arc_cost]
            routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);
            // [END arc_cost]

            // Add Distance constraint.
            // [START time_constraint]
            routing.AddDimension(
                transitCallbackIndex, // transit callback
                30, // allow waiting time
                30, // vehicle maximum capacities
                false,  // start cumul to zero
                "Time");
            RoutingDimension timeDimension = routing.GetMutableDimension("Time");
            // Add time window constraints for each location except depot.
            for (int i = 1; i < data.TimeWindows.GetLength(0); ++i)
            {
                long index = manager.NodeToIndex(i);
                timeDimension.CumulVar(index).SetRange(
                    data.TimeWindows[i, 0],
                    data.TimeWindows[i, 1]);
            }
            // Add time window constraints for each vehicle start node.
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                long index = routing.Start(i);
                timeDimension.CumulVar(index).SetRange(
                    data.TimeWindows[0, 0],
                    data.TimeWindows[0, 1]);
            }
            // [END time_constraint]

            // Add resource constraints at the depot.
            // [START depot_load_time]
            Solver solver = routing.solver();
            IntervalVar[] intervals = new IntervalVar[data.VehicleNumber * 2];
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                // Add load duration at start of routes
                intervals[2 * i] = solver.MakeFixedDurationIntervalVar(
                      timeDimension.CumulVar(routing.Start(i)), data.VehicleLoadTime,
                      "depot_interval");
                // Add unload duration at end of routes.
                intervals[2 * i + 1] = solver.MakeFixedDurationIntervalVar(
                      timeDimension.CumulVar(routing.End(i)), data.VehicleUnloadTime,
                      "depot_interval");
            }
            // [END depot_load_time]

            // [START depot_capacity]
            long[] depot_usage = Enumerable.Repeat<long>(1, intervals.Length).ToArray();
            solver.Add(solver.MakeCumulative(intervals, depot_usage,
                  data.DepotCapacity, "depot"));
            // [END depot_capacity]

            // Instantiate route start and end times to produce feasible times.
            // [START depot_start_end_times]
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                routing.AddVariableMinimizedByFinalizer(
                    timeDimension.CumulVar(routing.Start(i)));
                routing.AddVariableMinimizedByFinalizer(
                    timeDimension.CumulVar(routing.End(i)));
            }
            // [END depot_start_end_times]

            // Setting first solution heuristic.
            // [START parameters]
            RoutingSearchParameters searchParameters =
              operations_research_constraint_solver.DefaultRoutingSearchParameters();
            searchParameters.FirstSolutionStrategy =
              FirstSolutionStrategy.Types.Value.PathCheapestArc;
            // [END parameters]

            // Solve the problem.
            // [START solve]
            Assignment solution = routing.SolveWithParameters(searchParameters);
            // [END solve]

            // Print solution on console.
            PrintSolutionDepotLoadUnload(data, routing, manager, solution);
        }
        #endregion

        #region Penalties and Dropping Visits
        /// <summary>
        ///   Print the solution.
        /// </summary>
        static void PrintSolutionPenalties(
            in ORDataModel data,
            in RoutingModel routing,
            in RoutingIndexManager manager,
            in Assignment solution)
        {
            // Display dropped nodes.
            string droppedNodes = "Dropped nodes:";
            for (int index = 0; index < routing.Size(); ++index)
            {
                if (routing.IsStart(index) || routing.IsEnd(index))
                {
                    continue;
                }
                if (solution.Value(routing.NextVar(index)) == index)
                {
                    droppedNodes += " " + manager.IndexToNode(index);
                }
            }
            Console.WriteLine("{0}", droppedNodes);
            // Inspect solution.
            long totalDistance = 0;
            long totalLoad = 0;
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                Console.WriteLine("Route for Vehicle {0}:", i);
                long routeDistance = 0;
                long routeLoad = 0;
                var index = routing.Start(i);
                while (routing.IsEnd(index) == false)
                {
                    long nodeIndex = manager.IndexToNode(index);
                    routeLoad += data.Demands[nodeIndex];
                    Console.Write("{0} Load({1}) -> ", nodeIndex, routeLoad);
                    var previousIndex = index;
                    index = solution.Value(routing.NextVar(index));
                    routeDistance += routing.GetArcCostForVehicle(previousIndex, index, 0);
                }
                Console.WriteLine("{0}", manager.IndexToNode((int)index));
                Console.WriteLine("Distance of the route: {0}m", routeDistance);
                totalDistance += routeDistance;
                totalLoad += routeLoad;
            }
            Console.WriteLine("Total Distance of all routes: {0}m", totalDistance);
            Console.WriteLine("Total Load of all routes: {0}m", totalLoad);
        }

        public static void Penalties()
        {
            // Instantiate the data problem.
            ORDataModel data = new ORDataModel();

            // Create Routing Index Manager
            RoutingIndexManager manager = new RoutingIndexManager(
                data.DistanceMatrix.GetLength(0),
                data.VehicleNumber,
                data.Depot);

            // Create Routing Model.
            RoutingModel routing = new RoutingModel(manager);

            // Create and register a transit callback.
            int transitCallbackIndex = routing.RegisterTransitCallback(
                (long fromIndex, long toIndex) => {
            // Convert from routing variable Index to distance matrix NodeIndex.
            var fromNode = manager.IndexToNode(fromIndex);
                    var toNode = manager.IndexToNode(toIndex);
                    return data.DistanceMatrix[fromNode, toNode];
                }
                );

            // Define cost of each arc.
            routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

            // Add Capacity constraint.
            int demandCallbackIndex = routing.RegisterUnaryTransitCallback(
                (long fromIndex) => {
            // Convert from routing variable Index to demand NodeIndex.
            var fromNode = manager.IndexToNode(fromIndex);
                    return data.Demands[fromNode];
                }
                );
            routing.AddDimensionWithVehicleCapacity(
                demandCallbackIndex, 0,  // null capacity slack
                data.VehicleCapacities,   // vehicle maximum capacities
                true,                      // start cumul to zero
                "Capacity");
            // Allow to drop nodes.
            long penalty = 1000;
            for (int i = 1; i < data.DistanceMatrix.GetLength(0); ++i)
            {
                routing.AddDisjunction(
                    new long[] { manager.NodeToIndex(i) }, penalty);
            }

            // Setting first solution heuristic.
            RoutingSearchParameters searchParameters =
              operations_research_constraint_solver.DefaultRoutingSearchParameters();
            searchParameters.FirstSolutionStrategy =
              FirstSolutionStrategy.Types.Value.PathCheapestArc;

            // Solve the problem.
            Assignment solution = routing.SolveWithParameters(searchParameters);

            // Print solution on console.
            PrintSolutionPenalties(data, routing, manager, solution);
        }
        #endregion

        #region VRP with initial Routes
        /// <summary>
        ///   Print the solution.
        /// </summary>
        static void PrintSolutionInitialRoutes(
            in ORDataModel data,
            in RoutingModel routing,
            in RoutingIndexManager manager,
            in Assignment solution)
        {
            // Inspect solution.
            long maxRouteDistance = 0;
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                Console.WriteLine("Route for Vehicle {0}:", i);
                long routeDistance = 0;
                var index = routing.Start(i);
                while (routing.IsEnd(index) == false)
                {
                    Console.Write("{0} -> ", manager.IndexToNode((int)index));
                    var previousIndex = index;
                    index = solution.Value(routing.NextVar(index));
                    routeDistance += routing.GetArcCostForVehicle(previousIndex, index, 0);
                }
                Console.WriteLine("{0}", manager.IndexToNode((int)index));
                Console.WriteLine("Distance of the route: {0}", routeDistance);
                maxRouteDistance = Math.Max(routeDistance, maxRouteDistance);
            }
            Console.WriteLine("Maximum distance of the routes: {0}", maxRouteDistance);
        }

        public static void InitialRoute()
        {
            // Instantiate the data problem.
            ORDataModel data = new ORDataModel();

            // Create Routing Index Manager
            RoutingIndexManager manager = new RoutingIndexManager(
                data.DistanceMatrix.GetLength(0),
                data.VehicleNumber,
                data.Depot);

            // Create Routing Model.
            RoutingModel routing = new RoutingModel(manager);

            // Create and register a transit callback.
            int transitCallbackIndex = routing.RegisterTransitCallback(
                (long fromIndex, long toIndex) => {
            // Convert from routing variable Index to distance matrix NodeIndex.
            var fromNode = manager.IndexToNode(fromIndex);
                    var toNode = manager.IndexToNode(toIndex);
                    return data.DistanceMatrix[fromNode, toNode];
                }
                );

            // Define cost of each arc.
            routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

            // Add Distance constraint.
            routing.AddDimension(transitCallbackIndex, 0, 3000,
                true,  // start cumul to zero
                "Distance");
            RoutingDimension distanceDimension = routing.GetMutableDimension("Distance");
            distanceDimension.SetGlobalSpanCostCoefficient(100);

            // Get inital solution from routes.
            Assignment initialSolution = routing.ReadAssignmentFromRoutes(
                data.InitialRoutes, true);
            // Print initial solution on console.
            Console.WriteLine("Initial solution:");
            PrintSolutionInitialRoutes(data, routing, manager, initialSolution);

            // Setting first solution heuristic.
            RoutingSearchParameters searchParameters =
              operations_research_constraint_solver.DefaultRoutingSearchParameters();

            // Solve the problem.
            Assignment solution = routing.SolveWithParameters(searchParameters);

            // Print solution on console.
            Console.WriteLine("");
            Console.WriteLine("Solution after search:");
            PrintSolutionInitialRoutes(data, routing, manager, solution);
        }
        #endregion

        #region Setting start and end locations for routes
        /// <summary>
        ///   Print the solution.
        /// </summary>
        static void PrintSolutionStartEnd(
            in ORDataModel data,
            in RoutingModel routing,
            in RoutingIndexManager manager,
            in Assignment solution)
        {
            // Inspect solution.
            long maxRouteDistance = 0;
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                Console.WriteLine("Route for Vehicle {0}:", i);
                long routeDistance = 0;
                var index = routing.Start(i);
                while (routing.IsEnd(index) == false)
                {
                    Console.Write("{0} -> ", manager.IndexToNode((int)index));
                    var previousIndex = index;
                    index = solution.Value(routing.NextVar(index));
                    routeDistance += routing.GetArcCostForVehicle(previousIndex, index, 0);
                }
                Console.WriteLine("{0}", manager.IndexToNode((int)index));
                Console.WriteLine("Distance of the route: {0}m", routeDistance);
                maxRouteDistance = Math.Max(routeDistance, maxRouteDistance);
            }
            Console.WriteLine("Maximum distance of the routes: {0}m", maxRouteDistance);
        }

        public static void StartEnd()
        {
            // Instantiate the data problem.
            ORDataModel data = new ORDataModel();

            // Create Routing Index Manager
            RoutingIndexManager manager = new RoutingIndexManager(
                data.DistanceMatrix.GetLength(0),
                data.VehicleNumber,
                data.Starts,
                data.Ends);

            // Create Routing Model.
            RoutingModel routing = new RoutingModel(manager);

            // Create and register a transit callback.
            int transitCallbackIndex = routing.RegisterTransitCallback(
              (long fromIndex, long toIndex) => {
          // Convert from routing variable Index to distance matrix NodeIndex.
          var fromNode = manager.IndexToNode(fromIndex);
                  var toNode = manager.IndexToNode(toIndex);
                  return data.DistanceMatrix[fromNode, toNode];
              }
            );

            // Define cost of each arc.
            routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

            // Add Distance constraint.
            routing.AddDimension(transitCallbackIndex, 0, 2000,
                                 true,  // start cumul to zero
                                 "Distance");
            RoutingDimension distanceDimension = routing.GetMutableDimension("Distance");
            distanceDimension.SetGlobalSpanCostCoefficient(100);

            // Setting first solution heuristic.
            RoutingSearchParameters searchParameters =
              operations_research_constraint_solver.DefaultRoutingSearchParameters();
            searchParameters.FirstSolutionStrategy =
              FirstSolutionStrategy.Types.Value.PathCheapestArc;

            // Solve the problem.
            Assignment solution = routing.SolveWithParameters(searchParameters);

            // Print solution on console.
            PrintSolutionStartEnd(data, routing, manager, solution);
        }
        #endregion
    }
    public class ORToolService 
    {

    }
    public class ORDataModel
    {
        // this cal

        public int[] ServiceTimes =
        {
                0,
                10,
                05,
                05,
                10,
                05,
                10,
                10
            };

        public string[] Destinations =
        {
                //Case 1
                "Dubai Investment Park-2",
                "Alphamed - Rd B1",
                "6 31 A St",
                "Unnamed Road",
                "15th St",
                "20 Sheikh Zayed Rd",
                "Mazaya Center"



            };

        //VRPTWs
        //An array of travel times between locations. Note that this differs from previous examples, which use a distance matrix. 
        // If all vehicles travel at the same speed, you will get the same solution if you use a distance matrix or a time matrix, since travel distances are a constant multiple of travel times.
        public long[,] TimeMatrix = {
            {0, 6, 9, 8, 7, 3, 6, 2, 3, 2, 6, 6, 4, 4, 5, 9, 7},
            {6, 0, 8, 3, 2, 6, 8, 4, 8, 8, 13, 7, 5, 8, 12, 10, 14},
            {9, 8, 0, 11, 10, 6, 3, 9, 5, 8, 4, 15, 14, 13, 9, 18, 9},
            {8, 3, 11, 0, 1, 7, 10, 6, 10, 10, 14, 6, 7, 9, 14, 6, 16},
            {7, 2, 10, 1, 0, 6, 9, 4, 8, 9, 13, 4, 6, 8, 12, 8, 14},
            {3, 6, 6, 7, 6, 0, 2, 3, 2, 2, 7, 9, 7, 7, 6, 12, 8},
            {6, 8, 3, 10, 9, 2, 0, 6, 2, 5, 4, 12, 10, 10, 6, 15, 5},
            {2, 4, 9, 6, 4, 3, 6, 0, 4, 4, 8, 5, 4, 3, 7, 8, 10},
            {3, 8, 5, 10, 8, 2, 2, 4, 0, 3, 4, 9, 8, 7, 3, 13, 6},
            {2, 8, 8, 10, 9, 2, 5, 4, 3, 0, 4, 6, 5, 4, 3, 9, 5},
            {6, 13, 4, 14, 13, 7, 4, 8, 4, 4, 0, 10, 9, 8, 4, 13, 4},
            {6, 7, 15, 6, 4, 9, 12, 5, 9, 6, 10, 0, 1, 3, 7, 3, 10},
            {4, 5, 14, 7, 6, 7, 10, 4, 8, 5, 9, 1, 0, 2, 6, 4, 8},
            {4, 8, 13, 9, 8, 7, 10, 3, 7, 4, 8, 3, 2, 0, 4, 5, 6},
            {5, 12, 9, 14, 12, 6, 6, 7, 3, 3, 4, 7, 6, 4, 0, 9, 2},
            {9, 10, 18, 6, 8, 12, 15, 8, 13, 9, 13, 3, 4, 5, 9, 0, 9},
            {7, 14, 9, 16, 14, 8, 5, 10, 6, 5, 4, 10, 8, 6, 2, 9, 0},
          };
        //An array of time windows for the locations, which you can think of as requested times for a visit. Vehicles must visit a location within its time window.
        public long[,] TimeWindows = {
                    {0, 5},    // depot
                    {7, 12},   // 1
                    {10, 15},  // 2
                    {16, 18},  // 3
                    {10, 13},  // 4
                    {0, 5},    // 5
                    {5, 10},   // 6
                    {0, 4},    // 7
                    {5, 10},   // 8
                    {0, 3},    // 9
                    {10, 16},  // 10
                    {10, 15},  // 11
                    {0, 5},    // 12
                    {5, 10},   // 13
                    {7, 8},    // 14
                    {10, 15},  // 15
                    {11, 15},  // 16
                  };

        //distance_matrix: An array of distances between locations on meters.
        public long[,] DistanceMatrix = {
                  {0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354, 468, 776, 662},
                  {548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674, 1016, 868, 1210},
                  {776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164, 1130, 788, 1552, 754},
                  {696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822, 1164, 560, 1358},
                  {582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708, 1050, 674, 1244},
                  {274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628, 514, 1050, 708},
                  {502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856, 514, 1278, 480},
                  {194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320, 662, 742, 856},
                  {308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662, 320, 1084, 514},
                  {194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388, 274, 810, 468},
                  {536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764, 730, 388, 1152, 354},
                  {502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114, 308, 650, 274, 844},
                  {388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194, 536, 388, 730},
                  {354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0, 342, 422, 536},
                  {468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536, 342, 0, 764, 194},
                  {776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274, 388, 422, 764, 0, 798},
                  {662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730, 536, 194, 798, 0}
                };

        //The number of vehicles in the fleet
        public int VehicleNumber = 4;

        //The index of the depot, the location where all vehicles start and end their routes.
        public int Depot = 0;

        //Capacity Constraints
        //public long[] Demands = { 0, 1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8 };
        public long[] VehicleCapacities = { 15, 15, 15, 15 };
        // For penalty
        public long[] Demands = { 0, 1, 1, 3, 6, 3, 6, 8, 8, 1, 2, 1, 2, 6, 6, 8, 8 };

        //Vehicle Routing with Pickups and Deliveries
        public int[][] PickupsDeliveries = {
      new int[] {1, 6},
      new int[] {2, 10},
      new int[] {4, 3},
      new int[] {5, 9},
      new int[] {7, 8},
      new int[] {15, 11},
      new int[] {13, 12},
      new int[] {16, 14},
    };

        // Resource Constraints
        public int VehicleLoadTime = 5;
        public int VehicleUnloadTime = 5;

        public int[] VehicleLoadTimes = { 1, 2, 15, 16 };
        public int[] VehicleUnloadTimes = { 5, 10, 15, 05, 10, 20, 10, 10 };

        public int DepotCapacity = 2;

        //VRP with initial Routes
        public long[][] InitialRoutes = {
          new long[] {8, 16, 14, 13, 12, 11},
          new long[] {3, 4, 9, 10},
          new long[] {15, 1},
          new long[] {7, 5, 2, 6},
      };

        //Setting start and end locations for routes // note: each for each vehical
        public int[] Starts = { 1, 2, 15, 16 };
        public int[] Ends = { 0, 0, 0, 0 };

    }
    //public class ORDataModel 
    //{
    //    // this cal

    //    public int[] ServiceTimes =
    //    {
    //            0,
    //            10,
    //            05,
    //            05,
    //            10,
    //            05,
    //            10,
    //            10
    //        };

    //    public string[] Destinations =
    //    {
    //            //Case 1
    //            "Dubai Investment Park-2",
    //            "Alphamed - Rd B1",
    //            "6 31 A St",
    //            "Unnamed Road",
    //            "15th St",
    //            "20 Sheikh Zayed Rd",
    //            "Mazaya Center"



    //        };

    //    //VRPTWs
    //    //An array of travel times between locations. Note that this differs from previous examples, which use a distance matrix. 
    //    // If all vehicles travel at the same speed, you will get the same solution if you use a distance matrix or a time matrix, since travel distances are a constant multiple of travel times.
    //    public long[,] TimeMatrix = {
    //        {0, 6, 9, 8, 7, 3, 6, 2, 3, 2, 6, 6, 4, 4, 5, 9, 7},
    //        {6, 0, 8, 3, 2, 6, 8, 4, 8, 8, 13, 7, 5, 8, 12, 10, 14},
    //        {9, 8, 0, 11, 10, 6, 3, 9, 5, 8, 4, 15, 14, 13, 9, 18, 9},
    //        {8, 3, 11, 0, 1, 7, 10, 6, 10, 10, 14, 6, 7, 9, 14, 6, 16},
    //        {7, 2, 10, 1, 0, 6, 9, 4, 8, 9, 13, 4, 6, 8, 12, 8, 14},
    //        {3, 6, 6, 7, 6, 0, 2, 3, 2, 2, 7, 9, 7, 7, 6, 12, 8},
    //        {6, 8, 3, 10, 9, 2, 0, 6, 2, 5, 4, 12, 10, 10, 6, 15, 5},
    //        {2, 4, 9, 6, 4, 3, 6, 0, 4, 4, 8, 5, 4, 3, 7, 8, 10},
    //        {3, 8, 5, 10, 8, 2, 2, 4, 0, 3, 4, 9, 8, 7, 3, 13, 6},
    //        {2, 8, 8, 10, 9, 2, 5, 4, 3, 0, 4, 6, 5, 4, 3, 9, 5},
    //        {6, 13, 4, 14, 13, 7, 4, 8, 4, 4, 0, 10, 9, 8, 4, 13, 4},
    //        {6, 7, 15, 6, 4, 9, 12, 5, 9, 6, 10, 0, 1, 3, 7, 3, 10},
    //        {4, 5, 14, 7, 6, 7, 10, 4, 8, 5, 9, 1, 0, 2, 6, 4, 8},
    //        {4, 8, 13, 9, 8, 7, 10, 3, 7, 4, 8, 3, 2, 0, 4, 5, 6},
    //        {5, 12, 9, 14, 12, 6, 6, 7, 3, 3, 4, 7, 6, 4, 0, 9, 2},
    //        {9, 10, 18, 6, 8, 12, 15, 8, 13, 9, 13, 3, 4, 5, 9, 0, 9},
    //        {7, 14, 9, 16, 14, 8, 5, 10, 6, 5, 4, 10, 8, 6, 2, 9, 0},
    //      };
    //    //An array of time windows for the locations, which you can think of as requested times for a visit. Vehicles must visit a location within its time window.
    //    public long[,] TimeWindows = {
    //                {0, 5},    // depot
    //                {7, 12},   // 1
    //                {10, 15},  // 2
    //                {16, 18},  // 3
    //                {10, 13},  // 4
    //                {0, 5},    // 5
    //                {5, 10},   // 6
    //                {0, 4},    // 7
    //                {5, 10},   // 8
    //                {0, 3},    // 9
    //                {10, 16},  // 10
    //                {10, 15},  // 11
    //                {0, 5},    // 12
    //                {5, 10},   // 13
    //                {7, 8},    // 14
    //                {10, 15},  // 15
    //                {11, 15},  // 16
    //              };

    //    //distance_matrix: An array of distances between locations on meters.
    //    public long[,] DistanceMatrix = {
    //              {0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354, 468, 776, 662},
    //              {548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674, 1016, 868, 1210},
    //              {776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164, 1130, 788, 1552, 754},
    //              {696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822, 1164, 560, 1358},
    //              {582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708, 1050, 674, 1244},
    //              {274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628, 514, 1050, 708},
    //              {502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856, 514, 1278, 480},
    //              {194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320, 662, 742, 856},
    //              {308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662, 320, 1084, 514},
    //              {194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388, 274, 810, 468},
    //              {536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764, 730, 388, 1152, 354},
    //              {502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114, 308, 650, 274, 844},
    //              {388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194, 536, 388, 730},
    //              {354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0, 342, 422, 536},
    //              {468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536, 342, 0, 764, 194},
    //              {776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274, 388, 422, 764, 0, 798},
    //              {662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730, 536, 194, 798, 0}
    //            };

    //    //The number of vehicles in the fleet
    //    public int VehicleNumber =  4;

    //    //The index of the depot, the location where all vehicles start and end their routes.
    //    public int Depot = 0;

    //    //Capacity Constraints
    //    //public long[] Demands = { 0, 1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8 };
    //    public long[] VehicleCapacities = { 15, 15, 15, 15 };
    //    // For penalty
    //    public long[] Demands = { 0, 1, 1, 3, 6, 3, 6, 8, 8, 1, 2, 1, 2, 6, 6, 8, 8 };

    //    //Vehicle Routing with Pickups and Deliveries
    //    public int[][] PickupsDeliveries = {
    //  new int[] {1, 6},
    //  new int[] {2, 10},
    //  new int[] {4, 3},
    //  new int[] {5, 9},
    //  new int[] {7, 8},
    //  new int[] {15, 11},
    //  new int[] {13, 12},
    //  new int[] {16, 14},
    //};

    //    // Resource Constraints
    //    public int VehicleLoadTime = 5;
    //    public int VehicleUnloadTime = 5;

    //    public int[] VehicleLoadTimes = { 1, 2, 15, 16 };
    //    public int[] VehicleUnloadTimes = { 5, 10, 15, 05,10, 20, 10, 10 };

    //    public int DepotCapacity = 2;

    //    //VRP with initial Routes
    //    public long[][] InitialRoutes = {
    //      new long[] {8, 16, 14, 13, 12, 11},
    //      new long[] {3, 4, 9, 10},
    //      new long[] {15, 1},
    //      new long[] {7, 5, 2, 6},
    //  };

    //    //Setting start and end locations for routes // note: each for each vehical
    //    public int[] Starts = { 1, 2, 15, 16 };
    //    public int[] Ends = { 0, 0, 0, 0 };

    //}
}
