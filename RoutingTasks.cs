using System;
using System.Collections.Generic;
using System.Text;
using Google.OrTools.ConstraintSolver;
using Google.Protobuf.WellKnownTypes; // Duration
using GoogleApi.Entities.Maps.Common.Enums;
using GoogleApi.Entities.Maps.Directions.Response;
using GoogleApi.Entities.Maps.DistanceMatrix.Request;
using GoogleApi.Entities.Maps.DistanceMatrix.Response;

namespace ConsoleApp_ORtool
{
    class RoutingTasks
    {
        void TimeLimits() 
        {
            RoutingSearchParameters searchParameters = operations_research_constraint_solver.DefaultRoutingSearchParameters();
            searchParameters.TimeLimit = new Duration { Seconds = 10 };
        }
        void SolutionLimits()
        {
            RoutingSearchParameters searchParameters = operations_research_constraint_solver.DefaultRoutingSearchParameters();
            searchParameters.SolutionLimit=100;
        }
        //Setting initial routes for a search Note: The initial routes do not include the depot.
        public long[][] InitialRoutes = {
              new long[] {8, 16, 14, 13, 12, 11},
              new long[] {3, 4, 9, 10},
              new long[] {15, 1},
              new long[] {7, 5, 2, 6},
          };
        void initialSolutions()
        {
            //Assignment initialSolution = routing.ReadAssignmentFromRoutes(data.InitialRoutes, true);
            // Print initial solution on console.
            Console.WriteLine("Initial solution:");
            //PrintSolution(data, routing, manager, initialSolution); ;
        }

        //Setting start and end locations for routes
        public int[] Starts = { 1, 2, 15, 16 };
        public int[] Ends = { 0, 0, 0, 0 };

        //https://developers.google.com/optimization/routing/routing_options
        //https://developers.google.com/optimization/routing/google_direction

        /*Using the Google Distance Matrix API
The section shows how to use the Google Distance Matrix API to create the distance matrix for any set of locations defined by addresses, or by latitudes and longitudes. You can use the API to calculate the distance matrix for many types of routing problems.

To use the API, you'll need an API key. Here's how to obtain one. https://developers.google.com/optimization/routing/vrp#distance_matrix_api
Note that the location coordinates are not included in the problem data: all you need to solve the problem is the distance matrix, which we have pre-computed. You only need the location data to identify the locations in the solution, which are denoted by their indices (0, 1, 2 ...) in the above list.

The main purpose of showing the location coordinates and the city diagram in this and other examples is to provide a visual display of the problem and its solution. But this is not essential for solving a VRP.

For convenience in setting up the problem, the distances between locations are calculated using Manhattan distance, in which the distance between two points, (x1, y1) and (x2, y2) is defined to be |x1 - x2| + |y1 - y2|. However, there is no special reason to use this definition. You can use whatever method is best suited to your problem to calculate distances. Or, you can obtain a distance matrix for any set of locations in the world using the Google Distance Matrix API. See Distance Matrix API for an example of how to do this.
 */
        //long[,] create_distance_matrix(ORDataModel data) {
        //    string[] addresses =["New York, NY, USA"];// data["addresses"];
        //    string API_key = "";//data["API_key"]
        //                        //# Distance Matrix API only accepts 100 elements per request, so get rows in multiple requests.
        //    int max_elements = 100;
        //    int num_addresses = data.DistanceMatrix.GetLength(0);// len(addresses) # 16 in this example.
        //                                                         //# Maximum number of rows that can be computed per request (6 in this example).
        //    int max_rows = max_elements; // num_addresses
        ////# num_addresses = q * max_rows + r (q = 2 and r = 4 in this example).
        ////q, r = divmod(num_addresses, max_rows)
        // string dest_addresses = addresses;
        //    long[,] distance_matrix ;
        //    // # Send q requests, returning max_rows rows per request.
        //    foreach (var item in collection)
        //    {
        //        for i in range(q) :
        //    origin_addresses = addresses[i * max_rows: (i + 1) * max_rows]
        //    response = send_request(origin_addresses, dest_addresses, API_key)
        //    distance_matrix += build_distance_matrix(response)

        //  //# Get the remaining remaining r rows, if necessary.
        //            if r > 0:
        //    origin_addresses = addresses[q * max_rows: q * max_rows + r]
        //    response = send_request(origin_addresses, dest_addresses, API_key)
        //    distance_matrix += build_distance_matrix(response)
        //    }

        //    return distance_matrix;
        //}
        public void DrivingDistancebyAddress()
        {
            DistanceMatrixRequest request = new DistanceMatrixRequest();
            //sheffield
            request.DestinationsRaw = "Sheffield|Sheffield";//.AddDestination(new Location("Sheffield"));
            //rotherham
            request.OriginsRaw="Rotherham";

            request.TrafficModel = TrafficModel.Best_Guess;
            request.TravelMode = TravelMode.Driving;
            request.DepartureTime = null;


         //   DistanceMatrixResponse response =response... GetResponse(request);

            //Assert.IsTrue(response.Status == ServiceResponseStatus.Ok);
        }

        //private void Calculate()
        //{
        //    var req = new DistanceMatrixRequest();
        //    req.Origins=_start.ToString();
        //    req.AddDestination(new Waypoint() { Address = _stop.ToString() });
        //    req.Sensor = false;
        //    req.Mode = TravelMode.driving;
        //    req.Units = Units.imperial;

        //    var response = new DistanceMatrixService().GetResponse(req);

        //    Distance = Convert.ToInt32(response.Rows.First().Elements.First().distance.Value);
        //    Duration = Convert.ToInt32(response.Rows.First().Elements.First().duration.Value);

        //    //Console.WriteLine("Distance from {0}, to {1} is {2} - Travel Time: {3}", _start, _stop, Distance, Duration);
        //}
    }
}
