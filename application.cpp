// Filename: application.cpp 
// Author: Viraj Saudagar
// University of Illinois Chicago
// CS 251, Fall 2023
//
// This program implements an application which generates a graph using the defined graph class 
// on a given map file in the osm format. The program parses the input map file and generates a graph 
// populated with vertices and edges representing paths and points on the map with distance edges. The application then calculates 
// the shortest pathways for two people to meet from valid buildings at a valid building. 
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <queue>
#include <stack>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"


using namespace std;
using namespace tinyxml2;

const double INF = numeric_limits<double>::max();

class prioritize  
{
public:
  bool operator()(const pair<long long,double>& p1, const pair<long long, double>& p2) const
  {
    return p1.second > p2.second; 
  }
};


// Searches vector of buildings building abbreviation first if query exists & returns BuildingInfo if found.
// If query is not an abbreviation, it searches for the substring where it exists and returns that BuildingInfo. 
// If nothing is found "empty" denoted with abbrev -1 is returned. 
BuildingInfo searchBuilding(string query, vector<BuildingInfo> Buildings){

  BuildingInfo x;
  x.Abbrev = "-1";

  for(int i = 0; i < Buildings.size(); i++){

    if(Buildings.at(i).Abbrev == query){
      return Buildings.at(i);
    }

  }

  for(int i = 0; i < Buildings.size(); i++){

    if(Buildings.at(i).Fullname.find(query) != string::npos){
      return Buildings.at(i);
    }

  }

  return x;

}

/*
    Given a set of strings encoding the unreachable buildings, returns true if a given 
    query is unreachable & false if it is not.
*/
bool isUnreachable(set<string> unreachableBuildings, string query){

    set<string>::iterator i;

    for(i = unreachableBuildings.begin(); i != unreachableBuildings.end(); i++){

        if(*i == query){
            return true;
        }

    }

    return false;

}

/*
    Given a Coordinate which contains Lat and Lon values, returns the minmum distance building to that point
    and updates the index passed by reference to the index at which the building is at. Ensures that 
    the building is not an unreachable building as well. 
*/
double minDistanceToMidPoint(vector<BuildingInfo> Buildings, Coordinates midpoint, int& index, set<string> unreachableBuildings){

  double min = INF;

  for(int i = 0; i < Buildings.size(); i++){

    double lat1 = midpoint.Lat;
    double lon1 = midpoint.Lon;
    double lat2 = Buildings.at(i).Coords.Lat;
    double lon2 = Buildings.at(i).Coords.Lon;

    double distance = distBetween2Points(lat1, lon1, lat2, lon2);
    string name = Buildings.at(i).Fullname;

    if(distance < min && !isUnreachable(unreachableBuildings, name)){
      index = i;
      min = distance;
    }

  }

  return min;

}

/*
    Returns the nearest node on a footway found to a given building. 
*/
long long findNearestNode(vector<FootwayInfo> Footways, BuildingInfo building, map<long long, Coordinates> nodeMap){

  double minDist = INF;
  long long nodeId = -1;

  double lat1 = building.Coords.Lat;
  double lon1 = building.Coords.Lon;

  for(int i = 0; i < Footways.size(); i++){

    for(int x = 0; x < Footways.at(i).Nodes.size(); x++){

      double lat2 = (nodeMap.at(Footways.at(i).Nodes.at(x))).Lat;
      double lon2 = (nodeMap.at(Footways.at(i).Nodes.at(x))).Lon;

      double distance = distBetween2Points(lat1, lon1, lat2, lon2);

      if(distance < minDist){
        minDist = distance;
        nodeId = Footways.at(i).Nodes.at(x);
      }

    }

  }

  return nodeId;

}

/*
    Used in dijkstra function to determine if a query has already been visited (is in the visited container). 
    Returns true if it has been visited, false if not. 
*/
bool visitedCurrV(vector<long long> visited, long long query){

  for(int i = 0; i < visited.size(); i++){
    if(visited.at(i) == query){
      return true;
    }
  }

  return false;

}


/*
    Implements dijkstras algorithm to determine the shortest weighted pathways to other vertex given a start vertex. 
    Updates a predecessor and distance map that were passed by reference with the updated predecessors and distance to each node 
    for the given startVertex. 
*/
void DijkstraAlgorithm(long long startVertex, map<long long, Coordinates> Nodes, graph<long long, double> G, map<long long, long long>& predecessors, map<long long, double>& distances){

  // Priority Queue 
  priority_queue<pair<long long, double>, vector<pair<long long, double> >, prioritize> unvisitedQueue;
  // Visited 
  vector<long long> visited;

  vector<long long> vertices = G.getVertices();

  for(int i = 0; i < vertices.size(); i++){

    distances[vertices.at(i)] = INF;
    predecessors[vertices.at(i)] = 0;
    unvisitedQueue.push(make_pair(vertices.at(i), INF));

  }

  distances[startVertex] = 0;
  unvisitedQueue.push(make_pair(startVertex, 0));

  while(!unvisitedQueue.empty()){

    long long currentV = unvisitedQueue.top().first;
    unvisitedQueue.pop();

    if(distances[currentV] == INF){
      break;
    }
    else if(visitedCurrV(visited, currentV)){
      continue;
    }
    else{

      visited.push_back(currentV);
      set<long long> neighbors = G.neighbors(currentV);

      set<long long>::iterator i;

      for(i = neighbors.begin(); i != neighbors.end(); i++){

        long long nodeId = *i;
        double weight;

        G.getWeight(currentV, nodeId, weight);

        double altPathDistance = distances[currentV] + weight;

        if(altPathDistance < distances[nodeId]){
          distances[nodeId] = altPathDistance;
          predecessors[nodeId] = currentV;
          unvisitedQueue.push(make_pair(nodeId, altPathDistance));
        }

      }

    }

  }


}

/*
    Decodes a map of predecessors to extract the path from startVertex to the endVertex 
    using a stack. Returns a vector where the elements are in order of the path. 
*/
vector<long long> getPath(map<long long, long long> predecessors, long long endVertex){

  long long currV = endVertex;
  stack<long long> S;
  vector<long long> sol;

  while(currV != 0){
    S.push(currV);
    currV = predecessors[currV];
  }

  while(!S.empty()){
    currV = S.top();
    S.pop();
    sol.push_back(currV);
  }

  return sol;

}

/*
    Driver of the program -> gets user input for building1 & building2 and calculates the middle
    meeting building for both persons (buildings). Then runs dijkstras algorithm on the nearest nodes to each
    person (their building) and determines if a path exists between the two nodes and the most efficient path 
    if it does exist. 

*/
void application(
    map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
  string person1Building, person2Building;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);
    bool pathFound = false;

    // TODO MILESTONE 7: Search Buildings 1 and 2
    BuildingInfo person1 = searchBuilding(person1Building, Buildings);
    BuildingInfo person2 = searchBuilding(person2Building, Buildings);

    if(person1.Abbrev == "-1"){
      cout << "Person 1's building not found" << endl;
      pathFound = true;
    }
    else if(person2.Abbrev == "-1"){
      cout << "Person 2's building not found" << endl;
      pathFound = true;
    }

    bool first = true;      
    set<string> unreachableBuildings;    

    while(!pathFound){

      // TODO MILESTONE 8: Locate Center Building
      double lat1 = person1.Coords.Lat;
      double lon1 = person1.Coords.Lon;
      double lat2 = person2.Coords.Lat;
      double lon2 = person2.Coords.Lon;

      int buildingIndex = -1;

      Coordinates midpoint = centerBetween2Points(lat1, lon1, lat2, lon2);
      double minDistance = minDistanceToMidPoint(Buildings, midpoint, buildingIndex, unreachableBuildings);

      BuildingInfo destinationBuilding = Buildings.at(buildingIndex);

      // TODO MILESTONE 9: Find Nearest Nodes from buildings 1, 2 & Center
      long long person1NearestNode = findNearestNode(Footways, person1, Nodes);
      long long  person2NearestNode = findNearestNode(Footways, person2, Nodes);
      long long  destNearestNode = findNearestNode(Footways, destinationBuilding, Nodes);

      // Outputs for First Iteration
      if(first){
        
        cout << "Person 1's point:" << endl;
        cout << " " << person1.Fullname << endl;
        cout << " (" << person1.Coords.Lat << ", " << person1.Coords.Lon << ")" << endl;

        cout << "Person 2's point:" << endl;
        cout << " " << person2.Fullname << endl;
        cout << " (" << person2.Coords.Lat << ", " << person2.Coords.Lon << ")" << endl;

        cout << "Destination Building:" << endl;
        cout << " " << destinationBuilding.Fullname << endl;
        cout << " (" << destinationBuilding.Coords.Lat << ", " << destinationBuilding.Coords.Lon << ")" << endl;

        cout << endl;

        cout << "Nearest P1 node:" << endl;
        cout << " " << person1NearestNode << endl;
        cout << " (" << Nodes.at(person1NearestNode).Lat << ", " << Nodes.at(person1NearestNode).Lon << ")" << endl;

        cout << "Nearest P2 node:" << endl;
        cout << " " << person2NearestNode << endl;
        cout << " (" << Nodes.at(person2NearestNode).Lat << ", " << Nodes.at(person2NearestNode).Lon << ")" << endl;

        cout << "Nearest destination node:" << endl;
        cout << " " << destNearestNode << endl;
        cout << " (" << Nodes.at(destNearestNode).Lat << ", " << Nodes.at(destNearestNode).Lon << ")" << endl;

      }
      else{

        cout << "New destination building:" << endl;
        cout << " " << destinationBuilding.Fullname << endl;
        cout << " (" << destinationBuilding.Coords.Lat << ", " << destinationBuilding.Coords.Lon << ")" << endl;

        cout << "Nearest destination node:" << endl;
        cout << " " << destNearestNode << endl;
        cout << " (" << Nodes.at(destNearestNode).Lat << ", " << Nodes.at(destNearestNode).Lon << ")" << endl;

      }

      // TODO MILESTONE 10: Run Dijkstra’s Algorithm
      map<long long, long long> predecessors;
      map<long long, double> distances; 
      vector<long long> path;

      DijkstraAlgorithm(person1NearestNode, Nodes, G, predecessors, distances);
      path = getPath(predecessors, destNearestNode);

      map<long long, long long> predecessors2;
      map<long long, double> distances2; 
      vector<long long> path2;
    
      DijkstraAlgorithm(person2NearestNode, Nodes, G, predecessors2, distances2);
      path2 = getPath(predecessors2, destNearestNode);

      // No Path Found Checks
      if(distances[person2NearestNode] >= INF){
        cout << "Sorry, destination unreachable." << endl;
        // clear the unreachable buildings as we are starting over with 2 new buildings.
        unreachableBuildings.clear();
        first = true;
        break;
      }
      else if(distances[destNearestNode] >= INF || distances2[destNearestNode] >= INF){
        cout << "At least one person was unable to reach the destination building. Finding next closest building..." << endl;
        first = false;
      }
      else{
        // TODO MILESTONE 11: Print path (path found! break)
        
        cout << "Person 1's distance to dest: " << distances[destNearestNode] << " miles" << endl;
        cout << "Path: ";

        for(int i = 0; i < path.size(); i++){

          if(i != path.size()-1){
            cout << path.at(i) << "->";
          }
          else{
            cout << path.at(i);
          }

        }

        cout << endl;

        cout << "Person 2's distance to dest: " << distances2[destNearestNode] << " miles" << endl;
        cout << "Path: ";

        for(int i = 0; i < path2.size(); i++){

          if(i != path2.size()-1){
            cout << path2.at(i) << "->";
          }
          else{
            cout << path2.at(i);
          }

        }

        cout << endl;

        break;
      }

      // TODO MILESTONE 11: Find Second Nearest Destination (loop again)
      unreachableBuildings.insert(destinationBuilding.Fullname);

    }
    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }    
}

int main() {
  graph<long long, double> G;

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl; 
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;


  // TODO MILESTONE 5: Add Vertices 
  for(auto i : Nodes){

    G.addVertex(i.first);

  }

  // TODO MILESTONE 6: Add Edges
  for(int i = 0; i < Footways.size(); i++){

    for(int x = 0; x < Footways.at(i).Nodes.size()-1; x++){

      double lat1 = Nodes.at(Footways.at(i).Nodes.at(x)).Lat;
      double long1 = Nodes.at(Footways.at(i).Nodes.at(x)).Lon;
      double lat2 =  Nodes.at(Footways.at(i).Nodes.at(x+1)).Lat;
      double long2 = Nodes.at(Footways.at(i).Nodes.at(x+1)).Lon;

      double distance = distBetween2Points(lat1, long1, lat2, long2);

      G.addEdge(Footways.at(i).Nodes.at(x), Footways.at(i).Nodes.at(x+1), distance);
      G.addEdge(Footways.at(i).Nodes.at(x+1), Footways.at(i).Nodes.at(x), distance);

    }

  }

  // TODO: Uncomment below after MILESTONE 6
  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Menu
  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
