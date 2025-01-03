#include "application.h"
#include "json.hpp"
#include <fstream>

#include <iostream>
#include <limits>
#include <map>
#include <queue>  // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"
using namespace std;

class prioritize {
  public:
    bool operator()(const pair<long long, double>& p1, const pair<long long, double>& p2) const {
      return p1.second > p2.second;
    }
};

double INF = numeric_limits<double>::max();
using json = nlohmann::json;

void buildGraph(istream& input, graph<long long, double>& G, vector<BuildingInfo>& buildings) {
  json data;
  input >> data;

  unordered_map<long long, Coordinates> nodes;

  // Load waypoints
  for (const auto& waypoint : data["waypoints"]) {
    long long id = waypoint["id"];
    Coordinates location = { waypoint["lat"], waypoint["lon"] };
    nodes.emplace(id, location);
    G.addVertex(id);
  }

  // Load buildings
  for (const auto& building : data["buildings"]) {
    BuildingInfo b;
    b.id = building["id"];
    b.location = { building["lat"], building["lon"] };
    b.name = building["name"];
    b.abbr = building["abbr"];
    buildings.push_back(b);
    G.addVertex(b.id);
  }

  // Load footways and create edges
  for (const auto& footway : data["footways"]) {
    long long prev = -1;
    for (const auto& node : footway) {
      long long id = node;
      if (prev != -1) {
        double distance = distBetween2Points(nodes[prev], nodes[id]);
        G.addEdge(prev, id, distance);   
        G.addEdge(id, prev, distance);
      }
      prev = id;
    }
  }

  // Connect buildings to nearby waypoints
  for (const auto& building : buildings) {
    double minDist = INF;
    //long long closest = 0;

    for (const auto& [id, location] : nodes) {
      double distance = distBetween2Points(building.location, location);

      if (distance < minDist) {
        //closest = id;
        minDist = distance;
      }

      if (distance > 0 && distance < 0.036) {
        G.addEdge(building.id, id, distance);
        G.addEdge(id, building.id, distance);
      }
    }
  }
}

BuildingInfo getBuildingInfo(const vector<BuildingInfo>& buildings,
                             const string& query) {
  for (const BuildingInfo& building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo>& buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo& building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}



vector<long long> dijkstra(const graph<long long, double>& G, long long start, long long target, const set<long long>& ignoreNodes) {
  priority_queue<pair<double, long long>, vector<pair<double, long long>>, prioritize> worklist;
  map<long long, double> distances;
  map<long long, long long> storeVertices;

  for(auto vertice: G.getVertices()){
    distances[vertice] = INF;
  }
  distances[start] = 0.0;
  worklist.push({0.0, start});

  while(!worklist.empty()) {
    auto [currentDist, currentVertex] = worklist.top();
    worklist.pop();

    if (ignoreNodes.find(currentVertex) != ignoreNodes.end() && currentVertex != start && currentVertex != target) {
      continue;
    }

    for (const auto& neighbor : G.neighbors(currentVertex)) {
      if (ignoreNodes.find(neighbor) != ignoreNodes.end() && neighbor != start && neighbor != target) {
        continue;
      }

      double neighborWeight = G.getWeightValue(currentVertex, neighbor);
      double newDist = currentDist + neighborWeight;
      if (newDist < distances[neighbor]) {
        distances[neighbor] = newDist;
        storeVertices[neighbor] = currentVertex;
        worklist.push({newDist, neighbor});
      }
    }
  }

  vector<long long> Finalpath;
  if (distances[target] == INF) {
    return {}; 
  }

  for (long long at = target; at != start; at = storeVertices[at]) {
    Finalpath.push_back(at);
  }
  Finalpath.push_back(start);
  reverse(Finalpath.begin(), Finalpath.end()); //<algorithm> function, reverses elements can I use this?

  return Finalpath;
}

double pathLength(const graph<long long, double>& G,
                  const vector<long long>& path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long>& path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

void application(const vector<BuildingInfo>& buildings,
                 const graph<long long, double>& G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto& building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
