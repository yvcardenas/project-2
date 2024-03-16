#include "Graph.h"
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <set>
#include <iostream>
#include <algorithm>
#include <limits>
#include <utility>
#include <stack>

Graph::Graph(const char* const & edgelist_csv_fn) {
    // Create ifstream name file with the provided filename from parameter
    ifstream file(edgelist_csv_fn);
    string line;

    // Reads each line from the CSV file using getLine(). Each line = edge
    while(getline(file, line)){
        // Parse the line
        stringstream ss(line);

        // Labels of two nodes and the weight of the edge that connects the two
        string u_label, v_label;
        double weight;

        // Extract the two nodes from the stream, seperated by a comma
        getline(ss, u_label, ',');
        getline(ss, v_label, ',');
        // Extract weight directly
        ss >> weight;

        //Check is the nodes already exist in the nodes_set
        //If insert into set and create entry in adjacency_list
        if(nodes_set.find(u_label) == nodes_set.end()){
            nodes_set.insert(u_label);
            adjacency_list[u_label] = unordered_map<string, double>();
        }
        if(nodes_set.find(v_label) == nodes_set.end()){
            nodes_set.insert(v_label);
            adjacency_list[v_label] = unordered_map<string, double>();
        }

        // Weight of the edge is added to the adjacency_list
        adjacency_list[u_label][v_label] = weight;
        adjacency_list[v_label][u_label] = weight;
    }
}

unsigned int Graph::num_nodes() {
    //Returns the size of the nodes_set
    return nodes_set.size();
}

vector<string> Graph::nodes() {
    // Creates vector to store the nodes
    vector<string> nodes_vec;
    // For every node in the set, push onto the vector
    for(const auto& node : nodes_set){
        nodes_vec.push_back(node);
    }
    // Return vector with all the nodes
    return nodes_vec;
}

unsigned int Graph::num_edges() {
    // Declare variable that represents the total edges in the graph
    unsigned int total_edges = 0;
    for(const auto& neighbors : adjacency_list){
        total_edges += neighbors.second.size();
    }
    return (total_edges / 2); // Divide by 2 because edges are undirected
}

unsigned int Graph::num_neighbors(string const & node_label) {
    if(adjacency_list.find(node_label) != adjacency_list.end()){
        return adjacency_list[node_label].size();
    }
    return 0;
}

double Graph::edge_weight(string const & u_label, string const & v_label) {
    if(adjacency_list.find(u_label) != adjacency_list.end()){
        if(adjacency_list[u_label].find(v_label) != adjacency_list[u_label].end()){
            return adjacency_list[u_label][v_label];
        }
    }
    return -1; // Edge not found
}

vector<string> Graph::neighbors(string const & node_label) {
    // Declare vector that will hold all the labels of the neighbors of the given node in parameter
    vector<string> neighbors_vec;
    if(adjacency_list.find(node_label) != adjacency_list.end()){
        for(const auto& neighbor : adjacency_list[node_label]){
            neighbors_vec.push_back(neighbor.first);
        }
    }
    return neighbors_vec;
}

vector<string> Graph::shortest_path_unweighted(string const & start_label, string const & end_label) {
    // Initialize a queue for BFS and a set to track visited nodes
    queue<string> q;
    unordered_set<string> visited;
    unordered_map<string, string> parent;

    // Start BFS from the start node
    q.push(start_label);
    visited.insert(start_label);
    parent[start_label] = start_label;
    
    // Perform BFS until the queue is empty
    while(!q.empty()){
        string current = q.front();
        q.pop();

        // If the target node is found, construct and return the path
        if(current == end_label){
            vector<string> path;
            while(current != start_label){
                path.push_back(current);
                current = parent[current];
            }
            path.push_back(start_label);
            reverse(path.begin(), path.end());
            return path;
        }

        // Explore neighbors of the current node
        for(const auto& neighbor : adjacency_list[current]){
            string next = neighbor.first;
            if(visited.find(next) == visited.end()){
                q.push(next);
                visited.insert(next);
                parent[next] = current;
            }
        }
    }

    // If the target node is unreachable, return an empty path
    return vector<string>();
}

vector<tuple<string,string,double>> Graph::shortest_path_weighted(string const & start_label, string const & end_label) {   
    return vector<tuple<string,string,double>>();
}

vector<vector<string>> Graph::connected_components(double const & threshold) {
    unordered_map<string, bool> visited;
    vector<vector<string>> components;

    for(const auto& node : nodes_set){
        if(!visited[node]){
            vector<string> component;
            queue<string> q;

            q.push(node);
            visited[node] = true;

            while(!q.empty()){
                string current = q.front();
                q.pop();
                component.push_back(current);

                for(const auto& neighbor : adjacency_list[current]){
                    string next = neighbor.first;
                    if(!visited[next] && neighbor.second <= threshold){
                        q.push(next);
                        visited[next] = true;
                    }
                }
            }
            components.push_back(component);
        }
    }    
    return components;
}

bool Graph::can_reach_with_threshold(string const & start_label, string const & end_label, double threshold, unordered_set<string>& visited){
    visited.insert(start_label);

    if(start_label == end_label){
          return true;
    }

    for(const auto &neighbor : adjacency_list[start_label]){
        string next_label = neighbor.first;
        double edge_weight = neighbor.second;

        if(visited.find(next_label) == visited.end() && edge_weight <= threshold){
            if(can_reach_with_threshold(next_label, end_label, threshold, visited)){
                return true;
            }
        }
    }
    return false;
}

double Graph::smallest_connecting_threshold(string const & start_label, string const & end_label) {
    if(start_label == end_label){
        return 0.0;
    }
    unordered_set<string> visited;

    //Binary search for the smallest threshold
    double low = 0.0, high = numeric_limits<double>::max();
    while(low <= high){
        double mid = low + (high - low)/2;
        visited.clear();
        if(can_reach_with_threshold(start_label, end_label, mid, visited)){
            high = mid;
        }else{
            low = mid + numeric_limits<double>::epsilon();
        }
    }
    return high < numeric_limits<double>::max() ? high : -1.0;
}
