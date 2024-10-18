// Online C++ compiler to run C++ program online
#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <limits>
#include <set>
#include <numeric>

class Graph {
private:
    int V;
    std::vector<std::vector<std::pair<int, int>>> adj;  // {vertex, weight}
    std::vector<std::vector<int>> unweighted_adj;

public:
    Graph(int v) : V(v), adj(v), unweighted_adj(v) {}

    void addEdge(int u, int v, int w) {
        adj[u].push_back({v, w});
        adj[v].push_back({u, w});
        unweighted_adj[u].push_back(v);
        unweighted_adj[v].push_back(u);
    }

    void printGraph() const {
        for (int i = 0; i < V; ++i) {
            std::cout << i << ": ";
            for (const auto& [v, w] : adj[i]) {
                std::cout << "(" << v << ", " << w << ") ";
            }
            std::cout << std::endl;
        }
    }

    bool isEulerGraph() const {
        for (const auto& neighbors : unweighted_adj) {
            if (neighbors.size() % 2 != 0) {
                return false;
            }
        }
        return true;
    }

    std::vector<int> fleuryAlgorithm() {
        if (!isEulerGraph()) {
            return {};
        }

        std::vector<std::vector<int>> adj_copy = unweighted_adj;
        std::vector<int> circuit;
        int start = 0;

        auto dfs = [&](auto&& self, int u) -> void {
            for (int v : adj_copy[u]) {
                if (v != -1) {
                    adj_copy[u].erase(std::remove(adj_copy[u].begin(), adj_copy[u].end(), v), adj_copy[u].end());
                    adj_copy[v].erase(std::remove(adj_copy[v].begin(), adj_copy[v].end(), u), adj_copy[v].end());
                    self(self, v);
                }
            }
            circuit.push_back(u);
        };

        dfs(dfs, start);
        std::reverse(circuit.begin(), circuit.end());
        return circuit;
    }

    std::vector<int> dijkstra(int start) {
        std::vector<int> dist(V, std::numeric_limits<int>::max());
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;

        dist[start] = 0;
        pq.push({0, start});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            for (const auto& [v, w] : adj[u]) {
                if (dist[u] + w < dist[v]) {
                    dist[v] = dist[u] + w;
                    pq.push({dist[v], v});
                }
            }
        }

        return dist;
    }

    std::vector<std::pair<int, int>> primMST() {
        std::vector<bool> inMST(V, false);
        std::vector<int> key(V, std::numeric_limits<int>::max());
        std::vector<int> parent(V, -1);
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;

        key[0] = 0;
        pq.push({0, 0});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            inMST[u] = true;

            for (const auto& [v, w] : adj[u]) {
                if (!inMST[v] && w < key[v]) {
                    parent[v] = u;
                    key[v] = w;
                    pq.push({key[v], v});
                }
            }
        }

        std::vector<std::pair<int, int>> mst;
        for (int i = 1; i < V; ++i) {
            mst.push_back({parent[i], i});
        }
        return mst;
    }

    // Fundamental Cutsets
    std::vector<std::vector<int>> fundamentalCutsets(const std::vector<std::pair<int, int>>& mst) {
        std::vector<std::vector<int>> cutsets;

        for (const auto& [u, v] : mst) {
            // Temporarily remove edge u-v
            // Perform DFS/BFS to find reachable nodes from u
            // The nodes not reachable from u are part of the cutset
            std::vector<bool> visited(V, false);
            
            auto dfs = [&](auto&& self, int x) -> void {
                visited[x] = true;
                for (const auto& [y, w] : adj[x]) {
                    if (!visited[y] && !(x == u && y == v) && !(x == v && y == u)) {
                        self(self, y);
                    }
                }
            };

            dfs(dfs, u);

            // Collect nodes not visited from u
            std::vector<int> cutset;
            for (int i = 0; i < V; ++i) {
                if (!visited[i]) cutset.push_back(i);
            }
            
            cutsets.push_back(cutset);
        }
        
        return cutsets;
    }

    // Fundamental Circuits
    std::vector<std::vector<int>> fundamentalCircuits(const std::vector<std::pair<int, int>>& mst) {
        std::vector<std::vector<int>> circuits;

        // Iterate through each edge in the MST
        for (const auto& [u, v] : mst) {
            // Create a path from u to v using DFS
            // Add the edge u-v back to the graph temporarily
            addEdge(u,v,std::numeric_limits<int>::max()); // Temporarily add edge

            // Find a circuit using DFS starting from u
            std::set<int> visited; 
            
            auto dfs = [&](auto&& self, int x) -> void { 
                visited.insert(x); 
                for(const auto& [y,w]:adj[x]){ 
                    if(visited.find(y)==visited.end()){ 
                        self(self,y); 
                    } 
                } 
                circuits.back().push_back(x); 
             };

             circuits.push_back(std :: vector<int>{}); // Start new circuit with node u
             dfs(dfs,u); // Start DFS from u

             removeEdge(u,v); // Remove edge after finding circuit
         }
         return circuits; 
     }

     void removeEdge(int u,int v){ 
         for(auto it=adj[u].begin(); it!=adj[u].end();){ 
             if(it->first==v){ 
                 it=adj[u].erase(it); 
             }else{ 
                 ++it; 
             } 
         } 

         for(auto it=adj[v].begin(); it!=adj[v].end();){ 
             if(it->first==u){ 
                 it=adj[v].erase(it); 
             }else{ 
                 ++it; 
             } 
         } 
     }

     int getV() const { return V; }
};

Graph havelHakimi(std::vector<int>& sequence) {
    int n = sequence.size();
    std::vector<std::pair<int, int>> degrees;

    for (int i = 0; i < n; ++i) {
        if (sequence[i] >= n) {
            std::cout << "Degree " << sequence[i] << " at position " << i << " is too large for a graph with " << n << " vertices." << std::endl;
            return Graph(0);
        }
        degrees.push_back({sequence[i], i});
    }

    Graph G(n);

    while (!degrees.empty()) {
        std::sort(degrees.begin(), degrees.end(), std::greater<>());
        
        int d = degrees[0].first;
        int v = degrees[0].second;
        
        degrees.erase(degrees.begin());

       if (d > degrees.size()) { 
           std :: cout<<"Not enough vertices to connect to vertex "<<v<<" with degree "<<d<<"."<<std :: endl ; 
           return Graph(0);  
       } 

       static std :: vector<std :: set<int>> connections(n); 

       for (int i = 0; i < d; ++i) { 
           degrees[i].first--; 

           if(degrees[i].first<0){ 
               std :: cout<<"Negative degree encountered for vertex "<<degrees[i].second<<"."<<std :: endl ; 
               return Graph(0);  
           } 

           int u=degrees[i].second; 

           if(connections[v].find(u)==connections[v].end()){ 
               connections[v].insert(u); 
               connections[u].insert(v); 

               int weight=rand() % 100 + 1;  // Random weight between 1 and 100
               G.addEdge(v,u,weight);  
           }  
       }  
   }  

   return G;  
}  

int main() {  
   srand(static_cast<unsigned>(time(0)));  

   // Ask user to input graphical degree sequence  
   int n;  
   std :: cout<<"Enter the number of vertices: ";  
   std :: cin>>n;  

   std :: vector<int> sequence(n);  
   std :: cout<<"Enter the degree sequence: ";  
   for(int i=0;i<n;++i){  
       std :: cin>>sequence[i];  
   }  

   Graph G=havelHakimi(sequence);  
   if(G.getV()==0){  
       std :: cout<<"The sequence is not graphical."<<std :: endl;  
       return 0;  
   }  

   std :: cout<<"Graph created successfully."<<std :: endl;  
   G.printGraph();  

   // Test if the graph is Eulerian   
   if(G.isEulerGraph()){   
       std :: cout<<"The graph is Eulerian."<<std :: endl;   
       auto eulerCircuit=G.fleuryAlgorithm();   
       std :: cout<<"Euler circuit: ";   
       for(int v:eulerCircuit){   
           std :: cout<<v<<" ";   
       }   
       std :: cout<<std :: endl;   
   } else {   
       std :: cout<<"The graph is not Eulerian."<<std :: endl;   
   }   

   // Dijkstra's algorithm    
   int start=rand()%n;    
   std :: cout<<"Single source shortest paths from vertex "<<start<<":"<<std :: endl;    
   auto shortestPaths=G.dijkstra(start);    

   for(int i=0;i<n;++i){    
       if(shortestPaths[i]==std :: numeric_limits<int>::max()){    
           shortestPaths[i]=-1;// Indicate unreachable nodes    
       }    
       std :: cout <<"Distance to "<<i<<": "<<shortestPaths[i]<<std :: endl ;    
     }    

     // Minimum Spanning Tree    
     auto mst=G.primMST();    
     if(mst.empty()){    
         std :: cout<<"No MST exists"<<std :: endl;    
     }else{    
         std :: cout<<"Minimum Spanning Tree edges:"<<std :: endl;    
         for(const auto &[u,v]:mst){    
             std :: cout<<u<<" - "<<v<<std :: endl ;    
         }  
     }    

     // Fundamental Cutsets    
     auto cutsets=G.fundamentalCutsets(mst);    
     if(cutsets.empty()){
         std :: cout << "No Fundamental Cutsets found." <<std :: endl;
     } else {
         std :: cout<<"Fundamental Cutsets:"<<std :: endl;    
         for(const auto &cutset:cutsets){    
             for(int v:cutset){    
                 std :: cout<<v<<" ";    
             }    
             std :: cout<<std :: endl;
         }
     }

     // Fundamental Circuits     
     auto circuits=G.fundamentalCircuits(mst);     
     if(circuits.empty()){
         std :: cout << "No Fundamental Circuits found." <<std :: endl;
     } else {     
         std :: cout<<"Fundamental Circuits:"<<std :: endl;     
         for(const auto &circuit:circuits){     
             for(int v:circuit){     
                 std :: cout<<v<<" ";     
             }     
             std :: cout<<std :: endl;     
         }     
     }

     return 0 ;    
}