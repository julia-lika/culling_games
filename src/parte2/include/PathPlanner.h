#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "NavigationCore.h"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <iostream>

/**
 * @brief Grafo de navegação
 */
class NavigationGraph {
private:
    struct Node {
        int id;
        Coordinate2D position;
        std::vector<int> neighbors;
    };
    
    std::unordered_map<int, Node> nodes_;
    std::unordered_map<int, Coordinate2D> id_to_position_;
    std::map<Coordinate2D, int> position_to_id_;
    int robot_node_id_;
    int target_node_id_;

public:
    NavigationGraph() : robot_node_id_(-1), target_node_id_(-1) {}
    
    void add_node(int id, const Coordinate2D& pos, bool is_robot, bool is_target) {
        nodes_[id] = {id, pos, {}};
        id_to_position_[id] = pos;
        position_to_id_[pos] = id;
        
        if (is_robot) robot_node_id_ = id;
        if (is_target) target_node_id_ = id;
    }
    
    void add_edge(int from, int to) {
        if (nodes_.find(from) != nodes_.end() && nodes_.find(to) != nodes_.end()) {
            nodes_[from].neighbors.push_back(to);
        }
    }
    
    bool is_valid() const {
        return robot_node_id_ != -1 && target_node_id_ != -1;
    }
    
    int get_robot_node() const { return robot_node_id_; }
    int get_target_node() const { return target_node_id_; }
    
    const std::vector<int>& get_neighbors(int node_id) const {
        static std::vector<int> empty;
        auto it = nodes_.find(node_id);
        return (it != nodes_.end()) ? it->second.neighbors : empty;
    }
    
    Coordinate2D get_position(int node_id) const {
        auto it = id_to_position_.find(node_id);
        return (it != id_to_position_.end()) ? it->second : Coordinate2D();
    }
    
    int get_node_count() const { return nodes_.size(); }
    
    void print_statistics() const {
        std::cout << "\nEstatísticas do grafo:" << std::endl;
        std::cout << "  - Nós totais: " << nodes_.size() << std::endl;
        std::cout << "  - Nó do robô: " << robot_node_id_ << std::endl;
        std::cout << "  - Nó do alvo: " << target_node_id_ << std::endl;
        
        int total_edges = 0;
        for (const auto& [id, node] : nodes_) {
            total_edges += node.neighbors.size();
        }
        std::cout << "  - Arestas totais: " << total_edges / 2 << std::endl;
    }
};

/**
 * @brief Planejador de rotas usando BFS
 */
class PathPlanner {
public:
    /**
     * @brief Constrói grafo de navegação a partir do grid
     */
    NavigationGraph build_navigation_graph(const std::vector<std::vector<std::string>>& grid) {
        NavigationGraph graph;
        int node_id = 0;
        
        // Primeira passada: cria nós para células livres
        for (size_t i = 0; i < grid.size(); i++) {
            for (size_t j = 0; j < grid[i].size(); j++) {
                std::string cell = grid[i][j];
                
                // Ignora paredes
                if (cell == "b") continue;
                
                Coordinate2D pos(i, j);
                bool is_robot = (cell == "r");
                bool is_target = (cell == "t");
                
                graph.add_node(node_id++, pos, is_robot, is_target);
            }
        }
        
        // Segunda passada: conecta nós adjacentes
        std::vector<Coordinate2D> directions = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1}  // Norte, Sul, Oeste, Leste
        };
        
        for (size_t i = 0; i < grid.size(); i++) {
            for (size_t j = 0; j < grid[i].size(); j++) {
                if (grid[i][j] == "b") continue;
                
                Coordinate2D current(i, j);
                int current_id = get_node_id(graph, current);
                
                // Verifica vizinhos
                for (const auto& delta : directions) {
                    Coordinate2D neighbor = current + delta;
                    
                    // Verifica limites
                    if (neighbor.x < 0 || neighbor.x >= (int)grid.size() ||
                        neighbor.y < 0 || neighbor.y >= (int)grid[0].size()) {
                        continue;
                    }
                    
                    // Verifica se não é parede
                    if (grid[neighbor.x][neighbor.y] != "b") {
                        int neighbor_id = get_node_id(graph, neighbor);
                        graph.add_edge(current_id, neighbor_id);
                    }
                }
            }
        }
        
        graph.print_statistics();
        return graph;
    }
    
    /**
     * @brief Calcula caminho ótimo usando BFS
     */
    std::vector<Coordinate2D> compute_optimal_path(const NavigationGraph& graph) {
        std::vector<Coordinate2D> path;
        
        if (!graph.is_valid()) {
            return path;
        }
        
        int start = graph.get_robot_node();
        int goal = graph.get_target_node();
        
        // BFS para encontrar caminho mais curto
        std::queue<int> queue;
        std::unordered_map<int, int> parent;
        std::unordered_map<int, bool> visited;
        
        queue.push(start);
        visited[start] = true;
        parent[start] = -1;
        
        bool found = false;
        int nodes_explored = 0;
        
        while (!queue.empty() && !found) {
            int current = queue.front();
            queue.pop();
            nodes_explored++;
            
            if (current == goal) {
                found = true;
                break;
            }
            
            // Explora vizinhos
            for (int neighbor : graph.get_neighbors(current)) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    parent[neighbor] = current;
                    queue.push(neighbor);
                }
            }
        }
        
        std::cout << "  - Nós explorados no BFS: " << nodes_explored << std::endl;
        
        // Reconstrói caminho se encontrado
        if (found) {
            std::vector<int> node_path;
            int current = goal;
            
            while (current != -1) {
                node_path.push_back(current);
                current = parent[current];
            }
            
            std::reverse(node_path.begin(), node_path.end());
            
            // Converte para coordenadas
            for (int node : node_path) {
                path.push_back(graph.get_position(node));
            }
            
            std::cout << "  - Comprimento do caminho: " << path.size() - 1 << " movimentos" << std::endl;
        }
        
        return path;
    }

private:
    int get_node_id(const NavigationGraph& graph, const Coordinate2D& pos) {
        // Como não temos acesso direto ao mapa position_to_id_, 
        // vamos usar uma busca simples (funciona pois o grafo é pequeno)
        for (int id = 0; id < graph.get_node_count(); id++) {
            if (graph.get_position(id) == pos) {
                return id;
            }
        }
        return -1;
    }
};

#endif // PATH_PLANNER_H
