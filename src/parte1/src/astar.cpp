#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <vector>
#include <string>
#include <queue>
#include <limits>
#include <optional>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include "NavigationCore.h"
#include "RobotInterface.h"

// ---------------- Estruturas de mapa ----------------

struct MapData {
    std::vector<std::vector<char>> grid;
    int rows = 0;
    int cols = 0;
    int start = -1;
    int target = -1;

    inline int id(int r, int c) const { return r * cols + c; }
    inline std::pair<int,int> rc(int id) const { return { id / cols, id % cols }; }
    inline bool in_bounds(int r, int c) const { return r >= 0 && r < rows && c >= 0 && c < cols; }
};

// ---------------- Cliente /get_map ----------------

class GetMapClient : public rclcpp::Node {
public:
    GetMapClient() : Node("get_map_client") {
        client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
    }

    std::shared_ptr<cg_interfaces::srv::GetMap::Response> request_map(std::chrono::seconds timeout = std::chrono::seconds(5)) {
        RCLCPP_INFO(this->get_logger(), "Solicitando serviço /get_map...");
        if (!client_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "Serviço /get_map não disponível após %lds", timeout.count());
            return nullptr;
        }
        auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto fut = client_->async_send_request(req);
        auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut);
        if (ret == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Resposta /get_map recebida.");
            return fut.get();
        }
        RCLCPP_ERROR(this->get_logger(), "Falha ao chamar /get_map.");
        return nullptr;
    }

private:
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client_;
};

// ---------------- Helpers A* ----------------

MapData build_map(const std::shared_ptr<cg_interfaces::srv::GetMap::Response> &res) {
    MapData m;
    if (!res) return m;

    int rows = static_cast<int>(res->occupancy_grid_shape[0]);
    int cols = static_cast<int>(res->occupancy_grid_shape[1]);
    m.rows = rows;
    m.cols = cols;
    m.grid.assign(rows, std::vector<char>(cols, 'b'));

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            std::string cell = res->occupancy_grid_flattened[r * cols + c];
            char ch = cell.empty() ? 'b' : cell[0];
            m.grid[r][c] = ch;
            if (ch == 'r') m.start = m.id(r,c);
            if (ch == 't') m.target = m.id(r,c);
        }
    }
    return m;
}

std::vector<std::vector<int>> build_adj_list(const MapData &m) {
    const int N = m.rows * m.cols;
    std::vector<std::vector<int>> adj(N);
    static const int dr[4] = {-1, 1, 0, 0};
    static const int dc[4] = {0, 0, -1, 1};

    for (int r = 0; r < m.rows; ++r) {
        for (int c = 0; c < m.cols; ++c) {
            if (m.grid[r][c] == 'b') continue;
            int u = m.id(r,c);
            for (int k = 0; k < 4; ++k) {
                int nr = r + dr[k];
                int nc = c + dc[k];
                if (!m.in_bounds(nr,nc)) continue;
                if (m.grid[nr][nc] == 'b') continue;
                adj[u].push_back(m.id(nr,nc));
            }
        }
    }
    return adj;
}

inline int manhattan(int id, const MapData &m) {
    auto [r,c] = m.rc(id);
    auto [tr,tc] = m.rc(m.target);
    return std::abs(r - tr) + std::abs(c - tc);
}

struct PQNode {
    int id;
    int f;
    bool operator<(const PQNode &o) const { return f > o.f; } // min-heap
};

std::optional<std::vector<int>> astar(const MapData &m, const std::vector<std::vector<int>> &adj) {
    if (m.start < 0 || m.target < 0) return std::nullopt;

    const int N = m.rows * m.cols;
    const int INF = std::numeric_limits<int>::max() / 4;

    std::vector<int> g(N, INF), parent(N, -1);
    std::vector<char> closed(N, 0);

    std::priority_queue<PQNode> open;
    g[m.start] = 0;
    open.push({m.start, manhattan(m.start, m)});

    while (!open.empty()) {
        auto top = open.top(); open.pop();
        int u = top.id;
        if (closed[u]) continue;
        closed[u] = 1;

        if (u == m.target) {
            std::vector<int> path;
            for (int cur = u; cur != -1; cur = parent[cur]) path.push_back(cur);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int v : adj[u]) {
            if (closed[v]) continue;
            int tentative = g[u] + 1;
            if (tentative < g[v]) {
                g[v] = tentative;
                parent[v] = u;
                int f = tentative + manhattan(v, m);
                open.push({v, f});
            }
        }
    }
    return std::nullopt;
}

std::vector<Coordinate2D> path_to_coordinates(const std::vector<int> &path, const MapData &m) {
    std::vector<Coordinate2D> coords;
    for (auto id : path) {
        auto [r,c] = m.rc(id);
        coords.push_back(Coordinate2D(r,c));
    }
    return coords;
}

// ---------------- Main ----------------

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("astar_navigation");

    auto map_client = std::make_shared<GetMapClient>();
    auto resp = map_client->request_map(std::chrono::seconds(10));
    if (!resp) {
        RCLCPP_ERROR(node->get_logger(), "Não foi possível obter o mapa. Saindo.");
        rclcpp::shutdown();
        return 1;
    }

    MapData m = build_map(resp);
    auto adj = build_adj_list(m);

    auto path_opt = astar(m, adj);
    if (!path_opt.has_value()) {
        RCLCPP_ERROR(node->get_logger(), "Nenhum caminho encontrado.");
        rclcpp::shutdown();
        return 2;
    }

    auto path = path_opt.value();
    auto nav_path = path_to_coordinates(path, m);

    std::cout << "=== Caminho ===\n";
    for (auto &coord : nav_path) std::cout << "(" << coord.x << "," << coord.y << ") ";
    std::cout << "\n=== Executando movimentos ===\n";

    RobotInterface robot_interface;
    bool success = robot_interface.execute_navigation(node, nav_path, 150);

    if (success)
        RCLCPP_INFO(node->get_logger(), "Robô chegou ao destino!");
    else
        RCLCPP_WARN(node->get_logger(), "Falha ao percorrer o caminho.");

    rclcpp::shutdown();
    return 0;
}
