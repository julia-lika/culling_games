#ifndef EXPLORATION_ENGINE_H
#define EXPLORATION_ENGINE_H

#include "NavigationCore.h"
#include "RobotInterface.h"
#include <rclcpp/rclcpp.hpp>
#include <stack>

/**
 * @brief Motor de exploração do labirinto usando DFS otimizado
 */
class ExplorationEngine {
private:
    // Estatísticas de exploração
    struct ExplorationStats {
        int cells_explored;
        int backtrack_count;
        int dead_ends_found;
        
        ExplorationStats() : cells_explored(0), backtrack_count(0), dead_ends_found(0) {}
    };
    
    ExplorationStats stats_;

public:
    /**
     * @brief Executa exploração completa do labirinto
     */
    bool execute_exploration(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<SensorManager> sensor_manager,
        ExplorationData& exploration_data,
        NavigationState& nav_state
    ) {
        RCLCPP_INFO(node->get_logger(), "Iniciando exploração com DFS otimizado...");
        
        // Inicializa estado
        nav_state.start_position = Coordinate2D(0, 0);
        nav_state.current_position = nav_state.start_position;
        exploration_data.clear();
        
        // Executa exploração recursiva
        explore_recursive(
            node,
            sensor_manager,
            exploration_data,
            nav_state
        );
        
        // Exibe estatísticas
        RCLCPP_INFO(node->get_logger(), "Estatísticas da exploração:");
        RCLCPP_INFO(node->get_logger(), "  - Células exploradas: %d", stats_.cells_explored);
        RCLCPP_INFO(node->get_logger(), "  - Backtracks realizados: %d", stats_.backtrack_count);
        RCLCPP_INFO(node->get_logger(), "  - Dead-ends encontrados: %d", stats_.dead_ends_found);
        
        return nav_state.target_located;
    }

private:
    void explore_recursive(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<SensorManager> sensor_manager,
        ExplorationData& exploration_data,
        NavigationState& nav_state
    ) {
        // Marca posição como explorada
        exploration_data.explored_positions.insert(nav_state.current_position);
        stats_.cells_explored++;
        
        // Lê sensores
        auto sensors = sensor_manager->read_sensors();
        exploration_data.mapped_cells[nav_state.current_position] = sensors;
        
        // Verifica se encontrou o alvo
        if (check_target_adjacent(sensors, nav_state)) {
            RCLCPP_INFO(node->get_logger(), "Alvo detectado!");
            nav_state.target_located = true;
            exploration_data.explored_positions.insert(nav_state.target_position);
        }
        
        // Obtém direções exploráveis (ordem otimizada)
        auto explorable_dirs = get_explorable_directions(
            sensors,
            nav_state.current_position,
            exploration_data.explored_positions
        );
        
        if (explorable_dirs.empty()) {
            stats_.dead_ends_found++;
        }
        
        // Explora cada direção disponível
        for (const auto& dir : explorable_dirs) {
            Coordinate2D next_pos = nav_state.current_position + get_direction_delta(dir);
            
            // Tenta mover
            if (RobotInterface::move_robot(node, direction_to_command(dir), sensor_manager)) {
                Coordinate2D prev_pos = nav_state.current_position;
                nav_state.current_position = next_pos;
                exploration_data.movement_log.push_back(direction_to_command(dir));
                
                // Explora recursivamente
                explore_recursive(node, sensor_manager, exploration_data, nav_state);
                
                // Backtrack
                Direction opposite = get_opposite_direction(dir);
                if (RobotInterface::move_robot(node, direction_to_command(opposite), sensor_manager)) {
                    nav_state.current_position = prev_pos;
                    exploration_data.movement_log.pop_back();
                    stats_.backtrack_count++;
                }
            }
        }
    }
    
    bool check_target_adjacent(const SensorReading& sensors, NavigationState& nav_state) {
        if (sensors.target_north) {
            nav_state.target_position = nav_state.current_position + Coordinate2D(-1, 0);
            return true;
        }
        if (sensors.target_south) {
            nav_state.target_position = nav_state.current_position + Coordinate2D(1, 0);
            return true;
        }
        if (sensors.target_west) {
            nav_state.target_position = nav_state.current_position + Coordinate2D(0, -1);
            return true;
        }
        if (sensors.target_east) {
            nav_state.target_position = nav_state.current_position + Coordinate2D(0, 1);
            return true;
        }
        return false;
    }
    
    std::vector<Direction> get_explorable_directions(
        const SensorReading& sensors,
        const Coordinate2D& current_pos,
        const std::set<Coordinate2D>& explored
    ) {
        std::vector<Direction> directions;
        
        // Prioriza direções não exploradas (ordem: norte, leste, sul, oeste)
        std::vector<std::pair<Direction, bool>> candidates = {
            {Direction::NORTH, sensors.north && !sensors.target_north},
            {Direction::EAST, sensors.east && !sensors.target_east},
            {Direction::SOUTH, sensors.south && !sensors.target_south},
            {Direction::WEST, sensors.west && !sensors.target_west}
        };
        
        for (const auto& [dir, can_move] : candidates) {
            if (!can_move) continue;
            
            Coordinate2D next_pos = current_pos + get_direction_delta(dir);
            if (explored.find(next_pos) == explored.end()) {
                directions.push_back(dir);
            }
        }
        
        return directions;
    }
};

#endif // EXPLORATION_ENGINE_H
