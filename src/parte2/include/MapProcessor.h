#ifndef MAP_PROCESSOR_H
#define MAP_PROCESSOR_H

#include "NavigationCore.h"
#include <iostream>
#include <algorithm>
#include <iomanip>

/**
 * @brief Processador de mapas do labirinto
 */
class MapProcessor {
public:
    /**
     * @brief Gera grid a partir dos dados de exploração
     */
    std::vector<std::vector<std::string>> generate_grid_map(
        const ExplorationData& exploration_data,
        const NavigationState& nav_state
    ) {
        // Calcula limites do mapa
        MapBounds bounds = calculate_bounds(
            exploration_data.mapped_cells,
            nav_state
        );
        
        // Cria grid com margens
        int rows = bounds.max_x - bounds.min_x + 3; // +3 para margem de paredes
        int cols = bounds.max_y - bounds.min_y + 3;
        
        std::vector<std::vector<std::string>> grid(rows, std::vector<std::string>(cols, "b"));
        
        // Preenche células exploradas
        populate_grid(grid, exploration_data, nav_state, bounds);
        
        return grid;
    }
    
    /**
     * @brief Exibe o mapa formatado
     */
    void display_map(const std::vector<std::vector<std::string>>& grid) {
        std::cout << "\n╔═══════════════════════════════════╗" << std::endl;
        std::cout << "║         MAPA EXPLORADO            ║" << std::endl;
        std::cout << "╚═══════════════════════════════════╝" << std::endl;
        
        std::cout << "\nLegenda: [R]=Robô [T]=Alvo [.]=Livre [#]=Parede\n" << std::endl;
        
        // Imprime índices de coluna
        std::cout << "    ";
        for (size_t j = 0; j < grid[0].size(); j++) {
            std::cout << std::setw(2) << j % 10;
        }
        std::cout << std::endl;
        
        std::cout << "   ╔";
        for (size_t j = 0; j < grid[0].size(); j++) {
            std::cout << "══";
        }
        std::cout << "╗" << std::endl;
        
        // Imprime grid com índices de linha
        for (size_t i = 0; i < grid.size(); i++) {
            std::cout << std::setw(2) << i << " ║";
            for (const auto& cell : grid[i]) {
                std::cout << format_cell(cell) << " ";
            }
            std::cout << "║" << std::endl;
        }
        
        std::cout << "   ╚";
        for (size_t j = 0; j < grid[0].size(); j++) {
            std::cout << "══";
        }
        std::cout << "╝" << std::endl;
        
        // Estatísticas do mapa
        print_map_statistics(grid);
    }

private:
    struct MapBounds {
        int min_x, max_x;
        int min_y, max_y;
        
        MapBounds() : min_x(0), max_x(0), min_y(0), max_y(0) {}
    };
    
    MapBounds calculate_bounds(
        const std::map<Coordinate2D, SensorReading>& mapped_cells,
        const NavigationState& nav_state
    ) {
        MapBounds bounds;
        bounds.min_x = bounds.max_x = nav_state.start_position.x;
        bounds.min_y = bounds.max_y = nav_state.start_position.y;
        
        // Encontra limites das células exploradas
        for (const auto& [pos, _] : mapped_cells) {
            bounds.min_x = std::min(bounds.min_x, pos.x);
            bounds.max_x = std::max(bounds.max_x, pos.x);
            bounds.min_y = std::min(bounds.min_y, pos.y);
            bounds.max_y = std::max(bounds.max_y, pos.y);
        }
        
        // Considera posição do alvo
        if (nav_state.target_located) {
            bounds.min_x = std::min(bounds.min_x, nav_state.target_position.x);
            bounds.max_x = std::max(bounds.max_x, nav_state.target_position.x);
            bounds.min_y = std::min(bounds.min_y, nav_state.target_position.y);
            bounds.max_y = std::max(bounds.max_y, nav_state.target_position.y);
        }
        
        // Adiciona margem para paredes
        bounds.min_x--;
        bounds.min_y--;
        bounds.max_x++;
        bounds.max_y++;
        
        return bounds;
    }
    
    void populate_grid(
        std::vector<std::vector<std::string>>& grid,
        const ExplorationData& exploration_data,
        const NavigationState& nav_state,
        const MapBounds& bounds
    ) {
        // Preenche células exploradas
        for (const auto& [pos, sensors] : exploration_data.mapped_cells) {
            int grid_x = pos.x - bounds.min_x;
            int grid_y = pos.y - bounds.min_y;
            
            // Marca célula atual
            if (pos == nav_state.start_position) {
                grid[grid_x][grid_y] = "r";
            } else {
                grid[grid_x][grid_y] = "f";
            }
            
            // Marca células adjacentes baseado nos sensores
            mark_adjacent_cells(grid, grid_x, grid_y, sensors);
        }
        
        // Garante que o alvo está marcado
        if (nav_state.target_located) {
            int target_x = nav_state.target_position.x - bounds.min_x;
            int target_y = nav_state.target_position.y - bounds.min_y;
            grid[target_x][target_y] = "t";
        }
    }
    
    void mark_adjacent_cells(
        std::vector<std::vector<std::string>>& grid,
        int x, int y,
        const SensorReading& sensors
    ) {
        int rows = grid.size();
        int cols = grid[0].size();
        
        // Norte
        if (sensors.north && x > 0 && grid[x-1][y] == "b") {
            grid[x-1][y] = sensors.target_north ? "t" : "f";
        }
        
        // Sul
        if (sensors.south && x < rows - 1 && grid[x+1][y] == "b") {
            grid[x+1][y] = sensors.target_south ? "t" : "f";
        }
        
        // Oeste
        if (sensors.west && y > 0 && grid[x][y-1] == "b") {
            grid[x][y-1] = sensors.target_west ? "t" : "f";
        }
        
        // Leste
        if (sensors.east && y < cols - 1 && grid[x][y+1] == "b") {
            grid[x][y+1] = sensors.target_east ? "t" : "f";
        }
    }
    
    std::string format_cell(const std::string& cell) const {
        if (cell == "r") return "R";
        if (cell == "t") return "T";
        if (cell == "f") return ".";
        return "#";
    }
    
    void print_map_statistics(const std::vector<std::vector<std::string>>& grid) {
        int free_cells = 0;
        int wall_cells = 0;
        
        for (const auto& row : grid) {
            for (const auto& cell : row) {
                if (cell == "f" || cell == "r" || cell == "t") {
                    free_cells++;
                } else if (cell == "b") {
                    wall_cells++;
                }
            }
        }
        
        std::cout << "\nEstatísticas do mapa:" << std::endl;
        std::cout << "  - Dimensões: " << grid.size() << " x " << grid[0].size() << std::endl;
        std::cout << "  - Células livres: " << free_cells << std::endl;
        std::cout << "  - Paredes: " << wall_cells << std::endl;
        std::cout << "  - Total: " << (free_cells + wall_cells) << std::endl;
    }
};

#endif // MAP_PROCESSOR_H
