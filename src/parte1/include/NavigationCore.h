#ifndef NAVIGATION_CORE_H
#define NAVIGATION_CORE_H

#include <map>
#include <set>
#include <vector>
#include <utility>

/**
 * @brief Coordenada 2D no espaço do labirinto
 */
struct Coordinate2D {
    int x;  // linha
    int y;  // coluna
    
    Coordinate2D() : x(0), y(0) {}
    Coordinate2D(int row, int col) : x(row), y(col) {}
    
    bool operator<(const Coordinate2D& other) const {
        return (x != other.x) ? (x < other.x) : (y < other.y);
    }
    
    bool operator==(const Coordinate2D& other) const {
        return x == other.x && y == other.y;
    }
    
    Coordinate2D operator+(const Coordinate2D& delta) const {
        return Coordinate2D(x + delta.x, y + delta.y);
    }
};

/**
 * @brief Dados de sensoriamento do robô
 */
struct SensorReading {
    // Células adjacentes navegáveis
    bool north;
    bool south;
    bool west;
    bool east;
    bool northwest;
    bool northeast;
    bool southwest;
    bool southeast;
    
    // Detecção de objetivo
    bool target_north;
    bool target_south;
    bool target_west;
    bool target_east;
    
    SensorReading() {
        north = south = west = east = false;
        northwest = northeast = southwest = southeast = false;
        target_north = target_south = target_west = target_east = false;
    }
};

/**
 * @brief Dados coletados durante a exploração
 */
struct ExplorationData {
    std::map<Coordinate2D, SensorReading> mapped_cells;
    std::set<Coordinate2D> explored_positions;
    std::vector<std::string> movement_log;
    
    void clear() {
        mapped_cells.clear();
        explored_positions.clear();
        movement_log.clear();
    }
};

/**
 * @brief Estado atual da navegação
 */
struct NavigationState {
    Coordinate2D start_position;
    Coordinate2D current_position;
    Coordinate2D target_position;
    bool target_located;
    
    NavigationState() : target_located(false) {
        start_position = current_position = Coordinate2D(0, 0);
    }
};

/**
 * @brief Direções de movimento possíveis
 */
enum class Direction {
    NORTH,
    SOUTH,
    EAST,
    WEST,
    NONE
};

/**
 * @brief Converte direção em string de comando
 */
inline std::string direction_to_command(Direction dir) {
    switch(dir) {
        case Direction::NORTH: return "up";
        case Direction::SOUTH: return "down";
        case Direction::EAST: return "right";
        case Direction::WEST: return "left";
        default: return "";
    }
}

/**
 * @brief Obtém direção oposta
 */
inline Direction get_opposite_direction(Direction dir) {
    switch(dir) {
        case Direction::NORTH: return Direction::SOUTH;
        case Direction::SOUTH: return Direction::NORTH;
        case Direction::EAST: return Direction::WEST;
        case Direction::WEST: return Direction::EAST;
        default: return Direction::NONE;
    }
}

/**
 * @brief Obtém delta de coordenadas para uma direção
 */
inline Coordinate2D get_direction_delta(Direction dir) {
    switch(dir) {
        case Direction::NORTH: return Coordinate2D(-1, 0);
        case Direction::SOUTH: return Coordinate2D(1, 0);
        case Direction::EAST: return Coordinate2D(0, 1);
        case Direction::WEST: return Coordinate2D(0, -1);
        default: return Coordinate2D(0, 0);
    }
}

#endif // NAVIGATION_CORE_H
