#include "rclcpp/rclcpp.hpp"
#include "NavigationCore.h"
#include "ExplorationEngine.h"
#include "PathPlanner.h"
#include "RobotInterface.h"
#include "MapProcessor.h"
#include <iostream>
#include <chrono>
#include <thread>

/**
 * @brief Sistema de Navegação Autônoma para Labirinto
 * @author Sistema desenvolvido para o desafio de navegação
 * @version 2.0
 */
class AutonomousNavigator : public rclcpp::Node {
public:
    AutonomousNavigator() : Node("autonomous_navigator") {
        RCLCPP_INFO(this->get_logger(), "Inicializando sistema de navegação autônoma...");
    }
    
    void run() {
        print_header();
        
        // Fase de inicialização
        auto sensor_manager = std::make_shared<SensorManager>(shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        
        // Estruturas principais
        ExplorationData exploration_data;
        NavigationState nav_state;
        
        // Fase 1: Exploração inteligente
        RCLCPP_INFO(this->get_logger(), "[FASE 1] Iniciando exploração do ambiente...");
        auto start_time = std::chrono::steady_clock::now();
        
        ExplorationEngine explorer;
        bool success = explorer.execute_exploration(
            shared_from_this(),
            sensor_manager,
            exploration_data,
            nav_state
        );
        
        auto exploration_time = std::chrono::steady_clock::now() - start_time;
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(exploration_time).count();
        
        RCLCPP_INFO(this->get_logger(), "Exploração completa em %ld ms", duration_ms);
        RCLCPP_INFO(this->get_logger(), "Células mapeadas: %zu", exploration_data.mapped_cells.size());
        
        if (!success || !nav_state.target_located) {
            RCLCPP_ERROR(this->get_logger(), "Falha na exploração: Alvo não localizado");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Alvo localizado em: [%d, %d]", 
                    nav_state.target_position.x, nav_state.target_position.y);
        
        // Processamento do mapa
        MapProcessor map_processor;
        auto grid_map = map_processor.generate_grid_map(
            exploration_data,
            nav_state
        );
        
        map_processor.display_map(grid_map);
        
        // Fase 2: Planejamento de rota
        RCLCPP_INFO(this->get_logger(), "\n[FASE 2] Calculando rota ótima...");
        
        PathPlanner planner;
        auto navigation_graph = planner.build_navigation_graph(grid_map);
        
        if (!navigation_graph.is_valid()) {
            RCLCPP_ERROR(this->get_logger(), "Erro na construção do grafo de navegação");
            return;
        }
        
        auto optimal_path = planner.compute_optimal_path(navigation_graph);
        
        if (optimal_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Impossível calcular rota para o alvo");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Rota calculada: %zu movimentos necessários", 
                    optimal_path.size() - 1);
        
        // Fase 3: Execução da navegação
        RCLCPP_INFO(this->get_logger(), "\n[FASE 3] Pronto para navegação");
        RCLCPP_INFO(this->get_logger(), "Pressione ENTER para iniciar a execução...");
        std::cin.get();
        
        RobotInterface robot_interface;
        bool navigation_success = robot_interface.execute_navigation(
            shared_from_this(),
            optimal_path,
            120  // delay entre movimentos em ms
        );
        
        print_result(navigation_success);
    }
    
private:
    void print_header() {
        std::cout << "\n╔════════════════════════════════════════╗" << std::endl;
        std::cout << "║   SISTEMA DE NAVEGAÇÃO AUTÔNOMA v2.0   ║" << std::endl;
        std::cout << "║        Desafio de Labirinto            ║" << std::endl;
        std::cout << "╚════════════════════════════════════════╝\n" << std::endl;
    }
    
    void print_result(bool success) {
        if (success) {
            std::cout << "\n╔════════════════════════════════════════╗" << std::endl;
            std::cout << "║         ✓ MISSÃO COMPLETADA!           ║" << std::endl;
            std::cout << "╚════════════════════════════════════════╝" << std::endl;
        } else {
            std::cout << "\n╔════════════════════════════════════════╗" << std::endl;
            std::cout << "║      ⚠ FALHA NA EXECUÇÃO               ║" << std::endl;
            std::cout << "╚════════════════════════════════════════╝" << std::endl;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto navigator = std::make_shared<AutonomousNavigator>();
    navigator->run();
    
    rclcpp::shutdown();
    return 0;
}
