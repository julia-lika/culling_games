#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "NavigationCore.h"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <memory>

/**
 * @brief Gerenciador de sensores do robô
 */
class SensorManager {
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_subscription_;
    SensorReading current_reading_;
    bool data_available_;
    std::mutex data_mutex_;

public:
    explicit SensorManager(rclcpp::Node::SharedPtr node) 
        : node_(node), data_available_(false) {
        
        sensor_subscription_ = node_->create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors",
            10,
            [this](const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                parse_sensor_message(msg);
                data_available_ = true;
            }
        );
    }
    
    SensorReading read_sensors() {
        // Aguarda dados disponíveis
        while (!data_available_ && rclcpp::ok()) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        
        std::lock_guard<std::mutex> lock(data_mutex_);
        return current_reading_;
    }
    
    void reset() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        data_available_ = false;
    }

private:
    void parse_sensor_message(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        // Células navegáveis
        current_reading_.north = (msg->up == "f" || msg->up == "t");
        current_reading_.south = (msg->down == "f" || msg->down == "t");
        current_reading_.west = (msg->left == "f" || msg->left == "t");
        current_reading_.east = (msg->right == "f" || msg->right == "t");
        current_reading_.northwest = (msg->up_left == "f" || msg->up_left == "t");
        current_reading_.northeast = (msg->up_right == "f" || msg->up_right == "t");
        current_reading_.southwest = (msg->down_left == "f" || msg->down_left == "t");
        current_reading_.southeast = (msg->down_right == "f" || msg->down_right == "t");
        
        // Detecção de alvo
        current_reading_.target_north = (msg->up == "t");
        current_reading_.target_south = (msg->down == "t");
        current_reading_.target_west = (msg->left == "t");
        current_reading_.target_east = (msg->right == "t");
    }
};

/**
 * @brief Interface de controle do robô
 */
class RobotInterface {
public:
    /**
     * @brief Move o robô em uma direção específica
     */
    static bool move_robot(
        rclcpp::Node::SharedPtr node,
        const std::string& command,
        std::shared_ptr<SensorManager> sensor_manager
    ) {
        auto move_service = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        
        if (!move_service->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(node->get_logger(), "Serviço de movimento não disponível");
            return false;
        }
        
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = command;
        
        // Limpa dados anteriores
        sensor_manager->reset();
        
        auto future = move_service->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
            // Aguarda estabilização dos sensores
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            return future.get()->success;
        }
        
        return false;
    }
    
    /**
     * @brief Executa sequência de navegação
     */
    bool execute_navigation(
        rclcpp::Node::SharedPtr node,
        const std::vector<Coordinate2D>& path,
        int movement_delay_ms = 100
    ) {
        auto move_service = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        
        RCLCPP_INFO(node->get_logger(), "Iniciando navegação com %zu movimentos", path.size() - 1);
        
        // Aguarda serviço
        while (!move_service->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return false;
            RCLCPP_INFO(node->get_logger(), "Aguardando serviço de movimento...");
        }
        
        // Converte caminho em comandos
        auto commands = path_to_commands(path);
        
        // Executa comandos
        int successful_moves = 0;
        int total_moves = commands.size();
        
        for (size_t i = 0; i < commands.size(); i++) {
            auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
            request->direction = commands[i];
            
            auto future = move_service->async_send_request(request);
            
            if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
                if (future.get()->success) {
                    successful_moves++;
                    
                    // Feedback de progresso
                    if ((i + 1) % 10 == 0 || i == commands.size() - 1) {
                        RCLCPP_INFO(node->get_logger(), "Progresso: %zu/%zu movimentos", 
                                   i + 1, commands.size());
                    }
                } else {
                    RCLCPP_WARN(node->get_logger(), "Falha no movimento %zu", i + 1);
                }
            }
            
            // Delay entre movimentos
            if (i < commands.size() - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(movement_delay_ms));
            }
        }
        
        RCLCPP_INFO(node->get_logger(), "Navegação finalizada: %d/%d movimentos bem-sucedidos", 
                   successful_moves, total_moves);
        
        return successful_moves == total_moves;
    }

private:
    std::vector<std::string> path_to_commands(const std::vector<Coordinate2D>& path) {
        std::vector<std::string> commands;
        
        for (size_t i = 0; i < path.size() - 1; i++) {
            int dx = path[i + 1].x - path[i].x;
            int dy = path[i + 1].y - path[i].y;
            
            if (dx == -1 && dy == 0) {
                commands.push_back("up");
            } else if (dx == 1 && dy == 0) {
                commands.push_back("down");
            } else if (dx == 0 && dy == -1) {
                commands.push_back("left");
            } else if (dx == 0 && dy == 1) {
                commands.push_back("right");
            }
        }
        
        return commands;
    }
};

#endif // ROBOT_INTERFACE_H
