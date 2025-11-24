# **Documentação do Sistema de Navegação Autônoma para Labirinto**

## **Resumo do Projeto**

Este projeto implementa um sistema de navegação autônoma para robôs em labirintos, utilizando ROS2. Ele integra exploração inteligente, mapeamento, planejamento de caminho ótimo e execução de navegação. O robô explora o labirinto usando uma abordagem DFS otimizada, constrói um mapa interno, calcula a rota mais curta até o alvo usando BFS e executa os movimentos no mundo real.

- [Vídeo rodando o projeto + Explicando o código](https://drive.google.com/file/d/1mBLWrtoYluwnWSodmDuDcyWjJmGUetso/view?usp=sharing)
---

## **Módulos do Sistema**

### **1. NavigationCore.h**

Define as estruturas básicas de navegação:

* `Coordinate2D`: Representa uma posição no grid do labirinto.
* `SensorReading`: Representa leituras de sensores do robô (quais células adjacentes são livres e onde o alvo está).
* `ExplorationData`: Armazena células exploradas, mapeadas e o log de movimentos.
* `NavigationState`: Armazena posição inicial, atual e do alvo, e se o alvo foi localizado.
* `Direction`: Enum de direções possíveis (NORTE, SUL, LESTE, OESTE).
* Funções auxiliares:

  * `direction_to_command(Direction)`: converte direção em string de comando ("up", "down", etc.).
  * `get_opposite_direction(Direction)`: retorna a direção oposta.
  * `get_direction_delta(Direction)`: retorna o delta de coordenadas para mover no grid.

---

### **2. ExplorationEngine.h**

Implementa a exploração do labirinto usando **DFS otimizado**:

* Estrutura `ExplorationStats`: Guarda estatísticas de exploração (células exploradas, backtracks, dead-ends).
* Método principal `execute_exploration(...)`:

  1. Inicializa estado e posição do robô.
  2. Executa a função recursiva `explore_recursive(...)`.
  3. Retorna se o alvo foi localizado e imprime estatísticas.
* Função `explore_recursive(...)`:

  1. Marca célula atual como explorada.
  2. Lê sensores e atualiza `exploration_data`.
  3. Verifica se o alvo está adjacente.
  4. Obtém direções disponíveis e explora recursivamente.
  5. Realiza **backtrack** se necessário.
* Métodos auxiliares:

  * `check_target_adjacent`: detecta se o alvo está em alguma célula vizinha.
  * `get_explorable_directions`: prioriza direções não exploradas (N, E, S, W) evitando dead-ends.

---

### **3. MapProcessor.h**

Processa os dados coletados durante a exploração e gera um **mapa visual**:

* `generate_grid_map(...)`: cria uma matriz representando o labirinto.
* `display_map(...)`: imprime o mapa formatado no terminal com legendas.
* Funções auxiliares:

  * `calculate_bounds(...)`: calcula limites do mapa baseado nas células exploradas e no alvo.
  * `populate_grid(...)`: preenche células exploradas e marca o alvo e o robô.
  * `mark_adjacent_cells(...)`: marca células livres ou com alvo baseadas nas leituras dos sensores.
  * `format_cell(...)`: converte código interno para caracteres legíveis (R, T, ., #).
  * `print_map_statistics(...)`: imprime estatísticas do mapa (células livres, paredes, dimensões).

---

### **4. PathPlanner.h**

Constrói o grafo de navegação e calcula a rota ótima usando **BFS**:

* `NavigationGraph`: estrutura de grafo com nós representando células livres.

  * Cada nó possui id, posição e lista de vizinhos.
  * Armazena IDs do nó do robô e do alvo.
  * Função `print_statistics()` para exibir informações do grafo.
* `PathPlanner`:

  * `build_navigation_graph(grid)`: constrói grafo conectando células livres adjacentes.
  * `compute_optimal_path(graph)`: calcula caminho mínimo usando BFS, retornando lista de `Coordinate2D`.
  * `get_node_id(graph, pos)`: retorna o ID de um nó baseado em sua posição.

---

### **5. RobotInterface.h**

Interface de controle e leitura de sensores do robô via ROS2:

* `SensorManager`: gerencia leituras de sensores recebidas via tópico `/culling_games/robot_sensors`.

  * `read_sensors()`: retorna última leitura disponível.
  * `reset()`: limpa leitura anterior.
  * `parse_sensor_message(...)`: converte mensagem ROS para `SensorReading`.
* `RobotInterface`:

  * `move_robot(node, command, sensor_manager)`: envia comando de movimento ao robô usando serviço `/move_command`.
  * `execute_navigation(node, path, delay)`: percorre um caminho pré-calculado, enviando comandos de movimento.

---

### **6. main.cpp**

Executa todo o sistema de navegação autônoma:

* Classe `AutonomousNavigator` herda de `rclcpp::Node`.
* Passos principais (`run()`):

  1. Inicializa sensores e estruturas de exploração.
  2. **Fase 1: Exploração**

     * Executa DFS otimizado com `ExplorationEngine`.
     * Coleta células exploradas e localização do alvo.
  3. **Fase 2: Processamento do mapa e planejamento**

     * Gera grid com `MapProcessor`.
     * Constrói grafo e calcula rota ótima com `PathPlanner`.
  4. **Fase 3: Execução da navegação**

     * Envia comandos ao robô via `RobotInterface`.
     * Monitora progresso e sucesso da missão.
* Funções auxiliares:

  * `print_header()`: imprime cabeçalho do sistema.
  * `print_result(success)`: imprime resultado final (✓ MISSÃO COMPLETADA! ou ⚠ FALHA NA EXECUÇÃO).

---

## **Fluxo de Funcionamento**

1. Robô inicia na posição inicial `[0,0]`.
2. Sensores captam o ambiente.
3. `ExplorationEngine` explora o labirinto usando DFS otimizado.
4. `MapProcessor` gera e exibe mapa do labirinto.
5. `PathPlanner` constrói grafo de navegação e calcula caminho ótimo para o alvo.
6. `RobotInterface` executa movimentos no mundo real.
7. Estatísticas e resultados finais são exibidos.

---

## **Guia de Execução**

Para rodar o sistema completo em ROS2:

1. **Exploração do labirinto**

```bash
ros2 run cg maze
```

2. **Execução do A* (ou planejamento de caminho)**

```bash
ros2 run parte1 astar-node
```

3. **Execução do Navegador Autônomo**

```bash
ros2 run parte2 maze_navigator_node
```

> O robô realizará exploração, mapeará o labirinto, calculará a rota ótima e navegará até o alvo.

---
