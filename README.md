# Cooperative Collision Avoidance

Este repositório apresenta uma implementação em MATLAB de um algoritmo de **Prevenção Cooperativa de Colisão** para robôs móveis, baseada em restrições de velocidades lineares e angulares, formuladas como um problema de otimização.

## Visão Geral

A abordagem baseia-se na definição de uma região de velocidades admissíveis para cada robô, levando em consideração:

- **Capacidade dinâmica do robô** (limites de velocidade)
- **Restrições de não-colisão** em relação a outros robôs e obstáculos
- **Orientação atual** do robô para definição da região de velocidade possível

O objetivo é encontrar, a cada passo, a velocidade mais próxima da velocidade desejada (`v_pref`), respeitando todas as restrições.


