# Cooperative Collision Avoidance

Este repositório apresenta uma implementação em MATLAB de um algoritmo de **Prevenção Cooperativa de Colisão** para robôs móveis, baseada em restrições de velocidades lineares e angulares, formuladas como um problema de otimização.

## Visão Geral

A abordagem baseia-se na definição de uma região de velocidades admissíveis para cada robô, levando em consideração:

- **Capacidade dinâmica do robô** (limites de velocidade)
- **Restrições de não-colisão** em relação a outros robôs e obstáculos
- **Orientação atual** do robô para definição da região de velocidade possível

O objetivo é encontrar, a cada passo, a velocidade mais próxima da velocidade desejada (`v_des`), respeitando todas as restrições.

---

## Formulação Teórica

O problema é formulado como uma **Programação Linear (PL)**:

Minimizar:
$$
\| v - v_{\text{des}} \|^2
$$

Sujeito a:
1. **Restrição dinâmica (velocidade máxima holonômica)**:
$$
- v_{\max} \leq v_x \leq v_{\max}
$$
$$
- v_{\max} \leq v_y \leq v_{\max}
$$

2. **Restrição de região admissível $P_{AHV}$**:

A região $P_{AHV}$ é poligonal e definida com base nas velocidades relativas e nas zonas seguras de operação, levando em conta a orientação do robô.

3. **Restrição cooperativa de colisão**:

Para cada par de robôs $i$ e $j$, define-se a condição para evitar colisão:
$$
(n_{ij})^\top (v_i - v_j) \geq \tau_{ij}
$$
onde:
- $n_{ij}$ é o vetor normal à fronteira de colisão
- $\tau_{ij}$ é o termo de margem de segurança

---

## Região de Velocidades Admissíveis

Em vez de usar um limite circular para a magnitude da velocidade, este trabalho define a região como um **quadrado rotacionado** com lado $2v_{\max}$, alinhado com o referencial do robô.  
Isto permite limitar separadamente as velocidades longitudinal ($v_x$) e lateral ($v_y$) e aplicar uma rotação correspondente à orientação atual $\theta$ do robô.

A transformação para o referencial global é dada por:
$$
\begin{bmatrix}
v_x^{g} \\
v_y^{g}
\end{bmatrix}
=
R(\theta)
\begin{bmatrix}
v_x^{r} \\
v_y^{r}
\end{bmatrix}
$$

Com:
$$
R(\theta) =
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}
$$

---

## Resumo do Processo

1. **Definição das restrições** (dinâmicas e de colisão)
2. **Construção da região $P_{AHV}$** levando em conta a orientação
3. **Formulação da Programação Linear**
4. **Resolução usando solver de otimização**
5. **Aplicação da velocidade resultante** no robô

---

## Referências

- Van den Berg, J., et al. *Reciprocal Velocity Obstacles for real-time multi-agent navigation*. IEEE ICRA, 2008.
- Fiorini, P., Shiller, Z. *Motion Planning in Dynamic Environments Using Velocity Obstacles*. IJRR, 1998.

---
