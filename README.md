# Cooperative Collision Avoidance – Parte Teórica

Este repositório implementa, em MATLAB, um método de **evitação cooperativa de colisões** baseado em **Optimal Reciprocal Collision Avoidance (ORCA)** com restrições adicionais para velocidades holonômicas.

O foco desta descrição é explicar o funcionamento teórico por trás da implementação, sem entrar em detalhes específicos do código.

---

## 1. Conceito Geral

O objetivo do método é calcular, a cada instante, a **velocidade ótima** para um agente móvel que:

1. **Evite colisões** com outros agentes ou obstáculos.
2. **Respeite suas limitações cinemáticas**, como velocidade máxima.
3. **Tente manter o rumo desejado** (velocidade preferida).

O problema é formulado como uma **otimização por programação linear** sobre o espaço de velocidades.

---

## 2. Espaço de Velocidades e Restrições ORCA

Cada agente possui:

- **Velocidade atual**: \( \mathbf{v}_i \)  
- **Velocidade preferida**: \( \mathbf{v}_i^{\text{pref}} \)  
- **Velocidade máxima holonômica**: \( v_{\text{max}} \)  

A teoria do ORCA define, para cada par de agentes \( i \) e \( j \), um **meio-plano viável** no espaço de velocidades \((v_x, v_y)\), de forma que, se \( \mathbf{v}_i \) estiver nesse meio-plano, não haverá colisão dentro de um horizonte de tempo \( \tau \).

Cada restrição é definida pela equação:

\[
\mathbf{n}_{ij} \cdot (\mathbf{v} - \mathbf{v}_{ij}^*) \geq 0
\]

onde:
- \( \mathbf{n}_{ij} \) é o vetor normal à fronteira da região de não colisão.
- \( \mathbf{v}_{ij}^* \) é o ponto da fronteira mais próximo da velocidade relativa atual.

O conjunto de todas as restrições para um agente forma um **polígono convexo** de velocidades seguras.

---

## 3. Região \( P_{AHV} \)

A implementação define \( P_{AHV} \) como a **interseção** de:

1. Todas as **regiões ORCA** obtidas com outros agentes.
2. O **disco de velocidades possíveis** limitado por \( v_{\text{max}} \):

\[
\sqrt{v_x^2 + v_y^2} \leq v_{\text{max}}
\]

O resultado é um **polígono convexo** no espaço \((v_x, v_y)\) que representa todas as velocidades viáveis.

---

## 4. Formulação como Programação Linear

O problema é formulado como:

\[
\min_{\mathbf{v} \in P_{AHV}} \|\mathbf{v} - \mathbf{v}_i^{\text{pref}}\|
\]

Ou seja:
- **Variáveis**: \( v_x, v_y \)
- **Restrições**: Inequações lineares definindo \( P_{AHV} \) + restrição de norma para \( v_{\text{max}} \)
- **Objetivo**: Escolher a velocidade viável mais próxima da velocidade preferida.

Como a norma euclidiana não é linear, pode-se aproximá-la usando normas \( L_1 \) ou \( L_\infty \) para resolver via programação linear clássica.

---

## 5. Interpretação Geométrica

1. **ORCA** → Cada agente define um conjunto de **meios-planos**.
2. **Interseção** desses meios-planos → região convexa de segurança.
3. **Corte pelo disco de \( v_{\text{max}} \)** → região \( P_{AHV} \).
4. **Otimização** → Seleciona o ponto dentro de \( P_{AHV} \) mais próximo da velocidade desejada.

---

## 6. Referências

- Van den Berg, J., et al. *Reciprocal n-body collision avoidance*. Springer Tracts in Advanced Robotics, 2011.
- Alonso-Mora, J., et al. *Optimal Reciprocal Collision Avoidance for Multiple Holonomic Robots with Acceleration Constraints*. IEEE ICRA, 2015.

---

