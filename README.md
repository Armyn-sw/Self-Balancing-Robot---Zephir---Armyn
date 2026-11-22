# Self-Balancing-Robot---Zephir---Armyn

Objectives of the projetct
- Learn the hardware and the sofware and the integration of a PID/LQR

Final form: A self balancing robot


<img width="740" height="493" alt="image" src="https://github.com/user-attachments/assets/afbdb97c-c7c4-4f69-b0c3-81c8cfcd2c33" />
<img width="740" height="454" alt="image" src="https://github.com/user-attachments/assets/482a268d-a5c6-4ee1-ae75-5b7dd8ea2294" />

How does the robot work ?
1) How does the robot requilibrate itself ?
2) Movement equations

## Modélisation du robot auto-équilibré

### Paramètres physiques

| Symbole | Description | Valeur |
|---|---|---|
| $m_b$ | Masse du corps (body) | $0.2445\ \text{kg}$ |
| $m_w$ | Masse d'une roue | $0.0343\ \text{kg}$ |
| $J_b$ | Inertie du corps | $0.000255\ \text{kg·m}^2$ |
| $J_w$ | Inertie d'une roue | $0.000021\ \text{kg·m}^2$ |
| $R$ | Rayon de la roue | $0.035\ \text{m}$ |
| $l$ | Distance axe des roues → centre de gravité | $0.035\ \text{m}$ |
| $g$ | Accélération de la pesanteur | $9.81\ \text{m/s}^2$ |
| $\theta$ | Angle d'inclinaison du corps par rapport à la verticale | — |
| $x$ | Position horizontale du robot | — |
| $T_w$ | Couple appliqué sur chaque roue (entrée) | — |

### Équations du mouvement

Accélération linéaire :

```math
\ddot{x} = \frac{m_b l \sin(\theta)\,\dot{\theta}^2 - \dfrac{(m_b l)^2 g \cos(\theta)\sin(\theta)}{m_b l^2 + J_b} + 2T_w\left(\dfrac{1}{R} + \dfrac{m_b l \cos(\theta)}{m_b l^2 + J_b}\right)}{m_b + 2\left(m_w + \dfrac{J_w}{R^2}\right) - \dfrac{(m_b l \cos(\theta))^2}{m_b l^2 + J_b}}
```

Accélération angulaire :

```math
\ddot{\theta} = \frac{m_b l \sin(\theta)\left(g + \dfrac{m_b l \cos(\theta)\,\dot{\theta}^2}{m_b + 2\left(m_w + \frac{J_w}{R^2}\right)}\right) - 2\left(R + \dfrac{m_b l \cos(\theta)}{m_b + 2\left(m_w + \frac{J_w}{R^2}\right)}\right)\dfrac{T_w}{R}}{J_b + m_b l^2 - \dfrac{(m_b l \cos(\theta))^2}{m_b + 2\left(m_w + \frac{J_w}{R^2}\right)}}
```

Pour alléger l'écriture, on peut poser :

```math
M = m_b + 2\left(m_w + \frac{J_w}{R^2}\right), \qquad J = m_b l^2 + J_b
```

ce qui donne :

```math
\ddot{x} = \frac{m_b l \sin(\theta)\,\dot{\theta}^2 - \dfrac{(m_b l)^2 g \cos(\theta)\sin(\theta)}{J} + 2T_w\left(\dfrac{1}{R} + \dfrac{m_b l \cos(\theta)}{J}\right)}{M - \dfrac{(m_b l \cos(\theta))^2}{J}}
```

```math
\ddot{\theta} = \frac{m_b l \sin(\theta)\left(g + \dfrac{m_b l \cos(\theta)\,\dot{\theta}^2}{M}\right) - 2\left(R + \dfrac{m_b l \cos(\theta)}{M}\right)\dfrac{T_w}{R}}{J - \dfrac{(m_b l \cos(\theta))^2}{M}}
```

> Source des équations : [Theory and design of two wheels self-balancing robot](https://fkeng.blogspot.com/2019/03/theory-and-design-of-two-wheels-self.html)

### Implémentation (Python)

```python
import math

xdd = (mb*l*math.sin(theta)*(thetad**2) - ((mb*l)**2)*g*math.cos(theta)*math.sin(theta)/(mb*(l**2)+Jb) +
       2*Tw*(1/R + mb*l*math.cos(theta)/(mb*(l**2)+Jb))) / (mb + 2*(mw+Jw/(R**2)) - ((mb*l*math.cos(theta))**2)/(mb*(l**2) + Jb))

thetadd = (mb*l*math.sin(theta)*(g + mb*l*math.cos(theta)*(thetad**2)/(mb + 2*(mw + Jw/(R**2)))) -
           2*(R + mb*l*math.cos(theta)/(mb + 2*(mw + Jw/(R**2))))*Tw/R) / (Jb + mb*(l**2) - ((mb*l*math.cos(theta))**2)/(mb + 2*(mw + Jw/(R**2))))
```

### Contrôle PD

Correcteur proportionnel-dérivé avec dérivée filtrée :

```math
C(s) = k_p + k_d \, \frac{s}{\tau s + 1}
```

| Gain | Valeur |
|---|---|
| $k_p$ | $1$ |
| $k_d$ | $0.1$ |
| $\tau$ | $0.03$ |

L'erreur est calculée comme $e = \theta_{ref} - \theta$, multipliée par un gain de $-1$, puis envoyée au correcteur dont la sortie est le couple $T_w$ appliqué au robot.
