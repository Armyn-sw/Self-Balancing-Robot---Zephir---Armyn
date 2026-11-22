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

### Liste des courses : robot auto-équilibré

> Les liens mènent à des **recherches** sur Amazon.fr, pour que vous puissiez comparer les vendeurs et les prix.
> Pensez à vérifier les cotes réelles des pièces reçues avant de modéliser le châssis.

## Électronique (cerveau et capteurs)

| NOM | Utilité | Lien |
|---|---|---|
| ESP32 DevKit | Microcontrôleur qui fait tourner l'algorithme d'équilibre (PID). Bluetooth et Wi-Fi intégrés pour piloter ou régler depuis un téléphone | [Rechercher](https://www.amazon.fr/s?k=ESP32+DevKit) |
| MPU6050 (x2) | Gyroscope et accéléromètre qui mesurent l'inclinaison du robot. Le 2e sert de rechange | [Rechercher](https://www.amazon.fr/s?k=MPU6050) |

## Motorisation : option A, moteurs pas à pas (conseillée pour débuter)

| NOM | Utilité | Lien |
|---|---|---|
| Moteur NEMA17 (x2) | Font tourner les roues avec précision, sans encodeur | [Rechercher](https://www.amazon.fr/s?k=moteur+NEMA17) |
| Driver TMC2209 (x2) | Pilotent les moteurs pas à pas, de façon silencieuse | [Rechercher](https://www.amazon.fr/s?k=TMC2209+driver) |
| Coupleur / moyeu 5 mm (x2) | Fixent les roues sur l'axe des moteurs | [Rechercher](https://www.amazon.fr/s?k=moyeu+roue+axe+5mm) |

## Motorisation : option B, moteurs DC (plus léger)

| NOM | Utilité | Lien |
|---|---|---|
| Moteur JGA25-370 avec encodeur (x2) | Font tourner les roues. L'encodeur mesure la vitesse réelle | [Rechercher](https://www.amazon.fr/s?k=JGA25-370+encodeur) |
| Driver TB6612FNG | Pilote les deux moteurs DC | [Rechercher](https://www.amazon.fr/s?k=TB6612FNG) |
| Coupleur / moyeu 4 mm (x2) | Fixent les roues sur l'axe des moteurs | [Rechercher](https://www.amazon.fr/s?k=moyeu+roue+axe+4mm) |

## Alimentation

| NOM | Utilité | Lien |
|---|---|---|
| Batterie LiPo 3S 11,1 V 1500–2200 mAh | Alimente tout le robot | [Rechercher](https://www.amazon.fr/s?k=batterie+lipo+3s+2200mah) |
| Chargeur LiPo équilibreur (B6 ou ISDT) | Recharge la batterie en sécurité | [Rechercher](https://www.amazon.fr/s?k=chargeur+lipo+equilibreur) |
| Convertisseur abaisseur 5 V (LM2596) | Fournit du 5 V à l'ESP32 et aux capteurs à partir de la batterie | [Rechercher](https://www.amazon.fr/s?k=LM2596+abaisseur) |
| Interrupteur | Allume et éteint le robot | [Rechercher](https://www.amazon.fr/s?k=interrupteur+a+bascule+12V) |
| Connecteurs XT60 | Branchent la batterie | [Rechercher](https://www.amazon.fr/s?k=connecteur+XT60) |
| Alarme de tension LiPo | Sonne quand la batterie est presque vide, pour ne pas l'abîmer | [Rechercher](https://www.amazon.fr/s?k=alarme+tension+lipo) |

## Connectique

| NOM | Utilité | Lien |
|---|---|---|
| Plaque de prototypage (perfboard) | Support pour souder et organiser les composants | [Rechercher](https://www.amazon.fr/s?k=plaque+prototypage+perfboard) |
| Fils Dupont | Connexions rapides pendant les tests | [Rechercher](https://www.amazon.fr/s?k=fils+dupont) |
| Fil silicone 18–22 AWG | Câblage de puissance (batterie et moteurs) | [Rechercher](https://www.amazon.fr/s?k=fil+silicone+18+AWG) |
| Gaine thermorétractable | Isole les soudures | [Rechercher](https://www.amazon.fr/s?k=gaine+thermoretractable) |
| Condensateurs 100 µF | Stabilisent l'alimentation des drivers moteurs | [Rechercher](https://www.amazon.fr/s?k=condensateur+100uF+25V) |

## Mécanique et impression 3D

| NOM | Utilité | Lien |
|---|---|---|
| Filament PETG (ou PLA) | Impression du châssis. Le PETG résiste mieux aux chutes | [Rechercher](https://www.amazon.fr/s?k=filament+PETG+1.75) |
| Filament TPU | Impression des pneus pour avoir de l'adhérence | [Rechercher](https://www.amazon.fr/s?k=filament+TPU+1.75) |
| Roues 65–80 mm (si vous ne les imprimez pas) | Alternative aux roues imprimées | [Rechercher](https://www.amazon.fr/s?k=roue+robot+65mm) |
| Kit vis M3 + écrous | Assemblage du châssis | [Rechercher](https://www.amazon.fr/s?k=kit+vis+M3) |
| Inserts filetés à chaud M3 | Filetages solides dans les pièces imprimées | [Rechercher](https://www.amazon.fr/s?k=inserts+filetés+M3) |
| Entretoises M3 | Séparent les étages du châssis | [Rechercher](https://www.amazon.fr/s?k=entretoises+M3+kit) |

**Budget estimé :** environ 80–120 € sans le chargeur (les prix varient selon les vendeurs).
