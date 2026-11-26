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

> Mise à jour du 26.11.2026 : passage aux **moteurs DC avec encodeur** (à la place des NEMA17), à la **LiPo 3S 1100 mAh** et au driver **TB6612FNG**.
> La plupart des pièces sont commandées sur **AliExpress** (livraison prévue du 4 au 17 décembre). Les liens mènent à des **recherches**, pour comparer les vendeurs.
> La **batterie LiPo** s'achète en Suisse (Galaxus, Planet-RC, magasin de modélisme) : c'est plus sûr et le transport des LiPo depuis la Chine est compliqué.
> Pensez à mesurer les pièces reçues (pied à coulisse) avant de modéliser le châssis.

Statut : Commandé / À acheter / En stock

## Électronique (cerveau et capteurs)

| NOM | Utilité | Statut | Lien |
|---|---|---|---|
| ESP32 DevKit V1, 30 broches, USB-C | Microcontrôleur : lit l'IMU, compte les encodeurs (PCNT), calcule la commande (LQR / PID) et pilote les moteurs. Wi-Fi et Bluetooth pour régler depuis un téléphone | Commandé | [Rechercher](https://www.aliexpress.com/w/wholesale-ESP32-30pin-type-c.html) |
| MPU6050 GY-521 (lot de 5) | Gyroscope + accéléromètre : mesurent l'inclinaison θ et la vitesse de rotation. Les autres servent de rechange | Commandé | [Rechercher](https://www.aliexpress.com/w/wholesale-MPU6050-GY-521.html) |
| Driver TB6612FNG (lot de 3, broches à souder) | Double pont en H qui pilote les deux moteurs (1,2 A continu, 3,2 A crête, VM max 13,5 V). **Pas de DRV8833** (max ~10,8 V, incompatible 3S) | Commandé | [Rechercher](https://www.aliexpress.com/w/wholesale-TB6612FNG.html) |

## Motorisation : moteurs DC avec encodeur

| NOM | Utilité | Statut | Lien |
|---|---|---|---|
| Moteur JGB37-520 **12 V 333 tr/min** avec encodeur Hall (x2, +1 de rechange conseillé) | Font tourner les roues. Réducteur 1:30 (même rapport que le modèle du PDF), encodeur A/B ≈ 1320 impulsions/tour de roue pour mesurer x et ẋ. **Vérifier la vitesse (333 RPM) et le câble à 6 fils** | Commandé | [Rechercher](https://www.aliexpress.com/w/wholesale-JGB37-520-encoder-12V-333RPM.html) |
| Moyeu / coupleur **6 mm** (x2) | Fixent les roues imprimées sur l'arbre en D de 6 mm. Un trou imprimé prend du jeu, d'où un moyeu métallique | À acheter | [Rechercher](https://www.aliexpress.com/w/wholesale-6mm-flange-coupling.html) |

Câblage des moteurs (fils de l'encodeur) : rouge = moteur +, blanc = moteur −, bleu = encodeur + (3V3), noir = encodeur − (GND), jaune = canal A, vert = canal B.

## Alimentation

| NOM | Utilité | Statut | Lien |
|---|---|---|---|
| Batterie LiPo **3S 11,1 V 1100 mAh** (≥ 20C) | Alimente tout le robot (~1 h d'autonomie). Moteurs directement via le driver, électronique via le buck. **Acheter en Suisse**, vérifier le connecteur et les dimensions | À acheter | [Galaxus](https://www.galaxus.ch/fr/search?q=lipo%203s%201100mah) |
| Chargeur iMaxRC B3 Pro, prise EU, **10 W** | Recharge la batterie en équilibrant les 3 cellules (3 × 800 mA, < 1C). Pas la version 20 W | Commandé | [Rechercher](https://www.aliexpress.com/w/wholesale-B3-pro-compact-balance-charger.html) |
| Convertisseur abaisseur **LM2596-5V** (sortie fixe) | Fournit du 5 V stable à l'ESP32 (VIN) à partir de la batterie. Mesurer la sortie avant de brancher | Commandé | [Rechercher](https://www.aliexpress.com/w/wholesale-LM2596-5V.html) |
| Interrupteur KCD1 rond, 2 broches (lot de 5) | Allume et éteint le robot (6 A). Trou de montage ≈ 20 mm dans le châssis | Commandé | [Rechercher](https://www.aliexpress.com/w/wholesale-KCD1-round-rocker-switch.html) |
| Connecteurs XT30 ou XT60 | Branchent la batterie. Prendre le même type que la batterie achetée | À acheter | [Rechercher](https://www.aliexpress.com/w/wholesale-XT30-connector.html) |
| Sac de charge ignifugé LiPo | Charge en sécurité (ne jamais charger sans surveillance) | À acheter | [Galaxus](https://www.galaxus.ch/fr/search?q=lipo%20safe%20bag) |
| Pont diviseur 33 kΩ / 10 kΩ | Mesure la tension batterie sur une entrée analogique de l'ESP32 (12,6 V → ~2,9 V). Couper les moteurs sous 9,9 V. Remplace l'alarme de tension | En stock | — |

## Connectique

| NOM | Utilité | Statut | Lien |
|---|---|---|---|
| Plaques à pastilles 5×7 cm (perfboard) | Support pour souder et organiser les composants | Commandé | [Rechercher](https://www.aliexpress.com/w/wholesale-perfboard-5x7.html) |
| Fils Dupont M-M, M-F, F-F | Connexions pendant les tests. Prévoir ~25 fils F-F | Commandé | [Rechercher](https://www.aliexpress.com/w/wholesale-dupont-cable.html) |
| Fil silicone 18–22 AWG | Câblage de puissance (batterie, interrupteur, moteurs) | À acheter | [Rechercher](https://www.aliexpress.com/w/wholesale-silicone-wire-18AWG.html) |
| Gaine thermorétractable 2:1 (assortiment 164 pcs) | Isole les soudures. 2 mm signaux, 3 mm moteurs, 4–6 mm batterie/XT | Commandé | [Rechercher](https://www.aliexpress.com/w/wholesale-heat-shrink-tube-assortment.html) |
| Condensateurs **470 µF 25 V** + 100 nF | Au plus près de VM/GND du TB6612 : absorbent les pointes de courant des moteurs | En stock | — |

## Mécanique et impression 3D

| NOM | Utilité | Statut | Lien |
|---|---|---|---|
| Filament PETG (ou PLA) | Châssis et supports moteur (équerres vissées en M3 sur la face du réducteur). Le PETG résiste mieux aux chutes | À acheter | [Rechercher](https://www.amazon.fr/s?k=filament+PETG+1.75) |
| Filament TPU | Pneus imprimés pour l'adhérence (sinon joint torique / élastique large) | À acheter | [Rechercher](https://www.amazon.fr/s?k=filament+TPU+1.75) |
| Roues 80–90 mm (si pas imprimées) | Alternative aux roues imprimées. 80–90 mm donnent ~1,4 m/s max en 3S | À acheter | [Rechercher](https://www.aliexpress.com/w/wholesale-robot-wheel-85mm-6mm.html) |
| Kit vis M3 + écrous | Assemblage du châssis et fixation des moteurs | À acheter | [Rechercher](https://www.amazon.fr/s?k=kit+vis+M3) |
| Inserts filetés à chaud M3 | Filetages solides dans les pièces imprimées | À acheter | [Rechercher](https://www.amazon.fr/s?k=inserts+filetés+M3) |
| Entretoises M3 | Séparent les étages du châssis | À acheter | [Rechercher](https://www.amazon.fr/s?k=entretoises+M3+kit) |

Modèle 3D du moteur pour concevoir les supports : chercher « JGB37-520 STEP » sur GrabCAD.

## Retiré de la liste (et pourquoi)

| NOM | Raison |
|---|---|
| Moteurs NEMA17 + drivers TMC2209 | Remplacés par des moteurs DC avec encodeur : plus légers (≈ 150 g contre ≈ 300 g), pas de courant permanent, et l'encodeur donne la distance parcourue (indispensable pour rouler 20 m) |
| Moyeux 5 mm | L'arbre des JGB37-520 fait 6 mm |
| Condensateurs 100 µF | Remplacés par 470 µF 25 V (déjà en stock) |
| Alarme de tension LiPo | Remplacée par la mesure de tension via l'ESP32 |

**Budget estimé :** environ 80–120 € sans le chargeur (les prix varient selon les vendeurs).
