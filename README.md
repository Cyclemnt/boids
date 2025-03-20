## **Prédation**
&nbsp;&nbsp;&nbsp;&nbsp;Cette extension enrichit la simulation classique de boids en introduisant deux nouveaux types
d'entités,  
toutes issues de la classe Boid, offrant des interactions complexes et équilibrées, il y a donc
maintenant  
trois types d’entités.  


<p align="center">
    <img src="https://github.com/user-attachments/assets/3881d621-200f-4d4a-a3fe-123c97a08535" width="500">
</p>


### **Les nouvelles entités et leurs rôles**

1. **Les boids non-agressifs (boids classiques)** :
   - *fled* : Lorsqu’un predator est détecté dans leur champ de vision instinctif (*instinct*), les boids fuient avec un boost temporaire de vitesse.
   - *catch* : Interaction avec les predators lorsque ces derniers capturent un boid, ce qui déclenche des comportements spécifiques (comme la suppression du boid capturé).
   - *feed* : Permet aux boids de consommer des *foods*, déclenchant ainsi l'interaction *breed*, augmentant leur population.

2. **Les predators** :
   - *Prédation* : Chassent les boids en les détectant dans leur champ de vision (*FOV*). Lorsqu’un boid est capturé, celui-ci se transforme en predator.
   - *Instinct* : Les predators évitent les collisions entre eux.
   - *Durée de vie limitée* : Lorsqu’un predator atteint sa durée de vie maximale, il se transforme en *food*.

3. **Les foods** :
   - Entités statiques qui n'ont ni vitesse ni comportement de déplacement.
   - Seule interaction : *breed*, permettant aux boids de se reproduire en consommant ces ressources.

### **Méthodes supplémentaires**

- `Simulation::removeThisBoid` : Supprime un boid spécifique de la simulation (consommé par un predator).
- `Simulation::addPredator` : Ajoute un predator à une position donnée avec des paramètres personnalisés.
- `Simulation::removePredator` : Supprime le dernier predator ajouté.
- `Simulation::removeThisPredator` : Supprime un predator spécifique (par exemple, à la fin de sa durée de vie).
- `Simulation::initializePredatorsRandomly` : Initialise des predators à des positions et orientations aléatoires.
- `Simulation::addFood` : Ajoute une source de nourriture à une position donnée.
- `Simulation::removeThisFood` : Supprime une source de nourriture spécifique.
- `Zone::isWithinInstinct` : Vérifie si un predator est dans le champ de vision instinctif d’un boid.

### **Intérêts de l’extension**

- **Régulation de la population** : Les predators limitent la surpopulation des boids.
- **Cycle écologique dynamique** : Les *foods* permettent aux boids de se reproduire, maintenant un équilibre naturel.
- **Transformation des predators en food** : Ajoute une boucle de régénération des ressources, garantissant la stabilité des populations.

### **Quelques exemples**

- **Prédation et duplication des predators** : Lorsqu’un boid est capturé, il devient predator.
- **Transformation d'un predator en food** : Lorsqu’il atteint la fin de sa durée de vie.
- **Duplication de boids en consommant food** : Maintien de la population par reproduction.
- **Vue d'ensemble de la simulation** : Illustration du cycle de vie des entités dans l'écosystème simulé.
