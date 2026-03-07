flowchart TD
 %% Initialisation (sous-programme)
 Start([Début]) --> Init[[Initialisation]]
 Init --> CheckButton{Button Start\npressé ?}
 CheckButton -->|Non| CheckButton

 %% Logique principale
 CheckButton -->|Oui| CheckObstacle{Obstacle détecté ?}
 CheckObstacle -->|Oui| CheckObstacle
 CheckObstacle -->|Non| CheckBoth{Capteurs gauche ET\n droit == 0 ?}
 CheckBoth -->|Oui| Stop([Arrêt])
 CheckBoth -->|Non| CheckLeft{Capteur gauche == 0 ?}

 %% Branches gauche/droite (sous-programmes)
 CheckLeft -->|Oui| SubLeft[[À gauche]]
 CheckLeft -->|Non| CheckRight{Capteur droit == 0 ?}
 CheckRight -->|Oui| SubRight[[À droite]]
 CheckRight -->|Non| MoveForward[Avancer]

 %% Sous-programmes (retour à la boucle principale)
 SubLeft --> MoveForward
 SubRight --> MoveForward
 MoveForward --> CheckObstacle

 %% Fin de l'algorithme
 Stop --> End([Fin])

 %% Styles ISO 5807
 style Init fill:#f9f,stroke:#333,stroke-width:2px
 style SubLeft fill:#f9f,stroke:#333,stroke-width:2px
 style SubRight fill:#f9f,stroke:#333,stroke-width:2px
