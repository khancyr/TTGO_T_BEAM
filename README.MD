
Exemple signalement drone TTGO T-BEAM  
=====================================  
  
Ce depot est un code d'exemple de la librairie pour le signalement drone avec une carte TTGO T-BEAM.  
Le code repose sur l'utilisation du framework Arduino pour la puce ESP32 qui va générer les trames Wifi. 

La documentation de la carte TTGO T-BEAM peut etre trouvé sur http://www.lilygo.cn/prod_view.aspx?Id=1265
La carte permet de réaliser un module d'identification autonome. En effet, la carte possède un puce Wifi (ESP32), un GNSS (ublox Neo6), une puce LORA (inutile pour l'identification drone), une prise batterie (Li-Ion 18550). 

Cette example peut etre généralisé sur la majorité des cartes basé sur l'ESP32 avec quelques adaptations par rapport aux périphériques disponible.

Licence
-------

Tout le code est sous licence GPL https://www.gnu.org/licenses/ (ou synthétiquement : https://choosealicense.com/licenses/gpl-3.0/). 
En simplifiant **les obligations**: 
- Le code est libre dans son utilisation et sa distribution.
- Vous pouvez l'inclure dans vos projets mais cela étend la licence GPL à votre projet.
- Si vous vendez un module basé sur ce code, vous devez mentionner clairement à votre client que le code est open source et lui proposer un accés au code source, y compris vos modification du code originel.
- Le code ne peut pas être reliciencier.

Ce code a été initialement écrit et publier par Pierre Kancir avec le sponsoring de Airbot Systems en mai 2020.

# Installation
Je recommende l'utilisation de [Platformio](https://platformio.org/) pour la programmation de la carte.  
L'IDE Arduino peut aussi être utilisé.  

## PlatformIo
Suivez les instructions d'installation sur https://platformio.org/platformio-ide

## Arduino
TBD

# Utilisation

## PlatformIo
### Compilation
Maintenant que l’environnement de compilation est installé, utilisez:

    platformio run
pour lancer la compilation du programme. Platformio va se charger de télécharger les librairies et outils nécessaire à la compilation.

### Téléversement
Utilisez:

    platformio run -t upload


## Arduino
TBD

# Explication du code
Le code est basé sur le code d'exemple de la carte TTGO T-BEAM https://github.com/Xinyuan-LilyGO/LilyGO-T-Beam .

Il est composé de 3 fichiers:
- `board_def.h` : fichier de configuration de la carte.
- `LilyGO-T-Beam.ino` : fichier principal qui contient le code qui va être exécuté.
- `droneID_FR.h` : libraire pour généré la trame d'identification du drone.

Ma carte est un v0.7 (c'est marqué dessus), veuillez ajuster dans `board_def.h` la version de votre carte.
Par défaut, seul le GPS est utilisé. La carte possède aussi un module de transmission LORA. Vous pouvez l'utiliser pour connecter votre carte à un réseau LORA ouvert comme : https://www.thethingsnetwork.org/ pour tracker votre carte depuis internet (utile quand on pomme son drone).

La librairie `droneID_FR.h` sert a créér la frame wifi d'identification. Elle continent plusieurs méthodes pour passer les données et les formater correctement.

Le code principal est dans `LilyGO-T-Beam.ino`. Ce code utilise le framework arduino par simplicité: il y a donc deux fonctions principales:
- `setup()` : qui fait la configuration de la carte et de ses périphériques.
- `loop()` : qui est une boucle infini qui, dans notre cas, sert pour recupperer les données du GNSS, formate les données pour la trame d'identification, envoi la trame d'identification.

## Utilisation
Par default, ça émet sur un réseau wifi nommé : "ILLEGAL_DRONE_AP" et envoi une trame d'identification drone avec l'identifiant : "ILLEGAL_DRONE_APPELEZ_POLICE17".
Veuillez changer le nom du réseau wifi et l'ID du drone.

Le reste des valeurs ne devrait pas avoir besoin d'ajustement si vous utilisez la carte TTGO T-Beam !