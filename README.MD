
Exemple signalement drone TTGO T-BEAM
=====================================

Ce dépôt est un code d'exemple de la librairie pour le signalement drone avec une carte TTGO T-BEAM.
Le code repose sur l'utilisation du framework Arduino pour la puce ESP32 qui va générer les trames Wifi.

La documentation de la carte TTGO T-BEAM peut etre trouvé sur http://www.lilygo.cn/prod_view.aspx?Id=1265
La carte permet de réaliser un module d'identification autonome. En effet, la carte possède un puce Wifi (ESP32), un GNSS (ublox Neo6), une puce LORA (inutile pour l'identification drone), une prise batterie (Li-Ion 18550).

Cette exemple peut être généralisé sur la majorité des cartes basé sur l'ESP32 avec quelques adaptations par rapport aux périphériques disponible.

Auteurs
-------
Ce code a été initialement écrit et publié par Pierre Kancir avec le soutien de Airbot Systems en mai 2020.

Licence
-------

Tout le code est sous licence GPL https://www.gnu.org/licenses/ (ou synthétiquement : https://choosealicense.com/licenses/gpl-3.0/).
En simplifiant **les obligations**:
- Le code est libre dans son utilisation et sa distribution.
- Vous pouvez l'inclure dans vos projets, mais cela étend la licence GPL à votre projet.
- Si vous vendez un module basé sur ce code, vous devez mentionner clairement à votre client que le code est open source et lui proposer un accés au code source, y compris vos modifications du code originel.
- Le code ne peut pas être reliciencié.

# Installation
Je recommande l'utilisation de [Platformio](https://platformio.org/) pour la programmation de la carte.
L'IDE Arduino peut aussi être utilisé mais vous devrez gérer les dépendance à la main.

## PlatformIo
Suivez les instructions d'installation sur https://platformio.org/platformio-ide

## Arduino
Installer l'IDE Arduino : https://www.arduino.cc/ . Installer le support des cartes ESP32 pour Arduino : https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md .
Télécharger les zip des librairies mentionné dans le fichier `platformio.ini` et ajouter les avec le gestionnaire de bibliothèques.

## Librairie droneID_FR
La version la plus récente de la librairie droneID_FR peut être trouvé sur https://github.com/khancyr/droneID_FR
Copiez simplement la librairie droneID_FR (le fichier droneID_FR.h) dans le dossier src.

# Utilisation

## PlatformIo
### Compilation
Maintenant que l’environnement de compilation est installé, placez vous dans le dossier ou vous avez mis le dépot et utilisez :

    platformio run

pour lancer la compilation du programme. Platformio va se charger de télécharger les librairies et outils nécessaires à la compilation.
La liste des dépendances externe peut être trouvée dans le fichier `platformio.ini`.
Dans le cas ou vous utilisez l'IDE de platformio, appuyez sur le bouton compiler.

### Téléversement
Utilisez:

    platformio run -t upload


## Arduino
### Compilation
Maintenant que l’environnement de compilation est installé, ouvrez le dossier du dépot avec l'IDE et appuyez sur le bouton vérifier.

### Téléversement
Utilisez le bouton Téléverser.

# Explication du code
Le code est basé sur le code d'exemple de la carte TTGO T-BEAM https://github.com/Xinyuan-LilyGO/LilyGO-T-Beam .

Il est composé de 3 fichiers:
- `board_def.h` : fichier de configuration de la carte.
- `LilyGO-T-Beam.ino` : fichier principal qui contient le code qui va être exécuté.
- `droneID_FR.h` : libraire pour généré la trame d'identification du drone.

Ma carte est un v0.7 (c'est marqué dessus), veuillez ajuster dans `board_def.h` la version de votre carte.
Par défaut, seul le GPS est utilisé. La carte possède aussi un module de transmission LORA. Vous pouvez l'utiliser pour connecter votre carte à un réseau LORA ouvert comme : https://www.thethingsnetwork.org/ pour tracker votre carte depuis internet (utile quand on paume son drone).

La librairie `droneID_FR.h` sert a créér la frame wifi d'identification. Elle continent plusieurs méthodes pour passer les données et les formater correctement.

Le code principal est dans `LilyGO-T-Beam.ino`. Ce code utilise le framework arduino par simplicité : il y a donc deux fonctions principales :
- `setup()` : qui fait la configuration de la carte et de ses périphériques.
- `loop()` : qui est une boucle infinie qui, dans notre cas, sert pour recupérer les données du GNSS, formate les données pour la trame d'identification, envoi la trame d'identification.

Ainsi le code configure la carte pour activer la puce GPS et la puce Wifi. Ensuite on récuppère les données GPS. Si elles sont jugé valides, on les transmet à la librairies droneID_FR pour formatter les données et récupper la trame beacon. Lorsqu'il est temps d'envoyer la trame : soit toutes les 3s, soit après 30m parcouru, on envoie la trame.

## Utilisation
Par default, ça émet sur un réseau wifi nommé : "ILLEGAL_DRONE_AP" et envoi une trame d'identification drone avec l'identifiant : "ILLEGAL_DRONE_APPELEZ_POLICE17".
Veuillez changer le nom du réseau wifi et l'ID du drone.

Le reste des valeurs ne devrait pas avoir besoin d'ajustement si vous utilisez la carte TTGO T-Beam !

