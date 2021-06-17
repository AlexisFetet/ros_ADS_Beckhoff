Ce package ROS comporte un noeud serveur ayant pour but de faciliter la communication entre un environnement ROS et un automate Beckhoff.

**Présentation**

"beckhoff_plc_control" comporte 3 services :
```
/ADS_read_value : Lis la valeur d'une variable
/ADS_write_value : Ecris une valeur dans une variable
/ADS_read_variables : Lis toutes les variables de votre programme automate
```

**Installer le package**

Télécharger ce package ROS et l'ajouter dans votre répertoire `catkin_ws/src/`

**Utiliser le package**

- Ajouter à votre fichier launch le lancement du noeud serveur en renseignant vos paramètres IP :
```
<node pkg="beckhoff_plc_control" type="RosAds_server" name="RosAds_server" output="screen">
        <param name="localNetId" type="str" value="{255, 255, 255, 255, 1, 1}"/>
        <param name="remoteNetId" type="str" value="255, 255, 255, 255, 1, 1"/>
        <param name="remoteIpV4" type="str" value="255.255.255.255"/>
        <param name="PLCPathFile" type="str" value=""$(find beckhoff_plc_control)/config/your_file.tpy""/>
</node>
```
