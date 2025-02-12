# 3rd

## Loeng 3

See on suhteliselt keeruline loeng ning ilma lektorita võtab omajagu aega seedimiseks.
Passing through target point slaidil olevad joonised tekitasid algul segadust. Ilmselt see on jälle koht, kus lektor peatub pikemalt ning selgitab joonise põhjalikult lahti.

Mulle meeldis see, et slaididel on all viited pikematele artiklitele, millest võib keeruliste slaidide ja teemade puhul abi olla. Ehk võib ka tudengeid julgustada keerulisemate teemade puhul vaatama neid linke.

## Praktikum 3

**Bug:** Probleem 1. ja 2. sektsiooni juures etteantud failidega: failis `~/autoware_mini_ws/src/autoware_mini/nodes/planning/waypoints/waypoint_loader.py` pidi käsu `roslaunch autoware_mini_practice_solutions practice_3.launch` toimimiseks välja kommenteerima read 49 (gid) ja 60 (change_flag).

**Bug:** Probleem 2. sektsiooni juures etteantud failidega: `waypoint_loader` nodel ei ole ühendust `pure_pursuit_follower` nodega. Probleemi lahendamiseks peab failis `pure_pursuit_follower.py` muutma 5. rea impordi järgnevaks: `from autoware_mini.msg import Path`. Samuti tuleb siis seejärel asendada kõik `Lane` kirjed koodis asendada kirjega `Path`.

**Viga juhendis:** Sektsioonis 3 peaks olema message type hoopis `autoware_mini.msg/VehicleCmd` (mitte `autoware_msg/VehicleCmd`).

**Bug:** Sektsioonis 4 etteantud muutuja `path_linestring` on vigane. See peaks olema tegelikult selline: `path_linestring = LineString([(w.position.x, w.position.y) for w in msg.waypoints])`. Samalaadne viga on ka 6. sektsioonis `waypoints_xy` muutuja väärtustamisel.

**Viga juhendis:** Sektsioonis 4 punktis 5 peaks viimane lause olema hoopis "Fix it by checking if the value is not set the `path_callback` should return.", sest on vaja kontrollida, et `path_callback` funktsioon oleks `self.path_linstring` (või mis iganes nime tudeng määras klassimuutujale) muutujat uuendanud (et see poleks enam `None`).

**Bug:** Sektsioonis 6 antud koodis peaks `velocities` muutuja olema väärtustatud hoopis nii: `velocities = np.array([w.twist.twist.linear.x for w in msg.waypoints])`

Loengus või praktikumijuhendis võiks olla mainitud ka `threading.Lock` funktsiooni. Seda tuleks kasutada muutujate `self.path_linestring` ja `self.distance_to_velocity_interpolator` väärtustamisel, et need saaksid korraga väärtused.
