# 2nd

## Loeng 2

**Q:** Ei saa hetkel igast GNSS error sourcest hästi aru (ilmselt see ongi see osa, kus lektor räägib juurde). Ei saa aru, mis on orbit error, receiver noise ja multipath.

**Q:** Kuidas saab DGNSS puhul kindel olla, et sõidukil on samad errorid, mis baasjaamal?

**Q:** Mida kujutatakse error correction methods slaidil x-teljel (mida baseline tähendab)?

**Q:** Ei saa hästi aru, mis ülesanne on IMU-l (Inertial Measurement Unit) auto täpse asukoha tagamisel? Mida kujutab endast Sensor Fusion skeemil Drifting INS solution? Samuti, kui GNSS+IMU=INS, siis mis on GNSS+INS?

**Q:** Miks on ROS coordinate frames slaidil mingid asjad maha kriipsutatud? Ei saa ka üldiselt hästi aru, mida sellel slaidil kujutatud on.

**Q:** Mida mõeldakse Lexus important coordinate frames slaidil arbitrary point on map all? Kuidas ja mille järgi see valitakse?

**Q:** Mida tähendab coordinate transforms 1. slaidil homogeneous coordinates? Ja mida tähistatakse selle slaidi ülemises paremas nurgas oleval joonisel? Ei ole ka päris kindel, mida transform üldiselt tähendab (kas see on keha asukoht koos suunaga vaadeldavas ruumis)?

Kui ma sain õigesti aru, siis GNSS localization slaidil olevatele küsimustele vastamiseks tuleb vaadata esmalt, mis publisher on topicul /localization/current_pose ning seejärel, mis inbound connectionid on nodel /localization/novatel_oem7_localizer. Slaidil on küsimus, et mis sensor topicult saab see node infot. Ma leidsin, et sellel nodel on suisa 5 inbound ühendust, nendest sensorite ühendused on minu arusaamise järgi /novatel/oem7/bestpos, /novatel/oem7/inspva ja /gps/imu. Kuna see node saab info mitmelt sensorilt (kui ma ei eksi), siis võiks küsimused 2 ja 3 olla slaidil sõnastatud segaduse vältimiseks mitmuses ("From which sensor topics..." ja "What are the frequencies of the corresponding sensor topics?").

Proovisin `roswtf` käsku, mis andis ühe errori:

``` ERROR Not all paths in PYTHONPATH [/home/rauno/autoware_mini_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/rauno/carla_root/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:/home/rauno/carla_root/PythonAPI/carla/agents:/home/rauno/carla_root/PythonAPI/carla] point to a directory:
 * /home/rauno/carla_root/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg
```

Vaatasin ka, et errori saab ära nii, kui teha `cd /home/rauno/carla_root/PythonAPI/carla/dist/`, seejärel `unzip carla-0.9.13-py3.7-linux-x86_64.egg` ning seejärel muuta PYTHONPATH ~/.bashrc failis järgnevaks: `/home/rauno/autoware_mini_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/rauno/carla_root/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:/home/rauno/carla_root/PythonAPI/carla/agents:/home/rauno/carla_root/PythonAPI/carla`.
Kui peale neid fixe avada terminal uuesti ning teha uuesti `roswtf`, siis erroreid enam ei tule.
**NB! PYTHONPATH-i vastav muutmine teeb Carla simulatsioonide launchimise katki, seega ei tohiks ikkagi PYTHONPATH-i selliselt muuta!**  Pole muidugi kindel, kas see error üldse vajab fiximist, sest hetkel tundus, et see midagi ei mõjutanud.

## Praktikum 2

Praksilahenduste sidumisel autoware_mini repoga võiks juhendisse lisaks kirja panna selle, et `~/.bashrc` failis ära võtta (või välja kommenteerida) eelnev `devel/setup.bash` kirje, sest `source ~/autoware_mini_ws/devel/setup.bash` ja `source ~/autoware_mini_practice/devel/setup.bash` lähevad omavahel konflikti (selle tulemusel tekkis selline viga, et `roslaunch autoware_mini_practice_solutions practice_2.launch` viskas viga, et ei leita üles node `localizer.py`).

Publish pose sektsioonis on valideerimise näidistulemustes vist position.z vale, sest mina sain `msg.height - self.undulation` (see, kuidas juhendi järgi peaks position.z arvutama) tulemusteks arvud 34 ja 38 vahel (juhendi näidistulemustes on see 0, mul ei tulnud see kordagi 0, ei tahaks uskuda ka, et see peaks 0 olema).
