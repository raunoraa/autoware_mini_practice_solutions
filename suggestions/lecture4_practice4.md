# 4th

## Loeng 4

**Q:** Mida kujutab endast localization map?

**Viga slaidil:** Debugging global planner in RViZ: Smoothed path optionit ei ole enam planningu all. Ei leidnud ka, et seda oleks võimalik eraldi juurde lisada.

## Praktikum 4

**Viga juhendis:** Sissejuhatuse tekstilõigu 4. lause peaks algama hoopis nii: "The path should then be converted to a `Path` message ...". Samalaadseid vigu, kus `Lane` peaks olema vahetatud `Path` vastu, esineb juhendis veel.

**Bug:** 3. praktikumis antud `planning.yaml` fail ei ole selle praktikumi jaoks piisav (juhendis ei ole selle täiendamist ka nõutud). Puudu on järgnev:

``` yaml
local_path_length: 100
lanelet2_map_visualizer:
    use_map_extraction: False     # extract smaller map area around the ego vehicle for visualization
    map_extraction_distance: 500  # m - distance from ego vehicle to extract map data
```

Samuti on ka `practice_4.launch` failist puudu:
`<arg name="enable_auto_stop_checker"    default="true" />`
ning
`<param name="enable_auto_stop_checker" value="$(arg enable_auto_stop_checker)" />`

**Bug:** 3. sektsiooni 4. punktis on viidatud Autoware'i `Waypoint`-ile, kuid tegelikult kasutatakse autoware_mini `Waypoint`-e. Seega tuleks selles sektsioonis muuta link selliseks, mis viitaks autoware_mini `Waypoint` definitsioonile. Samuti erineb autoware_mini `Waypoint` natuke Autoware'i `Waypoint`-ist. Õige on nii: `waypoint.position.x` ja `waypoint.speed`.
