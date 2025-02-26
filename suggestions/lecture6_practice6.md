# 6th

## Loeng 6

**Viga slaididel:** Mitmetel slaididel tuleks ära vahetada `autoware_msgs` `autoware_mini/msg` vastu. Sellised slaidid on 3, ...

## Praktikum 6

**Bug:** 1. sektsiooni näidiskoodis peaks importima `autoware_mini.msg` teegist. Samuti tuleks seetõttu näidiskoodis igal pool ära vahetada `Lane` `Path` vastu. Samuti peaks olema `w.pose.pose.position.x` asemel `w.position.x` (sama kehtib ka y ja z koordinaatide puhul). Samuti on kiiruse jaoks `w.twist.twist.linear.x` asemel vaja hoopis `w.speed`. Samuti on `Path` objektil hoopis isendiväli `stopping_point_distance`, mitte enam `cost`, seetõttu tuleks koodi muuta veel nii: `lane.cost` asemel kirjutada `lane.stopping_point_distance`.

**Bug:** `practice_6_sim.launch` ja `practice_6_bag.launch` failidest on puudu:
`<arg name="enable_auto_stop_checker"    default="true" />`
ning
`<param name="enable_auto_stop_checker" value="$(arg enable_auto_stop_checker)" />`

**Bug:** Etteantud `planning.yaml` failist on puudu `safety_box_width: 2.7` ja `stopped_speed_limit: 1.0`.

**Bug:** Sama bug, mis oli 5. praktikumis:
Kuna `autoware_mini` repos ei eksisteeri enam faili `ground_removal.py` (viskas seetõttu ka errori, kui tegin `roslaunch`), siis otsustasin selle asendada `naive_ground_removal.py` failiga (oletan, et prakside raames ei kasutata `jcp_ground_removal.py` varianti). Kuna `naive_ground_removal.py` nõuab ka mitmeid teistsuguseid parameetreid võrreldes juhendis antud `detection.yaml` failiga, siis otsustasin `practice_6_bag.launch` ja `practice_6_sim.launch` failides viidata `autoware_mini` enda `detection.yaml` failile (ehk ei kasuta juhendis etteantud varianti). Selle jaoks muutsin `detection` nimeruumi `practice_6_bag.launch` failis järgmiseks:

``` launch
<!-- Detection -->
<group ns="detection">
    <group ns="lidar">
        <!-- Ground removal -->
        <group ns="center">
            <node pkg="autoware_mini" type="naive_ground_removal.py" name="naive_ground_removal" output="screen" required="true">
                <remap from="points_raw" to="/lidar_center/points_raw" />
            </node>
        </group>
        <!-- Filtering -->
        <node pkg="nodelet" type="nodelet" name="pcl_manager" args="manager" output="screen" required="true" />
        <node pkg="nodelet" type="nodelet" name="voxel_grid_filter" args="load pcl/VoxelGrid pcl_manager" output="screen" required="true">
            <remap from="~input" to="/detection/lidar/center/points_no_ground" />
            <remap from="~output" to="/detection/lidar/points_filtered" />
        </node>
        <!-- Point clusterer -->
        <node pkg="autoware_mini_practice_solutions" type="points_clusterer.py" name="points_clusterer" output="screen" required="true" />

        <node pkg="autoware_mini_practice_solutions" type="cluster_detector.py" name="cluster_detector" output="screen" required="true" />

        <node pkg="topic_tools" type="relay" args="detected_objects /detection/detected_objects" name="detected_objects_relay" output="screen" required="true" />
    </group>

    <group if="$(arg use_tracking)">
        <node pkg="autoware_mini" type="ema_tracker.py" name="ema_tracker" output="screen" required="true" />
        <node pkg="autoware_mini" type="detected_objects_visualizer.py" name="tracked_objects_visualizer" output="screen" required="true">
            <remap from="detected_objects" to="tracked_objects" />
            <remap from="detected_objects_markers" to="tracked_objects_markers" />
        </node>
            <node pkg="topic_tools" type="relay" args="tracked_objects final_objects" name="detected_objects_relay" output="screen" required="true" />
    </group>
    <node unless="$(arg use_tracking)" pkg="topic_tools" type="relay" args="detected_objects /detection/final_objects" name="detected_objects_relay" output="screen" required="true" />

    <node pkg="autoware_mini" type="detected_objects_visualizer.py" name="final_objects_visualizer" output="screen" required="true">
        <remap from="detected_objects" to="final_objects" />
        <remap from="detected_objects_markers" to="final_objects_markers" />
    </node>

    <!-- Config -->
    <!-- <rosparam command="load" file="$(find autoware_mini_practice_solutions)/config/detection.yaml"/> -->
    <rosparam command="load" file="$(find autoware_mini)/config/detection.yaml"/> 

</group>
```

ning `practice_6_sim.launch` failis järgmiseks:

``` launch
<!-- Detection -->
<group ns="detection">

    <node pkg="autoware_mini" type="obstacle_simulation.py" name="obstacle_simulation" output="screen" required="true" />

    <node pkg="topic_tools" type="relay" args="detected_objects /detection/final_objects" name="detected_objects_relay" output="screen" required="true" />

    <node pkg="autoware_mini" type="detected_objects_visualizer.py" name="final_objects_visualizer" output="screen" required="true">
        <remap from="detected_objects" to="final_objects" />
        <remap from="detected_objects_markers" to="final_objects_markers" />
    </node>

    <!-- Config -->
    <!-- <rosparam command="load" file="$(find autoware_mini_practice_solutions)/config/detection.yaml"/> -->
    <rosparam command="load" file="$(find autoware_mini)/config/detection.yaml"/> 

</group>
```

Juhendis võiks olla 4. ülesande juures vihjena mainitud, et muuta tuleb local path waypointide kiiruseid.

**Viga juhendis:** 6. sektsioonis viidatud link annab error 404. Ehk `ema_tracker.py` node enam ei eksisteeri või on see kuskil mujal.

Juhendi README-s on mõned vormistuslikud vead.

TODO alates 6.5
