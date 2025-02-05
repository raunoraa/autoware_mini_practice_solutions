# Autoware_mini installation guide related problems

* Enne ros update sammu laadisina alla autoware_misc (<https://gitlab.cs.ut.ee/autonomous-driving-lab/autoware.ai/local/autoware_misc>) repo, kus käivitasin autoware_misc/03_astuff_repos.sh skripti, seejärel lisasin chatpgt käsuga võtme (sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 6390D5EC6C3076CE) ning seejärel käivitasin autoware_misc/03_astuff_repos.sh skripti uuesti

* Lisaks cuda 11.8 tuleb alla laadida ka cuda 12.2. Ehk viimane prerequisites sektsioonis antud käsk võiks välja näha hoopis selline: sudo apt-get -y install cuda-11-8 libcudnn8 cuda-12-2. Lisaks on cuDNN link katki.

* requirements_cuml.txt failis on mitmete asjade alla laadimisel vaja teatud dependencyte eriversioone. Panen siia problemaatilised dependencyd, mille lisasin requirements_cuml.txt faili algusesse:
cuda-python==11.8.2
click==8.0
zipp==3.1.0
toolz==0.10.0
cloudpickle==1.5.0

* Ehk requirements_cuml.txt näeb lokaalselt välja hetkel selline:

``` txt
--extra-index-url <https://pypi.nvidia.com/>
cuda-python==11.8.2
click==8.0
zipp==3.1.0
toolz==0.10.0
cloudpickle==1.5.0
cubinlinker-cu11==0.3
ptxcompiler-cu11==0.7.0.post1
cuml-cu11 == 23.4.1
numexpr==2.7.3
numba==0.56.4
importlib-metadata==6.6.0
```

* README-s antud bag ei lähe niisama tööle (<exec_depend>autoware_msgs</exec_depend> tuli lisada suuremasse package.xml faili, seejärel tuli uuesti teha 'rosdep update', 'rosdep rosdep install --from-paths . --ignore-src -r -y' ning 'catkin build')

* carla download link on katki
