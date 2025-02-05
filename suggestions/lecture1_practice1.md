# 1st

## Loeng 1

Autoware_mini arhitektuuri skeemis on näha, et fooritulede tuvastusel kasutatakse kaarti, APIt ja kaamerat. Samas mulle tundub, et kaart tegelikult antakse API-le ette, mitte ei kasutata otseselt fooritulede tuvastamisel. Võib-olla ma eksin, aga lihtsalt selline tähelepanek.

**Q:** Oletan, et fooride API on selline, et linnalt saadakse live andmed fooride olekust. See tundub olevat minu arvates kõige täpsem meetod fooritulede tuvastamiseks. Miks kasutatakse fooritulede jaoks ka kaamerat (ehk siis masinõppel põhinevat pildituvastust, mis võib olla ebatäpne)? Kas see on ajutine lahendus näiteks selleks, kui isejuhtivas autos peaks internet mingil põhjusel kaduma või kui linnal ei ole fooritulede APIt veel pakkuda?

ROS on esmakordsel kasutamisel suhteliselt keeruline teema (vähemalt minu jaoks oli, robootika I ainekursuses kasutasime ainult Pythonit, seetõttu varasem kokkupuude ROSiga puudus). Loengumaterjalides on see minu arvates päris hästi lahti seletatud.

## Praktikum 1

Esmalt võiks courses juhend juhatada MAIN README lehele. See võiks olla nt vahetult enne course schedule pealkirja (hetkel on viited vaid peatükkidele, aga mitte main README-le). Seda teeks sellepast, et main READMEs on olulist infot, mida courses lehel pole -  näiteks on main READMEs kirjas, et githubi-ga võiks luua ühenduse privaatse ja avaliku võtmepaari abil.

Juhendis ei ole kirjas, et rosrun jaoks peab enne olema tehtud ka source devel/setup.bash (mis võiks olla lisatud ka ~/.bashrc nagu autoware_mini repo juhendis on kirjas).

**Q:** Kas ma saan õigesti aru, et ROS parameetrid on globaalsete muutujate hoidmiseks (mida eriti runtime ajal ei muudeta, kuigi set ja delete meetodid on parameetrite jaoks olemas?) ning tavalised publisher noded kindla skoobiga muutujate hoidmiseks. Kui jah, siis võiks see konventsioon ka juhendis olla, siis võib-olla tuleb paremini välja, miks parameetreid üldse vaja on, kui infot saab küsida ka lihtsalt publisher nodede käest. Mul lihtsalt vahepeal tekkis see küsimus, et miks on üldse parameetreid vaja, kui on võimalik luua lihtsalt publisher nodesid.
