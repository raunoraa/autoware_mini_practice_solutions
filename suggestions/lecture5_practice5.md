# 5th

## Loeng 5

**Viga slaididel:** Kuna nüüd kasutatakse `autoware_mini/msg`, siis peaks 3., 8. ja 9. slaidi joonisel olema hoopis `autoware_mini/msg/DetectedObjectArray`.

**Viga slaididel:** Rosis kasutatakse `final_objects` asemel `tracked_objects`. See mõjutab slaide 19, 26 ja 38.

**Viga slaidil:** Rvizis on center ja front `points_raw` tõstetud vastavalt kohtadesse `Sensing/Points raw center` ja `Sensing/Points raw front`. See mõjutab slaidi 38.

**Q:** Mida tähendab 39. slaidil lidar_sfa? Kas see tähendab seda, et lidari puhul kasutatakse objektide tuvastamisel närvivõrku klasterdamise asemel?

**Q:** Kas `autoware_mini` puhul kasutatakse kiiruse ennustamise puhul pigem närvivõrku või naiivset meetodit?

**Q:** Kui suurel määral võetakse objektide tuvastamisel arvesse kolme sensori tulemusi? Saan nii aru, et kolmemõõtmelist tee hetkeseisu saab näha a)Lidari abil ja  b)kaamerapildi ja radari kombineerimisel. Ehk kui palju võetakse objektide tuvastamisel arvesse meetodit a) ning kui palju meetodit b)? Oletan, et b) on natuke ilmastikukindlam (radari puhul ei ole vahet, kas on udu või sajab vihma või sajab lund, kuid kaamerapilti mõjutab see ikkagi oluliselt, kuid ehk natuke vähem kui lidarit?).

## Praktikum 5

**Viga juhendis:** Samad vead, mis slaididel seoses `autoware_mini/msg` kasutamisega.
