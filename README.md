# Nema-17-planetary-gear

Parametric plantary gear box, you can change gear ratio, number of planet carriers, backlash, module etc...

### Requirements 

Helical Gear Plus plugin https://apps.autodesk.com/FUSION/en/Detail/Index?id=1259509007239787473&appLang=en&os=Mac


### Generate gears

Open GearGenerator
![](images/gear%20generator%20project.png)



Open the params, examine the gear ratio and backlash
![](images/gear%20generator%20params.png)

Start creating the sun gears, open Helical Gear+
![](images/plugin.png)

Fill the fields with the sun params
![](images/sun.png)

(Advanced config will be the same for all gears)
![](images/all%20gears%20.png)

In the same manner create the planet gear
![](images/planet.png)

And the ring gear, don't forget to change Type to "Internal Gear"
![](images/ring.png)

You should end up with this
![](images/generated%20gears.png)

Export ring gear as "Ring_temp" to your project!
![](images/export%20ring.png)

In the same manner, export planet gear as Planet_temp and sun gear as Sun_temp

### Modify generated gears

Open Sun modifier
![](images/open%20Sun.png)

Check its params as well
![](images/sun%20params.png)

Insert the Sun_temp component
![](images/insert%20z_sun%20into%20current%20design.png)

Break the link
![](images/break%20link.png)

Extrude and fillet the bearing mount
![](images/extrude.png)
![](images/fillet.png)

Export component as "Sun"

Cut, fillet motor shaft
![](images/cut%20shaft.png)
![](images/fillet%20shaft.png)

Chamfer from the inside
![](images/shaft%20chamfer.png)

Should end up with this
![](images/motor%20shaft%20analysis.png)

Export component to project as "Motor input"

Move Sun_temp to trash

In the same way, open Planet modifier (it has params well) and insert Planet_temp
![](images/open%20Planet%20sktech%20and%20insert%20planet.png)

Cut hole for bearing
![](images/cut%20bearing.png)

Fillet
![](images/fillet%202.png)

Cut hole for bearing lip
![](images/cut%20for%20bearing%20lip%20hole.png)

Should end up with this
![](images/Planet%20analysis.png)

Export to project as Planet, move Planet_temp to trash

We have now have all the components, time to combine them

### Combine generated gears

In a new Hybrid Design, import Ring_temp, Ring top and Ring bottom
![](images/new%20hybrid%20design,%20insert%20ring,ring%20top%20and%20ring%20bottom.png)

Break link any of them

Join Ring bottom and Ring at their bottoms
![](images/join%20ring%20bottom%20and%20ring%20at%20the%20bottom.png)

Join Ring top's bottom and Ring_temp's top

Should end up with this
![](images/ring_joined.png)

Combine all of the as new component, export it to project as Ring, move Ring_temp to trash

Before moving on to the Carrier, open Carrier generator and check its params, expecially "n_planets"

![](images/examine%20Carrier%20params.png)

In a new Hybrid Design, insert Carrier generator and Sun
![](images/new%20hybrid%20design,%20insert%20carrier%20and%20sun.png)

Break link any of them

Join sun bottom at carrier top at their centers
![](images/join%20sun%20bottom%20at%20carrier%20top%20(centers).png)

Should end up with this
![](images/first%20stage%20joined.png)

Combine all of the as new component, export it to project as First stage

In a new Hybrid Design, insert Carrier generator and Carrier output generator
![](images/in%20same%20manner,%20create%20second%20stage%20using%20output%20component.png)

Break link any of them, join Output bottom at carrier top at their centers

Should end up with this
![](images/second%20stage%20joined.png)

Combine all as new component, export it to project as Second stage