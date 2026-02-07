# Nema-17/23-planetary-gear

Parametric planetary gearbox, you can change gear ratio, number of planets, backlash, module, etc...

![](images/fully%20assembled.png)

### Software Requirements 
[Fusion 360 (free to download)](https://www.autodesk.com/campaigns/fusion-360/download)

[Fusion 360 Helical Gear Plus plugin](https://apps.autodesk.com/FUSION/en/Detail/Index?id=1259509007239787473&appLang=en&os=Mac)

### Parts List
* M3 heat inserts 
    * (L5 $\times$ 4.2mm OD) $\times$ 18
* M3 hex bolts
    * 91290A120 (16mm)  $\times$ 3
    * 91290A115 (10mm headless)  $\times$ 6
    * 91290A111 (6mm)  $\times$ 4
* Bearings
    * 4668K269 (35mm $\times$ 47mm $\times$ 7mm) $\times$ 1
    * 2349K726 (12mm $\times$ 21mm $\times$ 5mm) $\times$ 2
    * 4668K225 (5mm $\times$ 11mm $\times$ 5mm) $\times$ 6

### Caveats
If you change a param in one generator, don't forget to change it in all others.

This violates DRY principle, instead, global params should be used.

[Global Parameters in Fusion 360 | Explained in 5 minutes](https://www.youtube.com/watch?v=VsqRV7JvBKc)


### Generate gears

Create a new project and import the .f3d files
![](images/uploaded.png)

Open `GearGenerator`

Have a look at the params
![](images/gear%20generator%20params.png)

Start creating the sun gear, open `Helical Gear+`
![](images/plugin.png)

Fill the fields with the sun gear params

![](images/sun.png)

(_Advanced config will be the same for planet and ring_)
![](images/all%20gears%20.png)

In the same manner create the planet gear
![](images/planet.png)

And the ring gear, don't forget to change Type to `Internal Gear`
![](images/ring.png)

You should end up with this
![](images/generated%20gears.png)

Export ring gear as `RingTemp` to your project
![](images/export%20ring.png)

In the same manner, export planet gear as `PlanetTemp` and sun gear as `SunTemp`

Close current design without saving

### Modify generated gears

Open `SunModifier`
![](images/open%20Sun.png)

Check its params as well
![](images/sun%20params.png)

Insert the `SunTemp` component
![](images/insert%20z_sun%20into%20current%20design.png)

Break the link
![](images/break%20link.png)

Extrude and fillet the bearing mount, make sure operation for extrude is `Join`
![](images/extrude.png)
![](images/fillet.png)

Export component as `Sun` to your project

Cut and fillet motor shaft
![](images/cut%20shaft.png)
![](images/fillet%20shaft.png)

Chamfer from the inside
![](images/shaft%20chamfer.png)

Should end up with this
![](images/motor%20shaft%20analysis.png)

Export component to project as `MotorInput`

Close current design without saving

_If you create another gearbox with a different gear ratio, `Sun` and `MotorInput` will stay the same, no need to regnerate them_

In the same manner, open `PlanetModifier` and insert `PlanetTemp`
![](images/open%20Planet%20sktech%20and%20insert%20planet.png)

Break link

Cut hole for bearing
![](images/cut%20bearing.png)

Fillet
![](images/fillet%202.png)

Cut hole for bearing lip
![](images/cut%20for%20bearing%20lip%20hole.png)

Should end up with this
![](images/Planet%20analysis.png)

Export to project as `Planet`, close current design without saving, move `PlanetTemp` to trash

We now have all the components, time to combine them

### Combine generated gears

In a new Hybrid Design, import `RingBottom`, `RingTemp` and `RingTop`
![](images/new%20hybrid%20design,%20insert%20ring,ring%20top%20and%20ring%20bottom.png)

Break link any of them

Join `RingBottom` and `RingTemp` at their bottoms
![](images/join%20ring%20bottom%20and%20ring%20at%20the%20bottom.png)

Join `RingTemp`'s top and `RingTop`'s bottom

Should end up with this
![](images/ring_joined.png)

Combine all of them as new component, export it to project as `Ring`, close current design without saving, move `RingTemp` to trash

Open `CarrierGenerator` and check its params, 
`n_planets` is the current planets' number

![](images/examine%20Carrier%20params.png)
---
❗ Note that the max number of planets the gear can have (_without collision_), follows this equation:

$n_{max} = floor(\frac{\pi}{arcsin(r/R)}$)

Where r is `planet_radius` and R is `planets_distance_from_sun`

But fusion cannot have trigonometric functions in params, so do this calculation externally before updating `n_planets`, make sure you compute $arcsign$ in _radians_

You'll also need as much 4668K225 bearings as planets' count, as well as 91390A100 bolts

---

In a new Hybrid Design, insert `CarrierGenerator` and `Sun`
![](images/new%20hybrid%20design,%20insert%20carrier%20and%20sun.png)

Break link any of them

Join `Sun`'s bottom with `Carrier`'s top at their centers
![](images/join%20sun%20bottom%20at%20carrier%20top%20(centers).png)

Should end up with this
![](images/first%20stage%20joined.png)

Combine all of the as new component, export it to project as `FirstStage`

Close current design without saving

Move `Sun` to trash

In a new Hybrid Design, insert `CarrierGenerator` and `CarrierOutput`
![](images/in%20same%20manner,%20create%20second%20stage%20using%20output%20component.png)

Break link any of them, join `CarrierOutput` bottom at `CarrierGenerator`'s top at their centers

Should end up with this
![](images/second%20stage%20joined.png)

Combine all as new component, export it to project as `SecondStage`

You can now export `Cover`, `FirstStage`, `MotorInput`, `Planet`, `Ring`, `SecondStage` and `SunTop` as .step files to print

❗ Make sure you print the stages at 45&deg; and 100% infill, especially the output stage, as it handles the most torque
![](images/print%20stages%20at%2045%20angle%20with%20100%20infill.png)


[The Correct Orientation to Print Boxes](https://www.youtube.com/watch?v=8NKVNwVaZU0)

[Autodesk Fusion: Make supports like Slant 3D](https://www.youtube.com/watch?v=sn2u949g7dM)

Print 6 `Planet`s ($n_{stages} \times n_{planetsPerStage}$)

Print 2 `SunTop`s (_1 per stage_)

![](images/fullprintplate.png)

Print takes ~5 hours and ~100 gm of filament