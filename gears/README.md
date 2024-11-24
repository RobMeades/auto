These files comprise the gears for `auto`.

The design originates in ArtSoft's [Gearotic](https://www.gearotic.com/), which allows creation and visualisation of gear-trains that can be exported to `.stl` files.  The `.stl` files were then loaded into Blender where the pinion gears were melded with the wheel gears, a flat was added on the inner of the pinion that mates into the motor (a 3&nbsp;V 6&nbsp;mm diameter motor from [eBay](https://www.ebay.co.uk/itm/253771137237) with built-in planetary gear box providing 46&nbsp;RPM operation), the teeth of the rack were melded into a larger sliding support structure and then the lot is put together in a frame that fits inside `auto`.

All of this is then output again to a `.stl` file for printing at 0.05&nbsp;mm resolution on a resin printer (in my case a Prusa SL1).  It is important to get a nice hard resin for this, for the longevity of the gears: I used [Siraya Tech Build](https://siraya.tech/pages/build-user-guide) resin.  No supports are required: just place all of the parts on their flattest face.

- `gearotic_steer.gth`: the [Gearotic](https://www.gearotic.com/) file containing the steering design; specifically, all gears default settings, circular gears modulus&nbsp;0.4, height stub&nbsp;1.5 and:
  - motor pinion: 10&nbsp;teeth, 1.5&nbsp;mm shaft, 4&nbsp;mm thick,
  - shaft&nbsp;1 wheel: 20&nbsp;teeth, 4&nbsp;mm shaft, 2&nbsp;mm thick,
  - shaft&nbsp;2 wheel: 30&nbsp;teeth, 4&nbsp;mm shaft, 2&nbsp;mm thick,
  - shaft&nbsp;3 wheel: 50&nbsp;teeth, 4&nbsp;mm shaft, 2&nbsp;mm thick,
  - shaft 1, 2 and 3&nbsp;pinion: 10&nbsp;teeth, 1.6&nbsp;mm shaft, 4&nbsp;mm thick,
- the non-circular rack gear was peculiar: the height stub setting in [Gearotic](https://www.gearotic.com/) did nothing, `ArtF` created a [special version of gearotic](https://gearotic.com/GearHeads/viewtopic.php?p=16683#p16683) that included an extra "shift" parameter which adjusted the tooth height but then I realised that the rack still wasn't meshing with the other gears properly when I ran it in simulation, it only did that if I set modulus to 0.5 instead of 0.4, which I think was the problem all along, the "shift" parameter could be left at 0.  So:
  - rack: modulus&nbsp;0.5, 20&nbsp;teeth, 3&nbsp;mm thick.
- `gearotic_steer.mp4`: a video capture of the steering gear train running in Gearotic.
- `gearotic_steer_*.stl`: the STL output files from `gear_steer.gth`.
- `gear_steer.blend`: the `gearotic_steer_*.stl` files imported into Blender plus a base which forms the chassis of `auto` in which the gears can be mounted with the aid of some 1.5&nbsp;mm inner diameter, 4&nbsp;mm outer diameter, 1.2&nbsp;mm high bearings (from [Bearing Boys](https://www.bearingboys.co.uk/Miniature-Bearings/W681X-Budget-Open-Stainless-Steel-Miniature-Ball-Bearing-15mm-x-4mm-x-12mm-62859-p)), 1.5&nbsp;mm diameter brass rod (from [eBay](https://www.ebay.co.uk/itm/134173633161)) cut into 3 x 14&nbsp;mm and 1 x 6&nbsp;mm lengths, an M2 tap and four M2 bolts.  Specific mods to the imported gears:
  - motor pinion and shaft&nbsp;2 pinion: add shoulder to lean against bearing,
  - motor pinion: widen upper half of shaft to 2&nbsp;mm radius and add flat (0.35&nbsp;mm in),
  - shaft x wheel/pinion: join them,
  - rack: merge the teeth of the rack into the rest of the rack structure.
- `gear_steer.stl`: an STL export of `gear_steer.blend` (exported at a scale of 1; these have all been created actual model size in Blender).