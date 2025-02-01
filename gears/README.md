These files comprise the gears for `auto`.

The design originates in ArtSoft's [Gearotic](https://www.gearotic.com/), which allows creation and visualisation of gear-trains that can be exported to `.stl` files.
- `gearotic_steer.gth`: the [Gearotic](https://www.gearotic.com/) file containing the steering design; specifically, all gears default settings, circular gears modulus&nbsp;0.4, height stub&nbsp;1.5 and:
  - motor pinion: 10&nbsp;teeth, 1.5&nbsp;mm shaft, 4&nbsp;mm thick,
  - shaft&nbsp;1 wheel: 20&nbsp;teeth, 4&nbsp;mm shaft, 2&nbsp;mm thick,
  - shaft&nbsp;2 wheel: 30&nbsp;teeth, 4&nbsp;mm shaft, 2&nbsp;mm thick,
  - shaft&nbsp;3 wheel: 50&nbsp;teeth, 4&nbsp;mm shaft, 2&nbsp;mm thick,
  - shaft 1, 2 and 3&nbsp;pinion: 10&nbsp;teeth, 1.6&nbsp;mm shaft, 4&nbsp;mm thick,
- the non-circular rack gear was peculiar: the height stub setting in [Gearotic](https://www.gearotic.com/) did nothing, `ArtF` created a [special version of gearotic](https://gearotic.com/GearHeads/viewtopic.php?p=16683#p16683) that included an extra "shift" parameter which adjusted the tooth height but then I realised that the rack still wasn't meshing with the other gears properly when I ran it in simulation, it only did that if I set modulus to 0.5 instead of 0.4, which I think was the problem all along, the "shift" parameter could be left at 0.  So:
  - rack: modulus&nbsp;0.5, 14&nbsp;teeth, 2.8&nbsp;mm thick.
- `gearotic_steer.mp4`: a video capture of the steering gear train running in Gearotic.
- `gearotic_steer_*.stl`: the STL output files from `gear_steer.gth`.

The `.stl` files were then loaded into Blender where the following modifications were made:
  - shaft x wheel/pinion gears: join them,
  - motor pinion and shaft&nbsp;2 pinion: add shoulder to lean against bearing,
  - motor pinion: widen upper half of shaft to 2&nbsp;mm radius and add flat (0.35&nbsp;mm in),
  - rack: merge the teeth of the rack into the rest of the rack structure.

All of this can be found in the `chassis.blend` file over in the [blender](../blender) directory, along with the frame they run inside which forms the chassis of `auto`.