extensions [vid]



; Elias A Reyes.




; variable declarations

;-------------------------------------------------------------------

breed [ robots   robot  ]        ;; this are the flocking agents
breed [ payloads payload]        ;; red square to be pushed
patches-own [ is-target is-wall scent payload-scent ]        ;; target , obstacles scent signals for target and payload


; global variables
robots-own
[
  vL                             ;left wheel vel
  vR                             ; right wheel vel
  radius                         ;For collision reporter
  local-density                  ;For MIPS
  linger-ticks                   ;to kill when loitering at target. loitering prevents the payload from reaching the target
]
globals
[
  order                          ;polarization of all robots
  phi-order                      ; polar order of dilute/fast phase
  collisions                     ; counter only
  target-x
  target-y
   max-scent                 ; this is the largest constant that is the max scent i expect
 run-number                  ;this global did not work use "behaviorspace-run-number" only needed when runing experiments
]



;;; ---------------------------------- SETUP -----------------------

to setup

  clear-all                      ; clean up
  create-robots num-robots       ; make N number of robots using slider
  [

    set shape "robot"
    setxy random-xcor random-ycor     ;drop at random patch
    set color green
    set size  2.5
    set heading random 360
    set vL speed                      ;initialize usiing speed slider
    set vR speed
    set linger-ticks 0                ; reset loitering ticks near target

    ;-----
    set radius 0.4                    ;i belive this is not used


  ]

  setup-payload                        ;calling all the methods
  setup-target
  setup-scent
  diffuse-scent
  setup-obstacles
  set max-scent 100                    ; initial strength before diffusion



  reset-ticks


  ;start the recorder
  if vid-recorder = true
  [
  vid:start-recorder
  ]
end



;;;---------------------------------- GO! --------------------------

to go



 ;this was used during one of the experiments.
 ; if ticks >= 700
 ; [ export-plot "Order parameter" (word "order_run" behaviorspace-run-number ".csv")
 ;   stop
 ; ]


  update-local-densities
  if target-reached? [
     export-plot "Order parameter" (word "order_run" behaviorspace-run-number ".csv")

    ;disabled message for behaviour space
    ;user-message  "Target reached"
    stop      ;; halt the simulation
  ]

  ask robots
  [
    ;rt random 360               ; pick a random direction
    turn-vicsek                  ; vicsek rule


    ;----diffy drive ----- basically the robot wheels
    let omega (vR - vL)          ;in radians
    let v     (vL + vR) / 2
    set heading heading + omega

    forward v

    ;-------------------------------------------- when loitering at target too long

    let d-target distancexy target-x target-y

    ifelse d-target < linger-radius [
      ;; close to target  then count up
      set linger-ticks linger-ticks + 1
      if linger-ticks > linger-max [
        die                     ;; remove this robot
      ]
    ] [
      ;; not close anymore reset counter
      set linger-ticks 0
    ]






  ]



 ask patches [
  ifelse is-wall [
    set pcolor orange                                   ;; obstacle
  ][
    ;; not a wall → decide which scent dominates locally
    ifelse scent > payload-scent [
      ;; target scent stronger → red scale
      set pcolor scale-color red   scent          0 max-scent
    ][
      ;; payload scent stronger → green scale
      set pcolor scale-color green payload-scent  0  max-scent
    ]
  ]
]



  ;-------------------------------------------------- paylod step and scent



  ask payloads [


    payload-step   ; ask the payloads to move if pushers nearby

    if target-reached? = true [
    stop
    ]

  ]
  diffuse-scent             ;cant recall why both are here byt only diffuse-payload-scent is necessary for this
  diffuse-payload-scent


  set-current-plot "Order parameter"
  set-current-plot-pen "global-order"
  plot order







   if vid-recorder = true
 [

  vid:record-view


  tick
  if ticks > 1000
  [ vid:save-recording "foo.mp4"
    stop
    vid:reset-recorder

  ]
  ]
tick


end

;;; ---------------------------------- collision reporter -----------------------

to-report collision-with [ current-bot pos-x pos-y]
  ;; true if current robot overlaps another robot after moving some dx dy
  report any? robots with
  [  self != current-bot  and
    distancexy ( [xcor] of current-bot + pos-x)
    ( [ycor] of current-bot + pos-y) < (radius + [radius] of current-bot)

  ]
end

;;; ---------------------------------- local density (mips) -----------------------

to update-local-densities
  ask robots
  [
    ;count your nbrs inside a circle of radii 4
    set local-density count (robots in-radius  4  )
  ]
end


;;; ---------------------------------- Interaction rules (vicsek) -----------------------

to turn-vicsek



  ; Ok this method is a bit complicated but basically the robot computes an initial heading nudge
  ;based on the detection of scent
  ;then vicksek rules are applyed
  ;lastly we check if the next step will make us colide with a box or another bot, then we try to avoid that




  ;scent detection
  steer-to-box ;; only if in the vecinity of box
  let best-neighbour max-one-of neighbors4 [ scent ]
  let scent-turn 0
  if [ scent ] of best-neighbour > [scent] of patch-here
  [
    ;heading from me to that neighbour
    set scent-turn subtract-headings
    (towards best-neighbour) heading

  ]

  ; how much does robot care for target scent

  set heading heading + scent-weight * scent-turn

 ;;;vicsek starts here
  let nbrs robots in-radius interaction-radius

  let vx mean [cos heading] of nbrs
  let vy mean [sin heading] of nbrs
  let desired-theta atan vy vx          ;; desired heading
  set order sqrt ( vx * vx + vy * vy)

  ;; noise -----------------------------------------------------
  let eta (random-float noise) - noise / 2

  ;; wheel speeds for differential drive -----------------------
  let dtheta subtract-headings desired-theta heading
  let delta 0.5 * dtheta                ;; wheel-base = 1


  ;change speed based on density  [MIPS IS CALCULATED IN HERE]---------------------------------------------------------------------
  ifelse local-density >= density-threshold
  [
    ask self [set color red ]
    let dense-speed speed * dense-speed-factor        ;dense speed maintains global speed

    set vL dense-speed - delta * eta
    set vR dense-speed + delta * eta

    ; prediction
    let new-heading heading + (vR - vL)           ;; turning this tick
    let v-step (vL + vR) / 2                      ;; forward distance

    ;; displacement components in NetLogo coordinates
    let d-X v-step * sin new-heading
    let d-Y v-step * cos new-heading
    let next-patch patch (xcor + d-X) (ycor + d-Y)

    if [is-wall] of next-patch [
      ;; very simple reaction: stop for this tick
      set heading heading + 180 + random-float 40
      stop     ;; skip rest of turn-vicsek for this robot
    ]
    if collision-with self d-X d-Y
    [
      ;; simple reaction: stop this tick and count
      set vL vL * 0.5
      set vR vR * 0.5
      set heading new-heading     ; yoooo this actually worked
      set collisions collisions + 1
    ]
  ]

  ;if density is not critical continue normally [NO MIPS]

  [



    set color green
    set vL speed - delta * eta
    set vR speed + delta * eta

    ; prediction
    let new-heading heading + (vR - vL)           ;; turning this tick
    let v-step (vL + vR) / 2                      ;; forward distance

    ;; displacement components in NetLogo coordinates
    let d-X v-step * sin new-heading
    let d-Y v-step * cos new-heading
    ;;;;;;;;;;;;;;;;;;;;;;;;DISABLED COLLISIONS FOR NOW
    let next-patch patch (xcor + d-X) (ycor + d-Y)

    if [is-wall] of next-patch [
      ;; very simple reaction: stop for this tick
      set heading heading + 180 + random-float 40
      stop     ;; skip rest of turn-vicsek for this robot
    ]
    if collision-with self d-X d-Y
    [
      ;;; simple reaction: stop this tick and count
      set vL vL * 0.5
      set vR vR * 0.5
      set heading new-heading     ; yoooo this actually makes them look for places that are not occupied
      set collisions collisions + 1


    ]





  ]


end


;;; ---------------------------------- payload -----------------------


; this block makes n number of payloads
to setup-payload
  let ok? false
  while [ not ok? ] [
    create-payloads 1 [
      setxy random-xcor random-ycor
      set shape "square"
      ifelse distancexy target-x target-y > target-radius + 5 [
        set ok? true
      ]

      [ die ]   ;; too close  retry


    ]
  ]
end

;define the interactions between payload and robots

to payload-step
  let pushers robots in-radius 1
  if not any? pushers [ stop ]

  ;; --- unit push direction from pushers -------------------------------
  let vx mean [sin heading] of pushers
  let vy mean [cos heading] of pushers
  let mag sqrt (vx * vx + vy * vy)
  if mag = 0 [ stop ]
  let ux vx / mag
  let uy vy / mag
  let n  count pushers

  ;; --- intended centre displacement -----------------------------------
  let step      push-gain * n
  let moved?    false
  let maxTry    100
  let tries     0

  while [ (not moved?) and (tries < maxTry) ] [

    ;; ----------  try straight ahead  -------------------------------
    set moved? (attempt-move ux uy step)

    ;; ------- if blocked, try left/right slides -----------------
    if not moved? [
      let left-u  rotate-vector ux uy  90
      let right-u rotate-vector ux uy -90
      set moved? (attempt-move item 0 left-u item 1 left-u step
                or attempt-move item 0 right-u item 1 right-u step)
    ]

    ;; ---------- shrink step and retry -----------------------------
    if not moved? [
      set step step * 0.5
      set tries tries + 1
    ]
  ]
end


;; helper: test if a move by (ux,uy)*step is possible and do it
to-report attempt-move [ dxu dyu step ]
  let new-x xcor + dxu * step
  let new-y ycor + dyu * step

  ;; patches covered by the 1×1 square (centre ±0.5)
  let footprint patches with [
        pxcor >= new-x - 0.5 and pxcor < new-x + 0.5 and
        pycor >= new-y - 0.5 and pycor < new-y + 0.5 ]

  if any? footprint with [ is-wall ] [ report false ]
  ;; free: move, scent, report true
  setxy new-x new-y

  ;; leave strong payload scent on current patch
  ask patch round new-x round new-y [ set payload-scent payload-scent-max ]
  report true
end

;; helper: rotate a 2-D unit vector (ux,uy) by angleDeg
to-report rotate-vector [ux uy angleDeg]
  let a (angleDeg * pi / 180)
  let cosA cos a
  let sinA sin a
  report (list (ux * cosA - uy * sinA) (ux * sinA + uy * cosA))
end


;;; ---------------------------------- target module  -----------------------




;------------------ displays a target


to setup-target
  ask patches [ set is-target false set pcolor black ]

  let margin 5
  let goal one-of patches with [
              pxcor > min-pxcor + margin and
              pxcor < max-pxcor - margin and
              pycor > min-pycor + margin and
              pycor < max-pycor - margin ]

  ask goal [
    set is-target true
    set pcolor pink
  ]

  ;; remember the centre of the goal patch
  set target-x [pxcor] of goal
  set target-y [pycor] of goal
end

;-------------------------> TARGET SCENT

to setup-scent
  ask patches [ set scent 0 ]
end

; evaporation of scent

to diffuse-scent


  ;emission

  ask patches with [ is-target ]
  [
    set scent 100
  ]

  diffuse scent 0.95

  ask patches [set scent scent ]


end

;------------------------------------payload scent ----------------
to diffuse-payload-scent

  ;; ---------- emission: the box writes 100 on its patch -------------
  ask payloads [
    ask patch round xcor round ycor [
      set payload-scent 100
    ]
  ]

  let diffusion-rate 0.95
  let decay-rate 0.95        ;; 0–1  (smaller fades faster)

  diffuse payload-scent diffusion-rate

  ask patches [
    set payload-scent payload-scent * decay-rate
  ]
end



; method to nudge robots towards payload-------------------------------

to steer-to-box
  let best-patch max-one-of neighbors4 [ payload-scent ]
  if [payload-scent] of best-patch > payload-scent [
    let scent-heading towards best-patch
    ;; 0 TO 1 controls how much you trust the scent (1 = alotttt)
    let alpha 0.1
    set heading (1 - alpha) * heading + alpha * scent-heading
  ]
end

;-------------------------> checks if the payload reaches target why is this here?

to-report target-reached?
  ;; distance from payload (centre) to target centre
  report any? payloads with [
          distancexy target-x target-y < target-radius ]
end


; this is for behaviourspace and is a constant that was to be used in machine learning
;sadly time did not allow
to-report fitness
  if target-reached? [
    report (- ticks)          ;; reward faster delivery
  ]
  report -5000               ;; penalty if never reached
end


;;; ---------------------------------- obstacles  -----------------------

to setup-obstacles

    ask patches [
  set is-wall false                ;check if that spot is a wall right biw
  if not is-target [ set pcolor black ]
]

    let n-obstacles obstacles    ; based on input
    repeat n-obstacles [
    let good-spot false

    ;locals
    let w 0
    let h 0
    let cx 0
    let cy 0

    while [ not good-spot ]
    [



    set w 1 + random 3  ;width
    set h 1 + random 3  ;height

    let margin 1  ; same as with the target---> keep off edge
    set cx random ( world-width - 2 * margin ) + min-pxcor + margin
    set cy random ( world-height - 2 * margin ) + min-pycor + margin

    ;; do my coordinates overlap with target?
      if not any? patches with [
        is-target and pxcor >= cx and pxcor < cx + w  and pycor >= cy and pycor < cy + h
      ]
      [ ; no overlap
        set good-spot true
      ]



    ask patches with
      [
      pxcor >= cx and pxcor < cx + w and
      pycor >= cy and pycor < cy + h
      ]
      [
        set is-wall true
        set pcolor orange
      ]
    ]
  ]

end







;;; ---------------------------------- polarization computation -----------------------

to compute-order

  ; here the mean is for all robotS

  let vx mean [cos heading] of robots
  let vy mean [sin heading] of robots
  set order sqrt( (vx * vx) + (vy * vy) )    ;magnitude of velocity vector

  set-current-plot "Order parameter"
  set-current-plot-pen  "polarization"
  plot order
end








@#$#@#$#@
GRAPHICS-WINDOW
0
12
642
655
-1
-1
11.46
1
10
1
1
1
0
1
1
1
-20
20
-20
20
0
0
1
ticks
30.0

SLIDER
696
12
906
45
num-robots
num-robots
10
4000
1134.0
1
1
agents
HORIZONTAL

SLIDER
696
62
906
95
speed
speed
0.1
2
1.159
0.001
1
patches/tick
HORIZONTAL

BUTTON
936
12
1003
45
SETUP
setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
1028
12
1091
45
GO!
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
698
112
910
145
interaction-radius
interaction-radius
0
20
3.976
0.001
1
patches
HORIZONTAL

SLIDER
698
158
870
191
noise
noise
0
10
2.2
0.1
1
NIL
HORIZONTAL

SLIDER
699
205
871
238
push-gain
push-gain
0
1
0.2
0.001
1
NIL
HORIZONTAL

MONITOR
700
259
760
304
NIL
collisions
17
1
11

TEXTBOX
700
340
850
415
motility induced phase separation (MIPS) inputs 
5
15.0
1

SLIDER
696
435
868
468
density-threshold
density-threshold
0
300
51.0
1
1
NIL
HORIZONTAL

SLIDER
696
488
868
521
dense-speed-factor
dense-speed-factor
0
1
0.31
0.01
1
NIL
HORIZONTAL

CHOOSER
785
259
923
304
vid-recorder
vid-recorder
true false
1

PLOT
933
59
1465
477
order parameter
NIL
0
0.0
10.0
0.0
1.0
true
false
"" ""
PENS
"global-order" 1.0 0 -7500403 true "" "plot order"

INPUTBOX
697
545
870
605
obstacles
20.0
1
0
Number

SLIDER
910
522
1082
555
scent-weight
scent-weight
0
1
0.05
0.01
1
NIL
HORIZONTAL

INPUTBOX
915
576
1084
636
target-radius
2.0
1
0
Number

SLIDER
1109
533
1281
566
linger-radius
linger-radius
0
5
2.0
1
1
NIL
HORIZONTAL

SLIDER
1108
588
1284
621
linger-max
linger-max
0
300
300.0
1
1
ticks -> kill robot 
HORIZONTAL

TEXTBOX
1109
479
1318
516
Target patch loitering conditions
4
15.0
1

TEXTBOX
1335
486
1485
510
Payload box scent 
4
15.0
1

SLIDER
1325
534
1497
567
payload-scent-max
payload-scent-max
0
1
1.0
0.01
1
NIL
HORIZONTAL

TEXTBOX
918
489
1106
510
Target patch scent 
12
15.0
1

@#$#@#$#@
## WHAT IS IT?

(a general understanding of what the model is trying to show or explain)

## HOW IT WORKS

(what rules the agents use to create the overall behavior of the model)

## HOW TO USE IT

(how to use the model, including a description of each of the items in the Interface tab)

## THINGS TO NOTICE

(suggested things for the user to notice while running the model)

## THINGS TO TRY

(suggested things for the user to try to do (move sliders, switches, etc.) with the model)

## EXTENDING THE MODEL

(suggested things to add or change in the Code tab to make the model more complicated, detailed, accurate, etc.)

## NETLOGO FEATURES

(interesting or unusual features of NetLogo that the model uses, particularly in the Code tab; or where workarounds were needed for missing features)

## RELATED MODELS

(models in the NetLogo Models Library and elsewhere which are of related interest)

## CREDITS AND REFERENCES

(a reference to the model's URL on the web if it has one, as well as any other necessary credits, citations, and links)
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

robot
true
0
Rectangle -7500403 false true 120 120 180 210
Rectangle -7500403 false true 105 150 120 195
Rectangle -7500403 false true 180 150 195 195
Rectangle -7500403 false true 120 165 180 195
Rectangle -7500403 false true 135 105 165 120
Rectangle -13345367 true false 120 120 180 210
Rectangle -13840069 true false 120 165 180 195

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270
@#$#@#$#@
NetLogo 6.4.0
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
<experiments>
  <experiment name="experiment" repetitions="100" runMetricsEveryStep="true">
    <setup>setup</setup>
    <go>go</go>
    <timeLimit steps="1500"/>
    <exitCondition>target-reached? or ticks &gt;= 3000</exitCondition>
    <metric>ticks</metric>
    <metric>target-reached?</metric>
    <metric>collisions</metric>
    <metric>fitness</metric>
    <enumeratedValueSet variable="density-threshold">
      <value value="20"/>
      <value value="30"/>
      <value value="40"/>
      <value value="50"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="num-robots">
      <value value="800"/>
      <value value="1000"/>
      <value value="1200"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="push-gain">
      <value value="0.3"/>
      <value value="0.5"/>
      <value value="0.7"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="dense-speed-factor">
      <value value="0.5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="scent-weight">
      <value value="0.01"/>
      <value value="0.3"/>
      <value value="0.5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="payload-scent-max">
      <value value="0.99"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="speed">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="interaction-radius">
      <value value="1"/>
      <value value="1.5"/>
      <value value="2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="noise">
      <value value="1"/>
      <value value="2"/>
      <value value="3"/>
      <value value="4"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="linger-radius">
      <value value="2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="linger-max">
      <value value="300"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="target-radius">
      <value value="2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="obstacles">
      <value value="10"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="vid-recorder">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="plots" repetitions="100" runMetricsEveryStep="true">
    <setup>setup</setup>
    <go>go</go>
    <timeLimit steps="1500"/>
    <metric>ticks</metric>
    <enumeratedValueSet variable="density-threshold">
      <value value="79"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="num-robots">
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="linger-radius">
      <value value="2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="push-gain">
      <value value="0.2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="linger-max">
      <value value="300"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="dense-speed-factor">
      <value value="0.91"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="vid-recorder">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="scent-weight">
      <value value="0.07"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="payload-scent-max">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="speed">
      <value value="0.788"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="target-radius">
      <value value="2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="interaction-radius">
      <value value="2.159"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="noise">
      <value value="2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="obstacles">
      <value value="20"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="plots 2" repetitions="100" runMetricsEveryStep="true">
    <setup>setup</setup>
    <go>go</go>
    <timeLimit steps="1500"/>
    <metric>ticks</metric>
    <enumeratedValueSet variable="density-threshold">
      <value value="69"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="num-robots">
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="linger-radius">
      <value value="2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="push-gain">
      <value value="0.2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="linger-max">
      <value value="300"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="dense-speed-factor">
      <value value="0.68"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="vid-recorder">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="scent-weight">
      <value value="0.05"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="payload-scent-max">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="speed">
      <value value="1.159"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="target-radius">
      <value value="2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="interaction-radius">
      <value value="3.976"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="noise">
      <value value="2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="obstacles">
      <value value="20"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="plots no mips" repetitions="100" runMetricsEveryStep="true">
    <setup>setup</setup>
    <go>go</go>
    <timeLimit steps="700"/>
    <metric>ticks</metric>
    <enumeratedValueSet variable="density-threshold">
      <value value="300"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="num-robots">
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="linger-radius">
      <value value="2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="push-gain">
      <value value="0.2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="linger-max">
      <value value="300"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="dense-speed-factor">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="vid-recorder">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="scent-weight">
      <value value="0.05"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="payload-scent-max">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="speed">
      <value value="1.159"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="target-radius">
      <value value="2"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="interaction-radius">
      <value value="3.976"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="noise">
      <value value="4.8"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="obstacles">
      <value value="20"/>
    </enumeratedValueSet>
  </experiment>
</experiments>
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180
@#$#@#$#@
0
@#$#@#$#@
