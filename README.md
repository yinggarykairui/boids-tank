# boids-tank

A tank of flocking darts you can tune while they fly — three sliders for the three rules that make a flock, and a button that blows it apart.

![The tank a quarter-second after scatter: 140 dark darts flying outward from
a single point a little above and left of the field's centre, each dragging a
long ink trail behind it, so the burst reads as a starburst of lines. A
handful more rise through the bottom edge, having wrapped round from the top.
Below the tank, three labelled sliders — cohesion 50%, alignment 62%,
separation 55% — and a scatter button.](screenshot.png)

**[Live demo](https://yinggarykairui.github.io/boids-tank/)**

## What it does

Up to 140 boids cross a pale field, each dragging a short ink trail. Three
rules steer them: cohesion toward the flock's centre, alignment with its
heading, separation from close neighbours. Every rule has a slider, and moving
one changes the flock in flight — at separation 0 it packs into one dark knot
about a dozen boids wide. The **scatter** button throws every boid outward
from the centre in a starburst of trails, and the flock finds itself again a
few seconds later.

Everything is keyboard-reachable, and it works at phone width.

## How to run

```
git clone https://github.com/yinggarykairui/boids-tank.git
cd boids-tank
```

Then open `index.html` in any browser. There is no build step and no dependency.

Or serve the folder if you prefer:

```
python3 -m http.server 8000
```

then open <http://localhost:8000/>.

`tests.html` runs the simulation's unit tests in the browser — open it and read
the counts.

## Why it exists

Seeded idea from the factory's warm-start pack ([hub issue #8](https://github.com/yinggarykairui/factory-hub/issues/8)):
"a flocking simulation tank with three sliders — cohesion, alignment,
separation — and a 'scatter' button." Boids are the canonical three-rule
emergent system, and the point of putting the rules on sliders is that you can
feel which rule is doing what.

---

*Day 016 of an autonomous build factory — [factory-hub](https://github.com/yinggarykairui/factory-hub)*
