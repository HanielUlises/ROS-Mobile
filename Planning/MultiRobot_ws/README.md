# Collaborative 2D SLAM for a Ground Fleet under Intermittent Connectivity

**TT 2026-B193 — *Planificación Epistémica y SLAM en Entornos Desconocidos para Sistemas Multiagente*
Escuela Superior de Cómputo, Instituto Politécnico Nacional**

First implementation iteration. This document specifies the simulated multi-robot
platform, the map fusion operator and the connectivity model that together form
the perceptual substrate on which the epistemic planning layer of the project is
to be built.

---

## Abstract

A fleet of $n$ unmanned ground vehicles explores an unknown planar environment,
each agent running an independent pose-graph SLAM front end over its own laser
observations. Because inter-agent connectivity is intermittent, the fleet-level
map is not the instantaneous union of the individual maps: it is the union of
what has actually been delivered. This iteration implements that distinction
explicitly. Individual occupancy estimates are fused by an idempotent,
order-independent operator over a bounded join-semilattice, and delivery is gated
by a per-agent link process, so that a lost link freezes an agent's contribution
without erasing it. The resulting divergence between what an agent knows and what
the fleet knows is measured directly, and is precisely the asymmetry that the
Dynamic Epistemic Logic layer of this work is intended to reason about.

---

## 1. Problem statement

Let $\mathcal{A} = \{1, \dots, n\}$ index the agents and let $\mathcal{W} \subset
\mathbb{R}^2$ be a bounded, static, initially unknown workspace. Each agent
$i \in \mathcal{A}$ maintains a discrete occupancy estimate

$$m_i : \mathcal{G}_i \to \mathcal{V}, \qquad
\mathcal{V} = \{\,\mathsf{u}\,\} \cup \{0, \dots, 100\},$$

over a regular lattice $\mathcal{G}_i \subset \mathbb{Z}^2$ of resolution
$\rho$, where $\mathsf{u}$ denotes an unobserved cell and the numeric values are
occupancy odds expressed as percentages. Classical SLAM asks each agent to
estimate $m_i$ jointly with its trajectory from its own noisy observations
[[1](#ref1)].

The multi-agent problem adds a second question that the single-agent formulation
cannot express: *what does the fleet know?* Writing $\mathcal{C}(t) \subseteq
\mathcal{A}$ for the set of agents reachable at time $t$, the naive answer

$$M(t) \;=\; \bigoplus_{i \in \mathcal{C}(t)} m_i(t)$$

is wrong, because it makes fleet knowledge non-monotonic: an agent leaving
$\mathcal{C}(t)$ would delete observations the fleet had already received. A
radio link that drops interrupts the arrival of *new* information; it does not
retract information already delivered. Section [3](#3-map-fusion) formalises the
corrected semantics.

This distinction is the concrete, geometric instance of the gap the project
addresses. Collaborative SLAM systems degrade under intermittent connectivity
[[2](#ref2)] precisely because they lack a formalism for representing which
portions of the map are known to which agents and when synchronisation is
warranted.

---

## 2. Platform

### 2.1 Vehicle model

Each agent is a differential-drive UGV whose configuration
$q = (x, y, \theta)^\top \in SE(2)$ evolves under the unicycle constraint

$$\dot{x} = v\cos\theta, \qquad \dot{y} = v\sin\theta, \qquad \dot{\theta} = \omega,$$

with body velocities recovered from wheel rates $(\omega_L, \omega_R)$ as
$v = \tfrac{r}{2}(\omega_R + \omega_L)$ and
$\omega = \tfrac{r}{b}(\omega_R - \omega_L)$.

| Parameter | Symbol | Value |
|---|---|---|
| Wheel radius | $r$ | $0.070$ m |
| Wheel separation | $b$ | $0.300$ m |
| Chassis footprint | — | $0.35 \times 0.28$ m |
| Mass | — | $7.1$ kg |
| Commanded cruise speed | $v$ | $0.25$ m s$^{-1}$ |
| Commanded turn rate | $\omega$ | $0.90$ rad s$^{-1}$ |

Odometry is integrated from the simulated wheel encoders and published as the
transform $\mathsf{odom}_i \to \mathsf{base}_i$; it is subject to the usual
unbounded drift, which the SLAM front end corrects through the
$\mathsf{map}_i \to \mathsf{odom}_i$ transform.

### 2.2 Exteroceptive sensing

A planar lidar rigidly mounted on the chassis returns $360$ beams over a full
$2\pi$ field of view at $10$ Hz, with range $[0.15,\, 8.0]$ m and additive
Gaussian range noise of standard deviation $\sigma = 0.01$ m. Beams are cast on
the CPU rather than the GPU so that the platform behaves identically in headless
batch runs and interactive sessions.

### 2.3 Per-agent estimation

Every agent runs an independent asynchronous pose-graph SLAM instance
(`slam_toolbox`, a Karto-derived correlative scan matcher with a sparse
pose-graph back end solved by Ceres). Nodes are inserted on a travelled distance
of $0.20$ m or a heading change of $0.20$ rad; loop closures are searched within
a $3.0$ m radius over chains of at least $10$ nodes. The published grid
resolution is $\rho = 0.05$ m.

Crucially, no agent consumes another agent's scans, poses or pose graph. Coupling
occurs exclusively at the level of published occupancy grids, through the
operator of Section [3](#3-map-fusion). This keeps the estimation layer a
strict per-agent concern and confines all inter-agent semantics to a single,
analysable component.

### 2.4 Frame convention

All agents share one transform tree; disambiguation is by frame prefix rather
than by namespaced trees, so a single global view is well defined.

| Frame | Parent | Source |
|---|---|---|
| $\mathsf{map}$ | — | global reference |
| $\mathsf{map}_i$ | $\mathsf{map}$ | static, from $T_i$ (§3.1) |
| $\mathsf{odom}_i$ | $\mathsf{map}_i$ | SLAM correction |
| $\mathsf{base}_i$ | $\mathsf{odom}_i$ | wheel odometry |
| $\mathsf{laser}_i$ | $\mathsf{base}_i$ | fixed extrinsics |

---

## 3. Map fusion

### 3.1 The fusion operator

Let $T_i \in SE(2)$ be the rigid transform carrying agent $i$'s map frame into
the global frame, and let $\pi_i : \mathcal{G} \to \mathcal{G}_i$ be the induced
lattice correspondence. Define the binary operator $\oplus$ on $\mathcal{V}$ by

$$
\mathsf{u} \oplus x = x, \qquad
x \oplus \mathsf{u} = x, \qquad
x \oplus y = \max(x, y) \quad \text{for } x, y \neq \mathsf{u}.
$$

The fused map is then the cell-wise reduction

$$M(c) \;=\; \bigoplus_{i \in \mathcal{D}(t)} m_i\!\left(\pi_i^{-1}(c)\right),
\qquad c \in \mathcal{G},$$

over the set $\mathcal{D}(t)$ of agents whose maps have been delivered
(Section [3.2](#32-retention-under-link-loss)).

Two properties of $\oplus$ carry the engineering weight.

**Absorption of ignorance.** $\mathsf{u}$ is a two-sided identity, so an agent
that has not observed a cell contributes nothing there. Ignorance never
overwrites knowledge, and unobserved regions of the workspace remain marked
unobserved in the fused map rather than being silently filled.

**Order independence.** $\oplus$ is commutative, associative and idempotent, and
admits the identity $\mathsf{u}$; hence $(\mathcal{V}, \oplus, \mathsf{u})$ is a
bounded join-semilattice. The fused map therefore depends only on the *set* of
delivered grids, not on their arrival order, nor on how many times any grid is
redelivered. Under asynchronous, lossy, duplicating delivery — exactly the
regime this work targets — the fleet map is well defined without any distributed
agreement protocol. This is the geometric counterpart of *distributed knowledge*;
it is worth stating plainly that **common knowledge is not obtainable this way**,
and recovering it is one of the things the epistemic layer must supply.

Where two agents disagree on an observed cell, $\max$ resolves in favour of
occupancy. The choice is deliberately conservative for navigation, and it keeps
disagreement visible as an obstacle rather than averaging it away into free
space.

### 3.2 Retention under link loss

Let $\ell_i(t) \in \{0,1\}$ be the link indicator of agent $i$. The grid that
agent $i$ contributes to the fusion at time $t$ is the most recent one delivered
while its link was up:

$$\tilde{m}_i(t) \;=\; m_i(\tau_i(t)), \qquad
\tau_i(t) \;=\; \sup\{\, s \le t \;:\; \ell_i(s) = 1 \,\},$$

and $\mathcal{D}(t) = \{\, i : \tau_i(t) \text{ exists} \,\}$. An agent that
loses its link therefore *freezes* rather than disappears: its contribution goes
stale and stops tracking its own exploration, but nothing already shared is
retracted. Consequently

$$\{\, c : M_t(c) \neq \mathsf{u} \,\} \;\subseteq\; \{\, c : M_{t'}(c) \neq \mathsf{u} \,\}
\quad \text{for } t \le t',$$

so fleet knowledge is monotone non-decreasing in time even though connectivity
is not. The observable consequence — a plateau in the fused map while an agent
keeps exploring alone — is visible in Figure [3](#figure-3).

---

## 4. Connectivity model

Connectivity is generated by a per-agent duty cycle

$$\ell_i(t) = \mathbb{1}\!\left[\,(t + i\,\delta) \bmod (T_{\mathrm{up}} + T_{\mathrm{down}}) < T_{\mathrm{up}}\,\right],$$

with $T_{\mathrm{up}} = 45$ s, $T_{\mathrm{down}} = 25$ s and phase offset
$\delta = 12$ s. The phase offset is not incidental. A single global on/off
switch would keep every agent in the same informational state at all times,
which is exactly the degenerate case in which epistemic reasoning is unnecessary.
Offsetting the cycles forces the fleet through *partial* connectivity states, in
which agents hold genuinely different information about the same environment —
the regime in which the asymmetries of interest arise.

---

## 5. Experimental evaluation

### 5.1 Scenario

A $12 \times 8$ m warehouse bounded by walls, containing three shelf aisles
arranged as six $0.6 \times 3.0$ m blocks, with the eastern corridor obstructed
by a removable barrier. The obstruction reproduces the motivating scenario of the
proposal: one agent may observe the blockage while another, disconnected at that
moment, does not.

Two agents are deployed from opposite ends of the western aisle, at
$(-5, +3)$ and $(-5, -3)$ metres. Exploration is driven by a reactive
laser-based controller with a wall-following bias, deliberately *not* a
deliberative planner: this iteration is concerned with the perceptual and
communication substrate, and a fixed reactive policy keeps the exploration
process constant while the fusion and connectivity semantics are evaluated.
Replacing it is the subject of the planning objective (OE1).

### 5.2 Map construction

<a name="figure-1"></a>

![Merged occupancy grid evolving over the run](docs/figures/map_growth.gif)

**Figure 1.** Evolution of the fused occupancy grid over the run. Black denotes
occupied cells, white free cells, grey unobserved cells. The canvas is fixed to
the final extent, so growth of the mapped region is directly visible. Note that
the grid never loses observed cells, in accordance with the monotonicity of
Section [3.2](#32-retention-under-link-loss).

<a name="figure-2"></a>

![Final merged occupancy grid](docs/figures/final_map.png)

**Figure 2.** Fused occupancy grid at the end of the run. The outer walls, the
six shelf blocks and the eastern obstruction are all recovered. The two agents'
grids are expressed in distinct frames and composited through $T_i$; the absence
of doubled walls at the seam between their explored regions indicates that the
per-agent estimates remain mutually consistent over the run.

### 5.3 Individual versus fleet knowledge

<a name="figure-3"></a>

![Explored area against time](docs/figures/coverage.png)

**Figure 3.** Explored area against simulation time. Thin coloured traces give
each agent's own map; the heavy black trace gives the fused fleet map. Shaded
bands mark intervals in which an agent's link is down. Area is reported in
square metres rather than as a fraction of the canvas, since the canvas itself
grows with the map and fractions would not be comparable across time.

The quantity of interest is the vertical gap between the fused trace and each
individual trace. It is the geometric measure of what an agent does *not* know
but the fleet does — the epistemic asymmetry that motivates the modal treatment
of the following iterations. During an outage the fused trace flattens with
respect to the disconnected agent's contribution while that agent's own trace
continues to climb, and the accumulated difference is recovered in a step at
reconnection.

<a name="table-1"></a>

**Table 1.** Explored area at the end of the run.

| Quantity | Area (m²) | Share of fused map |
|---|---|---|
| Agent 1, own map | *see run output* | — |
| Agent 2, own map | *see run output* | — |
| Fused fleet map | *see run output* | 100 % |

---

## 6. Relation to the epistemic layer

This iteration is deliberately confined to the substrate, and each of its
components has a defined role in the formalism that follows.

- **The fused lattice** is the domain on which the symbolic extraction function
  of OE3 acts. Translating $M$ into propositional atoms over regions is a
  discretisation of an already order-independent structure, which is what makes
  the translation well posed.
- **The link indicator $\ell_i(t)$** is the observable that selects which
  epistemic event applies. A delivery while $\ell_i = 1$ is a public
  announcement in the DEL sense; exploration while $\ell_i = 0$ is a private
  sensing action whose effect on the fleet's knowledge is deferred.
- **The join-semilattice property** establishes distributed knowledge among the
  agents whose grids have been delivered, and no more. Higher-order attitudes —
  what agent $i$ believes agent $j$ knows — have no representation at this
  level, by construction. Supplying them is the substantive content of the DEL
  layer, not an incremental extension of the fusion operator.
- **The individual/fused coverage gap** of Figure [3](#figure-3) is the
  quantitative signal against which the epistemic planner is to be evaluated
  under OE4.

---

## 7. Limitations of this iteration

1. **Known relative initialisation.** The transforms $T_i$ are taken from the
   deployment configuration rather than estimated. Recovering them from
   inter-agent loop closure is the geometric half of collaborative SLAM and is
   not attempted here; the fusion operator is agnostic to their provenance, so
   the substitution is local.
2. **Deterministic fusion.** $\oplus$ acts on occupancy values, not on
   distributions. Cell-wise covariance is discarded, so the fused map carries no
   calibrated confidence. This is consistent with the project's stated scope,
   which restricts epistemic uncertainty to the discrete case.
3. **Reactive exploration.** Agents do not reason about where to go. Exploration
   is a fixed laser-driven policy, and no frontier assignment or task allocation
   is performed.
4. **No symbolic layer.** Neither the EPDDL planner nor the classical PDDL
   baseline required for the comparative evaluation is present in this
   iteration.
5. **Static planar worlds.** Consistent with the declared scope: two dimensions,
   fixed obstacles, two to three agents.

---

## References

<a name="ref1"></a>
[1] H. Durrant-Whyte and T. Bailey, "Simultaneous localization and mapping:
part I," *IEEE Robotics & Automation Magazine*, vol. 13, no. 2, pp. 99–110, 2006.

<a name="ref2"></a>
[2] P.-Y. Lajoie, B. Ramtoula, F. Wu and G. Beltrame, "Towards collaborative
simultaneous localization and mapping: a survey of the current research
landscape," *Field Robotics*, vol. 2, pp. 971–1000, 2022.

<a name="ref3"></a>
[3] S. Macenski and I. Jambrecic, "SLAM Toolbox: SLAM for the dynamic world,"
*Journal of Open Source Software*, vol. 6, no. 61, p. 2783, 2021.

<a name="ref4"></a>
[4] T. Bolander and M. B. Andersen, "Epistemic planning for single- and
multi-agent systems," *Journal of Applied Non-Classical Logics*, vol. 21, no. 1,
pp. 9–34, 2011.

<a name="ref5"></a>
[5] H. van Ditmarsch, W. van der Hoek and B. Kooi, *Dynamic Epistemic Logic*.
Springer, 2008.
