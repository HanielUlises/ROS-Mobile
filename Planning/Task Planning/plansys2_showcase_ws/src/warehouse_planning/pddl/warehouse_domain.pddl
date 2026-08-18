;; Warehouse logistics: a classical (deterministic, fully observable) task
;; planning domain for a fleet of ground robots moving crates from storage bays
;; to a shipping dock.
;;
;; Three deliberate properties, because the point of the showcase is what the
;; planner does with them as the fleet grows:
;;
;;   * `move` is the only action with a variable duration, taken from the
;;     `travel_time` function whose values are metric distances over the
;;     vehicle's cruise speed. Makespan is therefore in seconds of wall clock,
;;     not in action counts, and a plan that sends the wrong robot is penalised
;;     by exactly the distance it wasted.
;;   * Nothing in the domain assigns crates to robots. Every robot can serve
;;     every crate; the allocation is entirely the planner's to discover, which
;;     is what makes the single-agent and multi-agent runs comparable.
;;   * A waypoint is a *place* that holds one vehicle. This is where a parallel
;;     plan has to serialise, and it took two revisions to get right.
;;
;; On waypoints as places. The first version of this domain held a `dock_free`
;; token that a `drop` took and returned, which serialises the handovers but
;; says nothing about the floor they happen on: two robots could be `robot_at`
;; the same dock at once, one waiting its turn — consistent in the model,
;; impossible in the warehouse. Every multi-robot run ended exactly there, with
;; the second vehicle stopped half a metre short of a dock its plan said was
;; free and its `move` timing out.
;;
;; Making the dock itself exclusive fixed that and moved the failure one node
;; upstream: with three and four robots the queue formed on the corridor node
;; *in front of* the dock, which the model still treated as having room for
;; everybody. Single occupancy is therefore asserted of every waypoint, which is
;; both the general statement of the constraint and the one that needs no
;; special-case actions: `(wp_clear ?to)` is taken when a vehicle commits to
;; driving somewhere and `(wp_clear ?from)` is returned when it has actually
;; left.
(define (domain warehouse)
(:requirements :strips :typing :durative-actions :fluents :negative-preconditions)

(:types
  robot waypoint crate - object
)

(:predicates
  (robot_at ?r - robot ?wp - waypoint)
  (crate_at ?c - crate ?wp - waypoint)
  (holding ?r - robot ?c - crate)
  (gripper_free ?r - robot)
  (connected ?from - waypoint ?to - waypoint)
  ;; True while no vehicle stands on this waypoint. The physical capacity of
  ;; the place, held as a token so the planner has to schedule around it — the
  ;; single most consequential predicate in the domain, and the one whose
  ;; absence ended the first three multi-robot runs.
  (wp_clear ?wp - waypoint)
  ;; Static: which waypoints are shipping docks. A `dock` subtype would say the
  ;; same thing more elegantly and is what this domain used for one revision,
  ;; but POPF does not bind objects of a subtype to a supertype parameter, so a
  ;; robot could never `move` onto a dock and the goal was reported unreachable.
  ;; A flat type with a static predicate is the portable statement.
  (is_dock ?wp - waypoint)
  (delivered ?c - crate)
)

(:functions
  ;; seconds, derived from the map: segment length / cruise speed + turn cost
  (travel_time ?from - waypoint ?to - waypoint)
)

;; Drive one edge of the roadmap. The robot leaves its origin at the start of
;; the action and only arrives at the end, so no plan can assume a vehicle is
;; at both ends of an edge while it is traversing it.
(:durative-action move
  :parameters (?r - robot ?from - waypoint ?to - waypoint)
  :duration (= ?duration (travel_time ?from ?to))
  :condition (and
    (at start (robot_at ?r ?from))
    (at start (wp_clear ?to))
    (over all (connected ?from ?to)))
  :effect (and
    (at start (not (robot_at ?r ?from)))
    (at start (not (wp_clear ?to)))
    (at end (robot_at ?r ?to))
    (at end (wp_clear ?from)))
)

(:durative-action pick
  :parameters (?r - robot ?c - crate ?wp - waypoint)
  :duration (= ?duration 8)
  :condition (and
    (over all (robot_at ?r ?wp))
    (at start (crate_at ?c ?wp))
    (at start (gripper_free ?r)))
  :effect (and
    (at start (not (crate_at ?c ?wp)))
    (at start (not (gripper_free ?r)))
    (at end (holding ?r ?c)))
)

;; Handing the crate over. No separate capacity token: a robot can only be here
;; if it holds this waypoint's standing place, so the handovers are serialised
;; by the same fact that keeps the vehicles apart.
(:durative-action drop
  :parameters (?r - robot ?c - crate ?d - waypoint)
  :duration (= ?duration 8)
  :condition (and
    (over all (robot_at ?r ?d))
    (over all (is_dock ?d))
    (at start (holding ?r ?c)))
  :effect (and
    (at start (not (holding ?r ?c)))
    (at end (crate_at ?c ?d))
    (at end (gripper_free ?r))
    (at end (delivered ?c)))
)
)
