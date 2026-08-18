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
;;   * A dock has finite capacity, held as a `dock_free` token taken at the
;;     start of a `drop` and returned at its end. With one robot the token is
;;     never contested; with four it is the only place where the agents can
;;     genuinely block one another, so it is where a parallel plan has to
;;     serialise.
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
  (is_dock ?wp - waypoint)
  (dock_free ?wp - waypoint)
  (delivered ?c - crate)
)

(:functions
  ;; seconds, derived from the map: segment length / cruise speed + turn cost
  (travel_time ?from - waypoint ?to - waypoint)
)

;; Drive one edge of the roadmap. The robot leaves its origin at the start of
;; the action and only arrives at the end, so no two-robot plan can assume a
;; vehicle is at both ends of an edge while it is traversing it.
(:durative-action move
  :parameters (?r - robot ?from - waypoint ?to - waypoint)
  :duration (= ?duration (travel_time ?from ?to))
  :condition (and
    (at start (robot_at ?r ?from))
    (over all (connected ?from ?to)))
  :effect (and
    (at start (not (robot_at ?r ?from)))
    (at end (robot_at ?r ?to)))
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

(:durative-action drop
  :parameters (?r - robot ?c - crate ?wp - waypoint)
  :duration (= ?duration 8)
  :condition (and
    (over all (robot_at ?r ?wp))
    (over all (is_dock ?wp))
    (at start (holding ?r ?c))
    (at start (dock_free ?wp)))
  :effect (and
    (at start (not (dock_free ?wp)))
    (at start (not (holding ?r ?c)))
    (at end (crate_at ?c ?wp))
    (at end (gripper_free ?r))
    (at end (dock_free ?wp))
    (at end (delivered ?c)))
)
)
