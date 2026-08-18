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
;;   * The dock is a *place* with room for one vehicle, not merely a service
;;     that handles one crate at a time. This is where a parallel plan has to
;;     serialise, and the distinction is the whole of the revision below.
;;
;; On the dock as a place. The first version of this domain held a `dock_free`
;; token that a `drop` took and returned, which serialises the handovers but
;; says nothing about the floor they happen on. Two robots could therefore be
;; `robot_at` the same dock simultaneously, one waiting its turn to drop —
;; consistent in the model, impossible in the warehouse. In every multi-robot
;; run that is exactly how the mission ended: the second vehicle stopped half a
;; metre short of a dock its plan said was available, and its `move` timed out.
;; The token is now taken by *entering* the dock and returned by *leaving* it,
;; which needs the two dedicated actions below, because a single generic `move`
;; has no way to say "this destination admits one vehicle".
(define (domain warehouse)
(:requirements :strips :typing :durative-actions :fluents :negative-preconditions)

(:types
  robot crate - object
  waypoint - object
  dock - waypoint
)

(:predicates
  (robot_at ?r - robot ?wp - waypoint)
  (crate_at ?c - crate ?wp - waypoint)
  (holding ?r - robot ?c - crate)
  (gripper_free ?r - robot)
  (connected ?from - waypoint ?to - waypoint)
  ;; True while no vehicle stands on this dock: the physical capacity of the
  ;; place, held as a token so the planner has to schedule around it.
  (dock_clear ?d - dock)
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
    (over all (connected ?from ?to)))
  :effect (and
    (at start (not (robot_at ?r ?from)))
    (at end (robot_at ?r ?to)))
)

;; Drive onto a dock, taking its one standing place for the whole approach.
;; The token is taken at the *start* of the drive rather than on arrival: a
;; vehicle committed to a dock already occupies the approach to it, and taking
;; the token on arrival would let two robots drive at the same slot and
;; discover the conflict with their bumpers.
(:durative-action enter_dock
  :parameters (?r - robot ?from - waypoint ?d - dock)
  :duration (= ?duration (travel_time ?from ?d))
  :condition (and
    (at start (robot_at ?r ?from))
    (at start (dock_clear ?d))
    (over all (connected ?from ?d)))
  :effect (and
    (at start (not (robot_at ?r ?from)))
    (at start (not (dock_clear ?d)))
    (at end (robot_at ?r ?d)))
)

;; Drive off a dock, returning the standing place at the end of the drive —
;; when the vehicle is actually clear of it, not when it starts to move.
(:durative-action leave_dock
  :parameters (?r - robot ?d - dock ?to - waypoint)
  :duration (= ?duration (travel_time ?d ?to))
  :condition (and
    (at start (robot_at ?r ?d))
    (over all (connected ?d ?to)))
  :effect (and
    (at start (not (robot_at ?r ?d)))
    (at end (robot_at ?r ?to))
    (at end (dock_clear ?d)))
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
;; if it holds the dock's standing place, so the handovers are serialised by
;; the same fact that keeps the vehicles apart.
(:durative-action drop
  :parameters (?r - robot ?c - crate ?d - dock)
  :duration (= ?duration 8)
  :condition (and
    (over all (robot_at ?r ?d))
    (at start (holding ?r ?c)))
  :effect (and
    (at start (not (holding ?r ?c)))
    (at end (crate_at ?c ?d))
    (at end (gripper_free ?r))
    (at end (delivered ?c)))
)
)
