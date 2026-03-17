(define (domain robot_logistics)
  (:requirements :strips :typing :adl :durative-actions)

  (:types
    robot location object - entity
  )

  (:predicates
    (robot_at ?r - robot ?l - location)
    (object_at ?o - object ?l - location)
    (robot_carrying ?r - robot ?o - object)
    (robot_free ?r - robot)
    (connected ?from - location ?to - location)
    (object_delivered ?o - object ?l - location)
  )

  (:durative-action move
    :parameters (?r - robot ?from - location ?to - location)
    :duration (= ?duration 5)
    :condition (and
      (at start (robot_at ?r ?from))
      (over all (connected ?from ?to))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
    )
  )

  (:durative-action pick_object
    :parameters (?r - robot ?o - object ?l - location)
    :duration (= ?duration 2)
    :condition (and
      (at start (robot_at ?r ?l))
      (at start (object_at ?o ?l))
      (at start (robot_free ?r))
      (over all (robot_at ?r ?l))
    )
    :effect (and
      (at start (not (object_at ?o ?l)))
      (at start (not (robot_free ?r)))
      (at end (robot_carrying ?r ?o))
    )
  )

  (:durative-action place_object
    :parameters (?r - robot ?o - object ?l - location)
    :duration (= ?duration 2)
    :condition (and
      (at start (robot_at ?r ?l))
      (at start (robot_carrying ?r ?o))
      (over all (robot_at ?r ?l))
    )
    :effect (and
      (at start (not (robot_carrying ?r ?o)))
      (at end (object_at ?o ?l))
      (at end (robot_free ?r))
      (at end (object_delivered ?o ?l))
    )
  )
)
