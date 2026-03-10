(define (domain tb3_navigation)
  (:requirements :strips :typing :durative-actions)
  (:types
    robot location
  )
  (:predicates
    (at ?r - robot ?l - location)
  )
  (:durative-action move
    :parameters (?r - robot ?from - location ?to - location)
    :duration (= ?duration 10)
    :condition (and
      (at start (at ?r ?from))
    )
    :effect (and
      (at start (not (at ?r ?from)))
      (at end (at ?r ?to))
    )
  )
)
