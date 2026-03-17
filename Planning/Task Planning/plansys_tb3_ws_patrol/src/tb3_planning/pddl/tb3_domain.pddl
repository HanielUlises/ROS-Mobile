(define (domain tb3_patrol)
  (:requirements :strips :typing :durative-actions)

  (:types robot location)

  (:predicates
    (at        ?r - robot ?l - location)
    (inspected ?l - location)
    (path      ?from - location ?to - location)
    (reported  ?l - location)
  )

  (:durative-action move
    :parameters (?r - robot ?from - location ?to - location)
    :duration   (= ?duration 10)
    :condition  (and
      (at start (at ?r ?from))
      (at start (path ?from ?to))
    )
    :effect (and
      (at start (not (at ?r ?from)))
      (at end   (at ?r ?to))
    )
  )

  (:durative-action inspect
    :parameters (?r - robot ?l - location)
    :duration   (= ?duration 8)
    :condition  (and
      (at start (at ?r ?l))
    )
    :effect (and
      (at end (inspected ?l))
    )
  )

  (:durative-action report
    :parameters (?r - robot ?l - location)
    :duration   (= ?duration 5)
    :condition  (and
      (at start (at ?r ?l))
      (at start (inspected ?l))
    )
    :effect (and
      (at end (reported ?l))
    )
  )

)