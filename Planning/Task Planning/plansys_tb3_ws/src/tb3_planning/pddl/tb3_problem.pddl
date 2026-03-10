(define (problem tb3_problem)
  (:domain tb3_navigation)
  (:objects
    tb3 - robot
    wp1 wp2 wp3 - location
  )
  (:init
    (at tb3 wp1)
  )
  (:goal
    (and (at tb3 wp3))
  )
)
