(define (problem patrol_mission)
  (:domain tb3_patrol)

  (:objects
    tb3 - robot
    wp0 wp1 wp2 wp3 - location
  )

  (:init
    (at tb3 wp0)
    (path wp0 wp1)
    (path wp1 wp0)
    (path wp1 wp2)
    (path wp2 wp1)
    (path wp2 wp3)
    (path wp3 wp2)
    (path wp3 wp0)
    (path wp0 wp3)
  )

  (:goal
    (and
      (inspected wp1)
      (inspected wp2)
      (inspected wp3)
      (reported  wp1)
      (reported  wp2)
      (reported  wp3)
      (at tb3 wp0)
    )
  )

)