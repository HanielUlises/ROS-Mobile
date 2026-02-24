(define (problem robot_delivery_problem)
  (:domain robot_logistics)

  (:objects
    robot1 - robot
    warehouse office kitchen - location
    package1 package2 - object
  )

  (:init
    (robot_at robot1 warehouse)
    (robot_free robot1)
    (object_at package1 warehouse)
    (object_at package2 warehouse)
    (connected warehouse office)
    (connected office warehouse)
    (connected office kitchen)
    (connected kitchen office)
    (connected warehouse kitchen)
    (connected kitchen warehouse)
  )

  (:goal (and
    (object_delivered package1 office)
    (object_delivered package2 kitchen)
  ))
)
