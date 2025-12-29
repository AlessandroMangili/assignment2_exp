(define (problem assignment2)
    (:domain simple)

    ;; Objects ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:objects
        r1 - robot
        wp1 wp2 wp3 wp4 starting_position - waypoint
    )

    ;; Initial state ;;;;;;;;;;;;;;;;;;;;;;;;
    (:init
        (robot_at r1 starting_position)
        (not_robot_at r1 wp1)
        (not_robot_at r1 wp2)
        (not_robot_at r1 wp3)
        (not_robot_at r1 wp4)

        (= (spotted_waypoint starting_position) 1)
        (= (spotted_waypoint wp1) 0)
        (= (spotted_waypoint wp2) 0)
        (= (spotted_waypoint wp3) 0)
        (= (spotted_waypoint wp4) 0)

        (= (reached_goals r1) 0)
    )

    ;; Goal ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:goal
        
            (finished r1)
        
    )
)