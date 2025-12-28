(define (domain simple)
    (:requirements :strips :typing :adl :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        robot
        waypoint
    )

    ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:predicates
        (robot_at ?r - robot ?wp - waypoint)
        (not_robot_at ?r - robot ?wp - waypoint)
        (finished ?r - robot)
    )

    ;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:functions
        (spotted_waypoint ?wp - waypoint)
        (reached_goals ?r - robot)
    )

    ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:action move_to_waypoint
        :parameters (?r - robot ?wp1 - waypoint ?wp2 - waypoint)
        :precondition (and 
            (robot_at ?r ?wp1)
            (not_robot_at ?r ?wp2)
            (< (spotted_waypoint ?wp2) 1)
        )
        :effect (and 
            (robot_at ?r ?wp2)
            (not (not_robot_at ?r ?wp2))
            (not (robot_at ?r ?wp1))
            (not_robot_at ?r ?wp1)
        )
    )

    (:action rotation
        :parameters (?r - robot ?wp - waypoint)
        :precondition (and 
            (robot_at ?r ?wp)
            (< (spotted_waypoint ?wp) 1)
        )
        :effect (and
            (increase (spotted_waypoint ?wp) 1)
            (increase (reached_goals ?r) 1)
        )
    )

    (:action checks_end
        :parameters (?r - robot)
        :precondition (>= (reached_goals ?r) 4)
        :effect (and
            (finished ?r)
        )
    )
)