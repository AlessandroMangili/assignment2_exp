(define (domain simple)
    (:requirements :strips :typing :adl :fluents)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        robot
        waypoint
    )


    (:constants
        starting_position - waypoint
    )

    ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:predicates
        (robot_at ?r - robot ?wp - waypoint)
        (not_robot_at ?r - robot ?wp - waypoint)
    )

    ;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:functions
        (spotted_waypoint ?wp - waypoint)
        (reached_goals ?r - robot)
    )

    ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:action move
        :parameters (?r - robot ?wp1 ?wp2 - waypoint)
        :precondition (and (robot_at ?r ?wp1) (not_robot_at ?r ?wp2) (< (spotted_waypoint ?wp2) 1))
        :effect (and 
            (robot_at ?r ?wp2)
            (not (not_robot_at ?r ?wp2))
            (not (robot_at ?r ?wp1))
            (not_robot_at ?r ?wp1)
        )
    )

    (:action rotation
        :parameters (?r - robot ?wp - waypoint)
        :precondition (and (robot_at ?r ?wp) (< (spotted_waypoint ?wp) 1))
        :effect (and
            (increase (spotted_waypoint ?wp) 1)
            (increase (reached_goals ?r) 1)
        )
    )
)
