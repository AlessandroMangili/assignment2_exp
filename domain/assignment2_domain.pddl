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
    (:durative-action move
        :parameters (?r - robot ?wp1 - waypoint ?wp2 - waypoint)
        :duration ( = ?duration 5)
        :condition (and 
            (at start(robot_at ?r ?wp1))
            (at start(not_robot_at ?r ?wp2))
            (at start(< (spotted_waypoint ?wp2) 1))
        )
        :effect (and 
            (at end(robot_at ?r ?wp2))
            (at end(not (not_robot_at ?r ?wp2)))
            (at end(not (robot_at ?r ?wp1)))
            (at end(not_robot_at ?r ?wp1))
        )
    )

    (:durative-action rotation
        :parameters (?r - robot ?wp - waypoint)
        :duration ( = ?duration 5)
        :condition (and 
            (at start(robot_at ?r ?wp))
            (at start(< (spotted_waypoint ?wp) 1))
        )
        :effect (and
            (at start(increase (spotted_waypoint ?wp) 1))
            (at end(increase (reached_goals ?r) 1))
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
