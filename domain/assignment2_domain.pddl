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
        (not_rotating ?r - robot)
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
            (> (spotted_waypoint ?wp1) 0)
        )
        :effect (and 
            (robot_at ?r ?wp2)
            (not (not_robot_at ?r ?wp2))
            (not (robot_at ?r ?wp1))
            (not_robot_at ?r ?wp1)
        )
    )

    (:durative-action rotation
        :parameters (?r - robot ?wp - waypoint)
        :duration (= ?duration 5)
        :condition (and 
            (at start(robot_at ?r ?wp))
            (at start(< (spotted_waypoint ?wp) 1))
        )
        :effect (and
            (at start(increase (spotted_waypoint ?wp) 1))
            (at start (not (not_rotating ?r)))
            (at end (increase (reached_goals ?r) 1))
            (at end (not_rotating ?r))
        )
    )

    ;(:action rotation
    ;    :parameters (?r - robot ?wp - waypoint)
    ;    :precondition (and 
    ;        (robot_at ?r ?wp)
    ;        (< (spotted_waypoint ?wp) 1)
    ;    )
    ;    :effect (and
    ;        (increase (spotted_waypoint ?wp) 1)
    ;        (increase (reached_goals ?r) 1)
    ;    )
    ;)

    (:action check_end
        :parameters (?r - robot)
        :precondition (and
            (>= (reached_goals ?r) 4)
            (not_rotating ?r)
        )
        :effect (and
            (finished ?r)
        )
    )
)