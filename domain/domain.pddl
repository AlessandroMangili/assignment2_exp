(define (domain simple)
    (:requirements :strips :typing :adl :fluents)
 
    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        robot
        waypoint
    )
 
    ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:predicates
        (robot_at ?r - robot ?wp - waypoint)
        (not_robot_at ?r - robot ?wp - waypoint)
        (not_robot_at_2 ?r - robot ?wp - waypoint)
        (finished ?r - robot)
        (completed ?r - robot)
        (free_to_check ?r - robot)
    )
 
    ;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:functions
        (spotted_waypoint ?wp - waypoint)
        (spotted_waypoint_2 ?wp - waypoint)
        (reached_goals ?r - robot)
        (reached_goals_2 ?r - robot)
        (rotation_allowed ?r - robot)
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
 
 
    (:action check_end
        :parameters (?r - robot ?wp1 - waypoint ?wp2 - waypoint)
        :precondition (and
            (>= (reached_goals ?r) 4)
            (robot_at ?r ?wp1)
            (free_to_check ?r)
            (> (spotted_waypoint ?wp2) 9)
        )
        :effect (and
            (finished ?r)
            (not (free_to_check ?r))
            ;(increase (rotation_allowed ?r) 10)
            (robot_at ?r ?wp2)
            (not (not_robot_at ?r ?wp2))
            (not (robot_at ?r ?wp1))
            (not_robot_at ?r ?wp1)
        )
    )
 
    (:action move_to_waypoint_2
        :parameters (?r - robot ?wp1 - waypoint ?wp2 - waypoint)
        :precondition (and
            (robot_at ?r ?wp1)
            (not_robot_at_2 ?r ?wp2)
            (finished ?r)
            (< (spotted_waypoint_2 ?wp2) 1)
            (> (spotted_waypoint_2 ?wp1) 0)
        )
        :effect (and
            (robot_at ?r ?wp2)
            (not (not_robot_at_2 ?r ?wp2))
            (not (robot_at ?r ?wp1))
            (not_robot_at_2 ?r ?wp1)
            (decrease (rotation_allowed ?r) 1)
        )
    )
 
    (:action alignment_and_picture
        :parameters (?r - robot ?wp - waypoint)
        :precondition (and
            (robot_at ?r ?wp)
            (< (spotted_waypoint_2 ?wp) 1)
            (finished ?r)
            (< (rotation_allowed ?r) 10)
        )
        :effect (and
            (increase (spotted_waypoint_2 ?wp) 1)
            (increase (reached_goals_2 ?r) 1)
        )
    )
 
    (:action finalization
        :parameters (?r - robot)
        :precondition (and
            (>= (reached_goals_2 ?r) 4)
        )
        :effect (and
            (completed ?r)
        )
    )
 
 
)