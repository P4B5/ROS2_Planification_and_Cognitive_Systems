(define (domain cognitive_apartment)
(:requirements :strips :equality :typing :durative-actions)
(:types
    corridor room zone - place 
    object 
    robot
)

(:predicates
    (robot_at ?rob - robot ?x - place)
    (robot_idle ?rob - robot)
    (robot_grab ?rob - robot ?o - object)

    (object_at ?o - object ?x - place)

    (places_together ?x1 - place ?x2 - place)

    (place_explored ?x - place)
)

(:durative-action move
    :parameters (?rob - robot ?from - place ?to - place)
    :duration (= ?duration 1)
    :condition (and 
        (at start (robot_at ?rob ?from))
        (at start (robot_idle ?rob))
    )
    :effect (and 
        (at start (not (robot_at ?rob ?from)))
        (at start (not (robot_idle ?rob)))
        (at end (robot_at ?rob ?to))
        (at end (robot_idle ?rob))
    )
)

(:durative-action explore
    :parameters (?rob - robot ?x - place)
    :duration (= ?duration 1)
    :condition (and 
        (at start (robot_idle ?rob))
        (over all (robot_at ?rob ?x))
    )
    :effect (and
        (at start (not (robot_idle ?rob)))
        (at end (place_explored ?x))
        (at end (robot_idle ?rob))
    )
)

(:durative-action grab_object
    :parameters (?rob - robot ?o - object ?x - place)
    :duration (= ?duration 5)
    :condition (and
        (at start (object_at ?o ?x))
        (at start (robot_idle ?rob))
        (over all (robot_at ?rob ?x))
    )
    :effect (and 
        (at start (not (object_at ?o ?x)))
        (at start (not (robot_idle ?rob)))
        (at end (robot_grab ?rob ?o))
        (at end (robot_idle ?rob))
    )
)

(:durative-action release_object
    :parameters (?rob - robot ?o - object ?x - place)
    :duration (= ?duration 5)
    :condition (and 
        (at start (robot_grab ?rob ?o))
        (at start (robot_idle ?rob))
        (over all (robot_at ?rob ?x))
    )
    :effect (and 
        (at start (not (robot_grab ?rob ?o)))
        (at start (not (robot_idle ?rob)))
        (at end (object_at ?o ?x))
        (at end (robot_idle ?rob))
    )
)
)