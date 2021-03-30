(define (domain apartment)
(:requirements :strips :equality :typing :durative-actions)
(:types
    corridor room zone - place 
    object 
    robot
)

(:predicates
    (robotAt ?rob - robot ?x - place)
    (robotIdle ?rob - robot)
    (robotGrab ?rob - robot ?o - object)

    (objectAt ?o - object ?x - place)

    (placesTogether ?x1 - place ?x2 - place)
)

(:durative-action move
    :parameters (?rob - robot ?from - place ?to - place)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (robotAt ?rob ?from)
            (robotIdle ?rob)
        ))
        (over all (and 
            (placesTogether ?from ?to)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotAt ?rob ?from))
            (not (robotIdle ?rob))
        ))
        (at end (and
            (robotAt ?rob ?to) 
            (robotIdle ?rob)
        ))
    )
)

(:durative-action grab_object
    :parameters (?rob - robot ?o - object ?x - place)
    :duration (= ?duration 5)
    :condition (and
        (at start (and 
            (objectAt ?o ?x)
            (robotIdle ?rob)
        ))
        (over all (and 
            (robotAt ?rob ?x)
        ))
    )
    :effect (and 
        (at start (and
            (not (objectAt ?o ?x))
            (not (robotIdle ?rob))
        ))
        (at end (and 
            (robotGrab ?rob ?o)
            (robotIdle ?rob)
        ))
    )
)

(:durative-action release_object
    :parameters (?rob - robot ?o - object ?x - place)
    :duration (= ?duration 5)
    :condition (and 
        (at start (and 
            (robotGrab ?rob ?o)
            (robotIdle ?rob)
        ))
        (over all (and 
            (robotAt ?rob ?x)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotGrab ?rob ?o))
            (not (robotIdle ?rob))
        ))
        (at end (and 
            (objectAt ?o ?x)
            (robotIdle ?rob)
        ))
    )
)
)