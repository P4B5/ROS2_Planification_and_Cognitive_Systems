(define (domain apartment)
(:requirements :strips :equality :typing :durative-actions)
(:types
    corridor 
    room 
    zone
    object 
    robot
)

(:predicates
    (robotAtcorridor ?rob - robot ?c - corridor)
    (robotAtroom ?rob - robot ?r - room)
    (robotAtzone ?rob - robot ?z - zone)
    (robotIdle ?rob - robot)
    (robotGrab ?rob - robot ?o - object)

    (objectAtcorridor ?o - object ?c - corridor)
    (objectAtroom ?o - object ?r - room)
    (objectAtzone ?o - object ?z - zone)

    (corridorBesideroom ?c - corridor ?r - room)
    (roomBesidecorridor ?r - room ?c - corridor)
    (roomBesidezone ?r - room ?z - zone)
    (zoneBesideroom ?z - zone ?r - room)
    (roomBesideroom ?r - room ?r - room)
)

(:durative-action move_corridor_to_room
    :parameters (?rob - robot ?from - corridor ?to - room)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (robotAtcorridor ?rob ?from)
            (robotIdle ?rob)
        ))
        (over all (and 
            (corridorBesideroom ?from ?to)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotAtcorridor ?rob ?from))
            (not (robotIdle ?rob))
        ))
        (at end (and
            (robotAtroom ?rob ?to) 
            (robotIdle ?rob)
        ))
    )
)

(:durative-action move_room_to_corridor
    :parameters (?rob - robot ?from - room ?to - corridor)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (robotAtroom ?rob ?from)
            (robotIdle ?rob)
        ))
        (over all (and 
            (roomBesidecorridor ?from ?to)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotAtroom ?rob ?from))
            (not (robotIdle ?rob))
        ))
        (at end (and
            (robotAtcorridor ?rob ?to) 
            (robotIdle ?rob)
        ))
    )
)

(:durative-action move_room_to_zone
    :parameters (?rob - robot ?from - room ?to - zone)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (robotAtroom ?rob ?from)
            (robotIdle ?rob)
        ))
        (over all (and 
            (roomBesidezone ?from ?to)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotAtroom ?rob ?from))
            (not (robotIdle ?rob))
        ))
        (at end (and
            (robotAtzone ?rob ?to) 
            (robotIdle ?rob)
        ))
    )
)

(:durative-action move_zone_to_room
    :parameters (?rob - robot ?from - zone ?to - room)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (robotAtzone ?rob ?from)
            (robotIdle ?rob)
        ))
        (over all (and 
            (zoneBesideroom ?from ?to)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotAtzone ?rob ?from))
            (not (robotIdle ?rob))
        ))
        (at end (and
            (robotAtroom ?rob ?to) 
            (robotIdle ?rob)
        ))
    )
)

(:durative-action move_room_to_room
    :parameters (?rob - robot ?from - room ?to - room)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (robotAtroom ?rob ?from)
            (robotIdle ?rob)
        ))
        (over all (and 
            (roomBesideroom ?from ?to)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotAtroom ?rob ?from))
            (not (robotIdle ?rob))
        ))
        (at end (and
            (robotAtroom ?rob ?to) 
            (robotIdle ?rob)
        ))
    )
)

(:durative-action grab_object_corridor
    :parameters (?rob - robot ?o - object ?x - corridor)
    :duration (= ?duration 5)
    :condition (and
        (at start (and 
            (objectAtcorridor ?o ?x)
            (robotIdle ?rob)
        ))
        (over all (and 
            (robotAtcorridor ?rob ?x)
        ))
    )
    :effect (and 
        (at start (and
            (not (objectAtcorridor ?o ?x))
            (not (robotIdle ?rob))
        ))
        (at end (and 
            (robotGrab ?rob ?o)
            (robotIdle ?rob)
        ))
    )
)

(:durative-action grab_object_room
    :parameters (?rob - robot ?o - object ?x - room)
    :duration (= ?duration 5)
    :condition (and
        (at start (and 
            (objectAtroom ?o ?x)
            (robotIdle ?rob)
        ))
        (over all (and 
            (robotAtroom ?rob ?x)
        ))
    )
    :effect (and 
        (at start (and
            (not (objectAtroom ?o ?x))
            (not (robotIdle ?rob))
        ))
        (at end (and 
            (robotGrab ?rob ?o)
            (robotIdle ?rob)
        ))
    )
)

(:durative-action grab_object_zone
    :parameters (?rob - robot ?o - object ?x - zone)
    :duration (= ?duration 5)
    :condition (and
        (at start (and 
            (objectAtzone ?o ?x)
            (robotIdle ?rob)
        ))
        (over all (and 
            (robotAtzone ?rob ?x)
        ))
    )
    :effect (and 
        (at start (and
            (not (objectAtzone ?o ?x))
            (not (robotIdle ?rob))
        ))
        (at end (and 
            (robotGrab ?rob ?o)
            (robotIdle ?rob)
        ))
    )
)

(:durative-action release_object_corridor
    :parameters (?rob - robot ?o - object ?x - corridor)
    :duration (= ?duration 5)
    :condition (and 
        (at start (and 
            (robotGrab ?rob ?o)
            (robotIdle ?rob)
        ))
        (over all (and 
            (robotAtcorridor ?rob ?x)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotGrab ?rob ?o))
            (not (robotIdle ?rob))
        ))
        (at end (and 
            (objectAtcorridor ?o ?x)
            (robotIdle ?rob)
        ))
    )
)

(:durative-action release_object_room
    :parameters (?rob - robot ?o - object ?x - room)
    :duration (= ?duration 5)
    :condition (and 
        (at start (and 
            (robotGrab ?rob ?o)
            (robotIdle ?rob)
        ))
        (over all (and 
            (robotAtroom ?rob ?x)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotGrab ?rob ?o))
            (not (robotIdle ?rob))
        ))
        (at end (and 
            (objectAtroom ?o ?x)
            (robotIdle ?rob)
        ))
    )
)

(:durative-action release_object_zone
    :parameters (?rob - robot ?o - object ?x - zone)
    :duration (= ?duration 5)
    :condition (and 
        (at start (and 
            (robotGrab ?rob ?o)
            (robotIdle ?rob)
        ))
        (over all (and 
            (robotAtzone ?rob ?x)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotGrab ?rob ?o))
            (not (robotIdle ?rob))
        ))
        (at end (and 
            (objectAtzone ?o ?x)
            (robotIdle ?rob)
        ))
    )
)
)