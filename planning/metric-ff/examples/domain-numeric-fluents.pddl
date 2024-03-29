(define
    (domain simple)
    (:requirements :strips :fluents)
    (:functions
        (fuel ?a)
    )
    (:predicates
        (object-is-ready ?a)
        (task-complete ?a)
    )
    (:action action1
        :parameters (?obj)
        :precondition (and 
            (object-is-ready ?obj)
        )
        :effect (and
            (task-complete ?obj)
            (increase (fuel ?obj) 100)
        )
    )
)
