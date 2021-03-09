;Header and description


; EXAMPLE CREATED BY fmrico
; (Modified)

(define (domain moving)

;remove requirements that are not needed
(:requirements :strips :fluents :typing :conditional-effects :equality)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    good house person task
)

; un-comment following line if constants are needed
;(:constants )

(:predicates ;todo: define predicates here
    (person_at ?p - person ?h - house)
    (good_at ?g - good ?h - house) 
    (carry ?p - person ?g - good)
    (person_working ?p - person ?h - house)
)


(:functions ;todo: define numeric functions here
)


(:action move
    :parameters (?p - person ?from - house ?to - house)
    :precondition (and
        (person_at ?p ?from)
        (forall (?g - good)
            (carry ?p ?g)
        )
    )
    :effect (and
        (not (person_at ?p ?from))
        (person_at ?p ?to)
    )
)

(:action do_work
    :parameters (?p - person ?h - house ?t - task)
    :precondition (and
        (person_at ?p ?h)
    )
    :effect (and
        (person_working ?p ?h)
    )
)

(:action pick
    :parameters (?p - person ?g - good ?h - house)
    :precondition (and
        (person_at ?p ?h)
        (good_at ?g ?h)
    )
    :effect (and
        (person_at ?p ?h)
        (good_at ?g ?h)
        (not (good_at ?g ?h))
        (carry ?p ?g)
    )
)

(:action place
    :parameters (?p - person ?g - good ?h - house)
    :precondition (and
        (carry ?p ?g)
        (person_at ?p ?h)
    )
    :effect (and
        (not (carry ?p ?g))
        (good_at ?g ?h)
    )
)
)