;Header and description


; EXAMPLE CREATED BY fmrico

(define (domain moving)

;remove requirements that are not needed
(:requirements :strips :fluents :durative-actions :typing :conditional-effects :equality)

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

(:durative-action move
    :parameters (?p - person ?from - house ?to - house)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
        (person_at ?p ?from)
        (forall (?g - good)
            (carry ?p ?g)
        )
        ))
    )
    :effect (and 
        (at start (and 
        (not (person_at ?p ?from))
        ))
        (at end (and
        (person_at ?p ?to) 
        ))
    )
)


(:durative-action do_work
    :parameters (?p - person ?h - house ?t - task)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (person_at ?p ?h)
        ))
    )
    :effect (and 
        (at start (and 
        (person_working ?p ?h)
        ))
        (at end (and 
        ))
    )
)


(:durative-action pick
    :parameters (?p - person ?g - good ?h - house)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
        (person_at ?p ?h)
        (good_at ?g ?h)
        ))
        
    )
    :effect (and 
        (at start (and 
        (person_at ?p ?h)
        (good_at ?g ?h)
        ))
        (at end (and 
        (not (good_at ?g ?h))
        (carry ?p ?g)
        ))
    )
)


(:durative-action place
    :parameters (?p - person ?g - good ?h - house)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
        (carry ?p ?g)
        (person_at ?p ?h)
        ))

    )
    :effect (and 
        (at start (and 
        (not (carry ?p ?g))
        ))
        (at end (and 
        (good_at ?g ?h)
        ))
    )
)




)