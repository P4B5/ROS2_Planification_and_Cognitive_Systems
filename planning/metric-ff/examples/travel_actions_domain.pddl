;Header and description

; EXAMPLE CREATED BY fmrico

(define (domain travel)

;remove requirements that are not needed
(:requirements :strips :fluents :typing :conditional-effects :equality)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
vehicle city
)

; un-comment following line if constants are needed
;(:constants )

(:predicates ;todo: define predicates here
    (car_at ?v - vehicle ?c - city)
    (is_road ?from ?to - city)
)


(:functions ;todo: define numeric functions here
    (distance ?from ?to - city)
    (speed ?v - vehicle)
    (distance_driven)
    ; (fuel_enough)
)

(:action drive
    :parameters (?v - vehicle ?from ?to - city)
    :precondition (and 
        (car_at ?v ?from)
        (is_road ?from ?to)
    )
    :effect (and 
        (not (car_at ?v ?from))
        (car_at ?v ?to)
        (increase (distance_driven) (distance ?from ?to))
    )
)
)