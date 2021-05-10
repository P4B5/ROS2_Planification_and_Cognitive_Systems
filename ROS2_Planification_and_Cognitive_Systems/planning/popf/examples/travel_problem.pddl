
; EXAMPLE CREATED BY fmrico

(define (problem travel_problem1) (:domain travel)
(:objects 
    minicooper - vehicle
    fuenlabrada leganes mostoles alcorcon madrid - city
)

(:init
    ;todo: put the initial state's facts and numeric values here
    (car_at minicooper fuenlabrada)

    (is_road fuenlabrada mostoles)
    (is_road mostoles fuenlabrada)

    (is_road fuenlabrada leganes)
    (is_road leganes fuenlabrada)

    (is_road mostoles alcorcon)
    (is_road alcorcon mostoles)

    (is_road madrid alcorcon)
    (is_road alcorcon madrid)

    (is_road madrid leganes)
    (is_road leganes madrid)

    (is_road leganes alcorcon)
    (is_road alcorcon leganes)

    ; (fuel_enough)

    (= (speed minicooper)100)

    (= (distance_driven)0)

    (= (distance fuenlabrada mostoles)10)
    (= (distance mostoles fuenlabrada)10)

    (= (distance fuenlabrada leganes)100)
    (= (distance leganes fuenlabrada)100)

    (= (distance mostoles alcorcon)5)
    (= (distance alcorcon mostoles)5)

    (= (distance madrid alcorcon)10)
    (= (distance alcorcon madrid)10)
     
    (= (distance leganes alcorcon)15)
    (= (distance alcorcon leganes)15)


)

(:goal (and
    (car_at minicooper madrid)
))




(:metric minimize (distance_driven))
)
