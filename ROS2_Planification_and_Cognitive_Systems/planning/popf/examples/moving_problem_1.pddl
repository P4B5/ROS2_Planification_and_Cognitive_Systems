; EXAMPLE CREATED BY fmrico

(define (problem moving-1) (:domain moving)
(:objects 
    paco - person
    house_alcorcon house_fuenlabrada - house
    guitar ball cat - good
    playing_guitar - task
)

(:init
    (person_at paco house_alcorcon)
    (good_at guitar house_alcorcon)
    (good_at cat house_alcorcon)
    (good_at ball house_alcorcon)
)

(:goal (and
   (person_at paco house_fuenlabrada)
    (forall (?g - good)
        (good_at ?g house_fuenlabrada)
    )
    ; (good_at guitar house_fuenlabrada)
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
