# popf planer


// Ejecutado en ros2, 
// Repo oficial: https://github.com/fmrico/popf.git
//launch popf: ros2 run popf popf [domain.pddl] [problem.pddl]


//Apuntes para averiguar que soporta y que no soporta POPF con PDDL

COSAS SOPORTADAS:

	Durative actions, 
	Funciones // devuelve algo para que se utilice despues ej: distancia
	Operaciones (división, multiplicacion) Operaciones logicas (and or..)
	Not (en los efectos)
	soporta if (= from to)
	for ALL
	Usa tipado
	Constantes
	Soporta increase // y decrease 
	Metricas de minimize (para que se recorra la minima distancia por ejemplo) (no he entendido bien como va me lo tengo que remirar)
	

	:strips       Permite	añadir	o	borrar	efectos
	:typing       Permite	usar	tipos
	:equality     Permite	comparar	si	dos	objetivos	son	el	mismo
	:universal-preconditions    Permite	usar	forall	es	goals	y	precondiciones
	
	:durative-actions
	:duration-inequalities Permiten	 usar	 desigualdades	 para	 expresar	duración
	:continuous-effects Permite el uso de efectos	continuos sobre números dentro de	acciones	durativas.
	timed-initial-literals A timed initial literal is defined using the time keyword, followed by the value for the point in time which the predicate becomes true, followed by the predicate itself.
	:numeric-fluents Permite usar funciones que representan valores numericos// En popf2 si esta soportado
	
	
	
COSAS NO SOPORTADAS:

	Not en las conditions
	For all en los goal en los goals
	No soporta funciones sin parametros
	
	No soporta ADL (action description language)  es una  planificación y programación automatizadas
	
	disjunctive-preconditions (or en las precondiciones
	existential-preconditions (exist en goals y precondiciones)
	conditional-effects (misma accion con diferentes tipos)
	domain-axioms (permite utilizar axiomas (:derived (clear ?x
	:subgoals-through-axioms  Permite	usar	axiomas como	subgoals
	safety-constraints Permite definir predicados que deben ser validos al final de la ejecucion de un plan
	open-world (de normal si no hay algo definido se dice que no existe, esto lo evida
	
	:quantified-preconditions Equivalente a :requirements, existential predonditions y universal preconditions
	:adl
	:ucpop
	
	:numeric-fluents Permite usar funciones que representan valores numericos// En popf2 si esta soportado
	negative-preconditions Not en las conditions
	
	:derived-predicates
	:constraints A derived predicate is declared by naming the predicate who’s result is being derived, and a logical expression which is evaluated to work out the value.
	:preferences
	
	:action-costs
	:goal-utilities
	:time
