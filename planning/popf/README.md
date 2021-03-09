
POPF PLANNER

This planner works with PDDL 1.2 to PDDL 3.1

Here is the official repo : https://github.com/fmrico/popf.git


Its launched in ROS2. This planner has to be launched as:
	ros2 run popf popf [domain.pddl] [problem.pddl]
	


Suported actions/things:

	:strips - Allows add or delete effects
	:typing - Allows use types
	:equality - Allows compare if 2 objetives are the same
	:universal-preconditions - Allows use for all in goals and preconditions
	:durative-actions - 
	:duration-inecualities - Allows use inecualities to express duration 
	:continuous-effects - Allows use continious effects about numbers inside durative actions 
	:timed-initial-literals - A timed initial literal is defined using the time keyword, followed by the value for the point in time which the predicate becomes true, followed by the predicate itself.
	:numeric-fluents - Allows use functions which write numeric values // In popf1 is not supported in popf2 yes
	
	
	
	
Not supported actions/things:

	:disjunctive-preconditions - Allows all in preconditions
	:existential-preconditions - Allows exist in goals and preconditions
	:conditional-effects - Allows same action with different types
	:domain-axioms - Allows use axioms (:Derived (clear ?x
	:subgoals-through-axioms - Axioms as subgoals
	:safety-constraints - Allows define predicates which must be valid at the end of the execution
	:open-world - 
	:quantified-preconditions - 
	:adl - 
	:ucpop - 
	:derived-predicates - 
	
	:constraints A derived predicate is declared by naming the predicate who’s result is being derived, and a logical expression which is evaluated to work out the value.
	:preferences - 
	
	:action-costs - 
	:goal-utilities - 
	:time - 
	
	
	
