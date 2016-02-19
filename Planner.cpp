#include "Planner.h"
#include <assert.h>

//pointer to user defined heuristic function
template<typename state_information, typename action_information>
int (*planner::Planner<state_information, action_information>::HeuristicApproximation)(state_information*);

//
template<typename state_information, typename action_information>
void planner::Planner<state_information, action_information>::AddNextPossibleStates(
	state<state_information, action_information>* current_state)
{
	//next possible state temporary
	state_information *next_state_information;

	//cycle through all possible actions
	for (int i = 0; i < possible_actions.size(); i++) {
		//compune next possible state
		next_state_information = ApplyActionToState(current_state->state_information_pointer, Planner::possible_actions.at(i).action_information_pointer);

		//check is the state is valid and is not a previous state
		if (next_state_information != NULL && states_map[*next_state_information] == false) {
			//add state to state map so we don't comute it again
			states_map[*next_state_information] = true;

			//create a new state
			planner::state<state_information, action_information> *next_state = new state<state_information, action_information>(next_state_information);

			//load new state with information
			next_state->parent_state = current_state;
			next_state->action_from_parent = &(Planner::possible_actions.at(i));
			next_state->steps_from_initial_state = current_state->steps_from_initial_state + 1;

			//add state to state heap
			InsertToQueue(next_state);
		}
	}
}

template <typename state_information, typename action_information>
void planner::Planner<state_information, action_information>::InsertToQueue(state<state_information, action_information> *next_state) {
	Planner::states_queue.insert(
		std::make_pair(
			next_state->steps_from_initial_state + Planner::HeuristicApproximation(next_state->state_information_pointer),
			state_pointer<state_information, action_information>(next_state)
			)
		);
}

/*
compare two states using user-defined < operator
*/
template <typename state_information>
bool compare(state_information *first_state, state_information *second_state) {
	return *first_state < *second_state;
}

template <typename state_information, typename action_information>
planner::state<state_information, action_information>*
planner::Planner<state_information, action_information>::SearchAllStates_AStar(
	state<state_information, action_information> *initial_state,
	state_information *final_state) {
	//initialisation
	Planner::states_queue.clear ();
	Planner::states_map.clear();

	//add initial state to queue
	InsertToQueue(initial_state);
	Planner::states_map[*(initial_state->state_information_pointer)] = true;

	state<state_information, action_information> *current_state;
	std::set<std::pair<int, state_pointer<state_information, action_information> > >::iterator first_element;
	//cycle through the states heap
	for (; !states_queue.empty(); ) {
		//get the first state from the heap
		first_element = Planner::states_queue.begin ();
		current_state = first_element->second.getState();
		states_queue.erase(states_queue.begin());		

		//check if final state
		if (!compare<state_information>(current_state->state_information_pointer, final_state) && !compare<state_information>(final_state, current_state->state_information_pointer)) {
			return current_state;
		}

		planner::Planner<state_information, action_information>::AddNextPossibleStates(current_state);
	}

	return NULL;
}

template <typename state_information, typename action_information>
std::vector<std::pair<state_information*, action_information*> >*
planner::Planner<state_information, action_information>::run(
	state_information initial_state_information, state_information final_state_information,
	std::vector<action_information> *possible_actions,
	state_information*(*ApplyActionToState)(state_information*, action_information*),
	int(*HeuristicApproximation)(state_information*)
	) {

	assert(!compare<state_information>(&initial_state_information, &initial_state_information), "Bool opearator < must pe weak ordered!");
	assert(compare<state_information>(&initial_state_information, &final_state_information) != compare<state_information>(&final_state_information, &initial_state_information), "Bool opearator < is invalid!");

	this->HeuristicApproximation = HeuristicApproximation;
	this->ApplyActionToState = ApplyActionToState;

	Planner<state_information, action_information>::possible_actions.clear();
	for (unsigned int index = 0; index < possible_actions->size(); index++) {
		Planner<state_information, action_information>::possible_actions.push_back(action<action_information>(&(possible_actions->at(index))));
	}

	planner::state<state_information, action_information> *initial_state =
		new planner::state<state_information, action_information>(&initial_state_information);
	initial_state->parent_state = NULL;
	initial_state->action_from_parent = NULL;
	initial_state->steps_from_initial_state = 0;

	Planner::final_state_reached = SearchAllStates_AStar(initial_state, &final_state_information);

	return Planner::getSolution(final_state_reached);
}

template<typename state_information, typename action_information>
std::vector<std::pair<state_information*, action_information*> >* 
planner::Planner<state_information, action_information>::getSolution(
	state<state_information, action_information> *final_state
	) {

	state<state_information, action_information> *current_state = final_state;
	std::vector<std::pair<state_information*, action_information*> > *vector_state_information = new std::vector<std::pair<state_information*, action_information*> >();
	while (current_state->parent_state != NULL) {
		vector_state_information->push_back(std::make_pair(current_state->state_information_pointer, current_state->action_from_parent->action_information_pointer));
		current_state = current_state->parent_state;
	}

	std::reverse(vector_state_information->begin(), vector_state_information->end());
	return vector_state_information;
}