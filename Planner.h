#include <iostream>
#include <fstream>
#include <map>
#include <set>
#include <vector>

namespace planner {
	/*
	Holder for user defined actions
	*/
	template <typename action_information>
	struct action {
		action_information *action_information_pointer;

		action(action_information *action_information_pointer) {
			this->action_information_pointer = action_information_pointer;
		}

		action() {
			this->action_information_pointer = NULL;
		}
	};

	/*
	Holds a reference to a state. Works like a simple list.
	Works with user defined states.
	*/
	template <typename state_information, typename action_information>
	struct state {
		state<state_information, action_information>* parent_state;
		action<action_information>* action_from_parent;
		state_information *state_information_pointer;
		int steps_from_initial_state;

		bool operator <(const state<state_information, action_information>& another_state) const {
			return *(this->state_information_pointer) < *(another_state.state_information_pointer);
		}

		state<state_information, action_information>(state_information *current_state) {
			state_information_pointer = current_state;
		}
	};

	/*
	A class used solely to hold a pointer to a state.
	It overrides operator < for heuristic use.
	*/
	template <typename state_information, typename action_information>
	class state_pointer {
		state<state_information, action_information>* actual_state;

	public:
		state<state_information, action_information>* getState() const {
			return actual_state;
		}

		bool operator <(state_pointer<state_information, action_information> another_state_pointer) const {
			return *(getState()) < *(another_state_pointer.getState());
		}

		state_pointer<state_information, action_information>(state<state_information, action_information> *state) {
			this->actual_state = state;
		}
	};

	template <typename state_information, typename action_information>
	class Planner {
		/*
		User defined possible actions that can be applied on a state
		*/
		std::vector<action<action_information> > possible_actions;

		/*
		A hastable for visited states
		*/
		std::map<state_information, bool> states_map;

		/*
		Priority queue used to choose the next state to expand
		*/
		std::set<std::pair<int, state_pointer<state_information, action_information> > > states_queue;

		/*
		The final state reached after calling run function.
		*/
		state<state_information, action_information> *final_state_reached;

		/*
		Pointer to user defined heuristic function
		*/
		static int (*HeuristicApproximation)(state_information*);
		
		/*
		Pointer to user defined state transition function.
		This function takes a state and an action and return the resulting state
		or a null pointer if the resulting state is invalid
		*/
		state_information* (*ApplyActionToState)(state_information*, action_information*);

		/*
		Add to states queue all possible new states that can be reached from a given state
		*/
		void AddNextPossibleStates(state<state_information, action_information>* current_state);

		/*
		Search through the possible states graph using A* method
		*/
		state<state_information, action_information>* SearchAllStates_AStar(
			state<state_information, action_information> *initial_state,
			state_information *final_state);

		/*
		Add a state to queue
		*/
		void InsertToQueue(state<state_information, action_information> *next_state);

		/*
		Returns a stl vector containing the path to the final state from an initial state.
		This function should only be called internally.
		*/
		std::vector<std::pair<state_information*, action_information*> >*
			planner::Planner<state_information, action_information>::getSolution(
				state<state_information, action_information> *final_state);

	public:
		/*
		Given an initial state and a state to find, and user defined state transition function and heuristic,
		this function return the path found for the solution.
		The return value is a stl vector of pairs of a state and the action used to reach that state from a previous one
		*/
		std::vector<std::pair<state_information*, action_information*> >* run(
			state_information intial_state_information, state_information final_state_information,
			std::vector<action_information> *possible_actions,
			state_information*(*ApplyActionToState)(state_information*, action_information*),
			int(*HeuristicApproximation)(state_information*)
			);
	};
}