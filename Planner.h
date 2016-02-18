#include <iostream>
#include <fstream>
#include <map>
#include <set>
#include <vector>

namespace planner {
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
		std::vector<action<action_information> > possible_actions;
		std::map<state_information, bool> states_map;
		std::set<std::pair<int, state_pointer<state_information, action_information> > > states_queue;
		state<state_information, action_information> *final_state_reached;

		static int (*HeuristicApproximation)(state_information*);
		state_information* (*ApplyActionToState)(state_information*, action_information*);

		void AddNextPossibleStates(state<state_information, action_information>* current_state);
		state<state_information, action_information>* SearchAllStates(
			state<state_information, action_information> *initial_state,
			state_information *final_state);

		void InsertToQueue(state<state_information, action_information> *next_state);

	public:
		std::vector<std::pair<state_information*, action_information*> >* A_Star(
			state_information intial_state_information, state_information final_state_information,
			std::vector<action_information> *possible_actions,
			state_information*(*ApplyActionToState)(state_information*, action_information*),
			int(*HeuristicApproximation)(state_information*)
			);

		std::vector<std::pair<state_information*, action_information*> >* planner::Planner<state_information, action_information>::getSolution(
			state<state_information, action_information> *final_state);
	};
}